"""핸드-아이 캘리브레이션 계산 (eye-to-hand: 카메라 고정, 마커는 그리퍼에).

numpy 만 사용 (이 환경의 OpenCV 5 에는 calibrateHandEye 가 없고 scipy 도 없음).

모델
  p_base = R_X @ p_cam + t_X                         (구하려는 카메라→로봇 베이스 변환)
  마커 중심 = 그리퍼 자세 T_bg 로 옮긴 오프셋 t_Y      (그리퍼 좌표계에서 본 마커 위치, 함께 추정)
  표본 i 마다:  R_X @ p_i + t_X  =  R_bg_i @ t_Y + t_bg_i
마커의 '위치'만 쓰고 회전은 쓰지 않는다 (작은 마커의 회전 추정은 노이즈가 커서).
t_Y 는 그리퍼 방향이 표본마다 달라야(손목을 여러 각도로 기울여야) 구해진다.
방향이 거의 같으면 t_Y 는 사전값(prior, 기본 0 = 마커가 TCP 에 있음)에 묶인다.

두산 posx: [x, y, z, a, b, c]  mm, deg, ZYZ 오일러 (R = Rz(a) Ry(b) Rz(c)).
"""
import json
import math
import os

import numpy as np


# ---------- 회전 ----------
def rot_z(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rot_y(d):
    c, s = math.cos(math.radians(d)), math.sin(math.radians(d))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def zyz_to_R(a, b, c):
    return rot_z(a) @ rot_y(b) @ rot_z(c)


def posx_to_T(posx):
    """두산 posx (mm, deg ZYZ) → 4x4 (m)."""
    T = np.eye(4)
    T[:3, :3] = zyz_to_R(*posx[3:6])
    T[:3, 3] = np.asarray(posx[:3], float) / 1000.0
    return T


def rotvec_to_R(v):
    th = np.linalg.norm(v)
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def R_to_rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if math.pi - th < 1e-6:  # 180도 근처
        w, V = np.linalg.eigh((R + np.eye(3)) / 2)
        return V[:, np.argmax(w)] * th
    return th / (2 * math.sin(th)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def R_to_quat(R):
    """(x, y, z, w)"""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def kabsch(P, Q):
    """Q ≈ R P + t 를 만족하는 R, t (최소제곱)."""
    cp, cq = P.mean(0), Q.mean(0)
    U, _, Vt = np.linalg.svd((P - cp).T @ (Q - cq))
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cq - R @ cp


# ---------- 풀이 ----------
def _residuals(x, P, Rbg, tbg, prior, w_prior):
    R, tX, tY = rotvec_to_R(x[:3]), x[3:6], x[6:9]
    r = (P @ R.T + tX) - (np.einsum('nij,j->ni', Rbg, tY) + tbg)
    return np.concatenate([r.ravel(), w_prior * (tY - prior)])


def orientation_spread_deg(Rbg):
    """표본들 그리퍼 방향의 최대 차이 (deg). 작으면 마커 오프셋을 구할 수 없음."""
    return max((math.degrees(np.linalg.norm(R_to_rotvec(Rbg[i].T @ Rbg[j])))
                for i in range(len(Rbg)) for j in range(i + 1, len(Rbg))), default=0.0)


def solve(p_cam, T_bg, prior=(0, 0, 0), min_spread_deg=15.0, iters=50):
    """p_cam: (N,3) m 카메라 좌표 마커 중심, T_bg: (N,4,4) 그리퍼 자세 → 결과 dict."""
    P = np.asarray(p_cam, float)
    T_bg = np.asarray(T_bg, float)
    Rbg, tbg = T_bg[:, :3, :3], T_bg[:, :3, 3]
    prior = np.asarray(prior, float)
    spread = orientation_spread_deg(Rbg)
    # 방향 변화가 작으면 오프셋을 사전값에 강하게 묶음, 충분하면 아주 약하게(수치 안정용)
    w_prior = 10.0 if spread < min_spread_deg else 1e-3

    R0, t0 = kabsch(P, tbg + np.einsum('nij,j->ni', Rbg, prior))
    x = np.concatenate([R_to_rotvec(R0), t0, prior])
    lam = 1e-3
    f = lambda x: _residuals(x, P, Rbg, tbg, prior, w_prior)
    r = f(x)
    for _ in range(iters):  # Levenberg-Marquardt, 수치 미분
        J = np.empty((r.size, 9))
        for k in range(9):
            d = np.zeros(9)
            d[k] = 1e-6
            J[:, k] = (f(x + d) - f(x - d)) / 2e-6
        g, A = J.T @ r, J.T @ J
        step = -np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), g)
        r_new = f(x + step)
        if r_new @ r_new < r @ r:
            x, r, lam = x + step, r_new, lam * 0.3
            if np.linalg.norm(step) < 1e-10:
                break
        else:
            lam *= 10
    R, tX, tY = rotvec_to_R(x[:3]), x[3:6], x[6:9]
    res = (P @ R.T + tX) - (np.einsum('nij,j->ni', Rbg, tY) + tbg)
    err = np.linalg.norm(res, axis=1)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, tX
    return {'T_base_cam': T, 'marker_offset': tY, 'errors_m': err,
            'rms_m': float(np.sqrt(np.mean(err ** 2))), 'max_m': float(err.max()),
            'orientation_spread_deg': spread, 'offset_estimated': spread >= min_spread_deg}


def leave_one_out(p_cam, T_bg, **kw):
    """표본 하나씩 빼고 풀어서 뺀 표본을 예측했을 때의 오차 (m). 실제 정확도에 가까운 값."""
    P, T_bg = np.asarray(p_cam, float), np.asarray(T_bg, float)
    out = []
    for i in range(len(P)):
        m = np.arange(len(P)) != i
        s = solve(P[m], T_bg[m], **kw)
        T, tY = s['T_base_cam'], s['marker_offset']
        pred = T[:3, :3] @ P[i] + T[:3, 3]
        true = T_bg[i, :3, :3] @ tY + T_bg[i, :3, 3]
        out.append(np.linalg.norm(pred - true))
    return np.array(out)


# ---------- 저장 / 불러오기 ----------
def save(path, result, loo=None, n=None, note=''):
    T = result['T_base_cam']
    data = {
        'T_base_cam': T.round(6).tolist(),
        'translation_m': T[:3, 3].round(6).tolist(),
        'quaternion_xyzw': [round(v, 6) for v in R_to_quat(T[:3, :3])],
        'marker_offset_m': np.round(result['marker_offset'], 5).tolist(),
        'offset_estimated': bool(result['offset_estimated']),
        'orientation_spread_deg': round(result['orientation_spread_deg'], 1),
        'samples': n,
        'rms_mm': round(result['rms_m'] * 1000, 2),
        'max_mm': round(result['max_m'] * 1000, 2),
        'loo_rms_mm': None if loo is None else round(float(np.sqrt(np.mean(loo ** 2))) * 1000, 2),
        'loo_max_mm': None if loo is None else round(float(loo.max()) * 1000, 2),
        'note': note,
    }
    os.makedirs(os.path.dirname(os.path.expanduser(path)), exist_ok=True)
    with open(os.path.expanduser(path), 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return data


def load(path):
    """저장된 변환 → (4x4 numpy, 전체 dict). 파일이 없으면 (None, None)."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None, None
    if path.endswith('.npy'):  # 강의 캘리브레이션 GUI 결과 (camera→base 4x4, m)
        T = np.load(path)
        info = {'samples': None, 'loo_rms_mm': None, 'note': 'HamdEyeCal'}
        rep = os.path.join(os.path.dirname(path), 'calibration_report.txt')
        if os.path.exists(rep):
            for line in open(rep, encoding='utf-8'):
                if line.startswith('samples:'):
                    info['samples'] = int(line.split(':')[1])
                if line.startswith('verify std'):
                    info['loo_rms_mm'] = line.split('max')[1].split(',')[0].strip() if 'max' in line else None
        return T, info
    with open(path) as f:
        data = json.load(f)
    return np.array(data['T_base_cam'], float), data
