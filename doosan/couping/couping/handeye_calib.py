"""핸드-아이 캘리브레이션 측정 도구 (카메라 고정 + 그리퍼에 ArUco 마커).

준비
  1) 마커 인쇄: ~/venv/yolo/bin/python ~/couping_ws/tools/make_aruco_marker.py  → 100% 크기로 인쇄
  2) 마커를 그리퍼 끝에 평평하게 붙이고 카메라 쪽을 보게 함 (TCP 에 가까울수록 좋음)
  3) 두산 bringup 실행 (dsr_bringup2_rviz.launch.py mode:=real ...)
실행 (ROS 환경 source 후)
  ~/venv/yolo/bin/python -m couping.handeye_calib
  ~/venv/yolo/bin/python -m couping.handeye_calib --compute   # 저장된 표본으로 다시 계산만
조작 (창에서)
  로봇을 티치펜던트·직접교시로 옮기고 멈춘 뒤  SPACE: 표본 저장   u: 마지막 표본 취소
  c: 계산·저장   q: 종료
표본 모으는 법 (12~15개)
  - 진열대 앞 작업 공간 전체(좌우·위아래·앞뒤)에 골고루
  - 손목을 여러 방향으로 20~40도씩 기울인 자세를 절반 이상 섞기 (안 그러면 마커-TCP 거리를 못 구함)
  - 마커가 카메라에서 잘 보이게 (창에 초록 테두리)
결과: ~/couping_ws/calib/handeye.json   (shelf_vision 이 자동으로 읽음)
표본: ~/couping_ws/calib/handeye_samples.json (표본마다 바로 저장)
"""
import argparse
import json
import os
import time

import cv2
import numpy as np

from couping import handeye as he

CALIB_DIR = os.path.expanduser('~/couping_ws/calib')
SAMPLES = os.path.join(CALIB_DIR, 'handeye_samples.json')
RESULT = os.path.join(CALIB_DIR, 'handeye.json')


def marker_center(det, gray, depth, cam, marker_id):
    """ArUco 를 찾아 중심 픽셀과 깊이로 3D(m) → (xyz, 모서리) 또는 (None, 모서리)."""
    corners, ids, _ = det.detectMarkers(gray)
    if ids is None or marker_id not in ids.ravel():
        return None, None
    if list(ids.ravel()).count(marker_id) > 1:  # 같은 번호 마커가 둘 이상 보이면 어느 쪽인지 알 수 없음
        return None, None
    c = corners[list(ids.ravel()).index(marker_id)].reshape(4, 2).astype(np.float32)
    cv2.cornerSubPix(gray, c, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
    ctr = c.mean(0)
    inner = (ctr + (c - ctr) * 0.6).astype(np.int32)  # 마커 안쪽 60% (가장자리 깊이 튐 제외)
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, inner, 1)
    vals = depth[(mask > 0) & (depth > 0)]
    if vals.size < 20:
        return None, c
    return np.array(cam.deproject(ctr[0], ctr[1], np.median(vals))), c


def make_detector():
    """비스듬하거나 작게 보이는 마커도 잡도록 기본값보다 너그럽게."""
    p = cv2.aruco.DetectorParameters()
    p.adaptiveThreshWinSizeMin, p.adaptiveThreshWinSizeMax, p.adaptiveThreshWinSizeStep = 3, 53, 4
    p.minMarkerPerimeterRate = 0.01          # 작은 마커 허용
    p.polygonalApproxAccuracyRate = 0.05     # 비스듬해서 찌그러진 사각형 허용
    p.perspectiveRemovePixelPerCell = 8
    p.errorCorrectionRate = 0.8
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), p)


class Robot:
    """두산 TCP 자세 읽기 (/dsr01/dsr_controller2/aux_control/get_current_posx)."""

    def __init__(self, service):
        import rclpy
        from dsr_msgs2.srv import GetCurrentPosx
        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node('handeye_calib')
        self.cli = self.node.create_client(GetCurrentPosx, service)
        self.Req = GetCurrentPosx.Request
        self.service = service

    def ready(self):
        return self.cli.service_is_ready()

    def posx(self):
        req = self.Req()
        req.ref = 0  # DR_BASE
        fut = self.cli.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
        r = fut.result()
        if r is None or not r.success:
            return None
        return [float(v) for v in r.task_pos_info[0].data[:6]]

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def load_samples():
    if os.path.exists(SAMPLES):
        with open(SAMPLES) as f:
            return json.load(f)
    return []


def save_samples(samples):
    os.makedirs(CALIB_DIR, exist_ok=True)
    with open(SAMPLES, 'w') as f:
        json.dump(samples, f, indent=1)


def compute(samples, prior):
    if len(samples) < 6:
        print(f'표본이 {len(samples)}개라 부족합니다 (최소 6, 권장 12~15).')
        return None
    P = np.array([s['p_cam'] for s in samples])
    T = np.array([he.posx_to_T(s['posx']) for s in samples])
    res = he.solve(P, T, prior=prior)
    loo = he.leave_one_out(P, T, prior=prior)
    print(f"\n표본 {len(samples)}개 | 손목 방향 차이 최대 {res['orientation_spread_deg']:.0f}도")
    for i, (s, e, l) in enumerate(zip(samples, res['errors_m'], loo), 1):
        flag = '  <-- 큼' if l > 0.005 else ''
        print(f"  #{i:2d} posx {np.round(s['posx'], 1)}  잔차 {e * 1000:4.1f}mm  빼고예측 {l * 1000:4.1f}mm{flag}")
    print(f"잔차 RMS {res['rms_m'] * 1000:.1f}mm (최대 {res['max_m'] * 1000:.1f})"
          f" | 빼고예측 RMS {np.sqrt(np.mean(loo ** 2)) * 1000:.1f}mm (최대 {loo.max() * 1000:.1f})  ← 실제 정확도에 가까운 값")
    print(f"마커 위치(그리퍼 좌표계, mm): {np.round(res['marker_offset'] * 1000, 1)}")
    if not res['offset_estimated']:
        print('\n[중단] 손목 방향 변화가 15도 미만이라 마커-TCP 거리를 구할 수 없습니다.'
              '\n       이대로 저장하면 마커가 TCP 에서 떨어진 만큼 좌표가 통째로 어긋납니다.'
              '\n       손목을 20~40도 기울인 자세를 몇 개 더 모으거나, 직접 잰 값을 --offset 으로 주세요.')
        if prior == (0, 0, 0):
            return None
    out = he.save(RESULT, res, loo, len(samples), note=time.strftime('%Y-%m-%d %H:%M'))
    print(f"\n저장: {RESULT}\n카메라 위치(로봇 베이스 기준, mm): {np.round(np.array(out['translation_m']) * 1000, 1)}")
    if out['loo_rms_mm'] > 5:
        print('[주의] 빼고예측 오차가 5mm 를 넘습니다. 큰 표본(<-- 큼)을 u 로 지우고 다시 모으세요.')
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--service', default='/dsr01/dsr_controller2/aux_control/get_current_posx')
    ap.add_argument('--marker-id', type=int, default=0)
    ap.add_argument('--frames', type=int, default=15, help='표본 1개당 평균 낼 프레임 수')
    ap.add_argument('--compute', action='store_true', help='측정 없이 저장된 표본으로 계산만')
    ap.add_argument('--offset', type=float, nargs=3, default=(0.0, 0.0, 0.0),
                    help='직접 잰 마커 위치(그리퍼 좌표계, m). 손목을 못 기울일 때만')
    ap.add_argument('--reset', action='store_true', help='저장된 표본을 지우고 새로 시작')
    args = ap.parse_args()
    prior = tuple(args.offset)

    if args.compute:
        compute(load_samples(), prior)
        return
    samples = [] if args.reset else load_samples()
    if samples:
        print(f'이전 표본 {len(samples)}개를 이어서 씁니다 (새로 하려면 --reset)')

    from couping.shelf_vision import RealSense
    cam = RealSense(1280, 720, 30)
    robot = Robot(args.service)
    det = make_detector()
    win = 'handeye calib'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)
    msg, last_dump = 'move robot, stop, press SPACE', 0.0
    try:
        while True:
            f = cam.grab()
            if f is None:
                continue
            img, depth = f
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            p, c = marker_center(det, gray, depth, cam, args.marker_id)
            vis = img.copy()
            if c is not None:
                cv2.polylines(vis, [c.astype(np.int32)], True, (0, 255, 0) if p is not None else (0, 165, 255), 3)
            ok_robot = robot.ready()
            status = (f"samples {len(samples)} | marker {'OK' if p is not None else 'NOT FOUND'}"
                      f" | robot {'OK' if ok_robot else 'NO SERVICE (bringup?)'}")
            if p is not None:
                status += f' | cam xyz(mm) {np.round(p * 1000).astype(int).tolist()}'
            cv2.putText(vis, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(vis, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis, msg, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(vis, 'SPACE=save  u=undo  c=compute  q=quit', (10, 700), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(win, vis)
            if time.time() - last_dump > 2:  # 원격 확인용
                cv2.imwrite(os.path.join(CALIB_DIR, 'latest_view.jpg'), cv2.resize(vis, None, fx=0.5, fy=0.5))
                last_dump = time.time()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('u') and samples:
                samples.pop()
                save_samples(samples)
                msg = f'undo -> {len(samples)} samples'
            if key == ord('c'):
                out = compute(samples, prior)
                msg = (f"saved: LOO RMS {out['loo_rms_mm']}mm" if out else 'not saved (see terminal)')
            if key == ord(' '):
                msg = capture(cam, det, robot, args, samples)
                print(msg, flush=True)
    finally:
        cam.close()
        robot.close()
        cv2.destroyAllWindows()


def capture(cam, det, robot, args, samples):
    if not robot.ready():
        return 'robot service not ready'
    q1 = robot.posx()
    pts = []
    for _ in range(args.frames):
        f = cam.grab()
        if f is None:
            continue
        p, _ = marker_center(det, cv2.cvtColor(f[0], cv2.COLOR_BGR2GRAY), f[1], cam, args.marker_id)
        if p is not None:
            pts.append(p)
    q2 = robot.posx()
    if q1 is None or q2 is None:
        return 'robot pose read failed'
    if np.linalg.norm(np.subtract(q1[:3], q2[:3])) > 0.5 or np.abs(np.subtract(q1[3:], q2[3:])).max() > 0.2:
        return 'robot is moving - stop and retry'
    if len(pts) < args.frames * 0.5:
        return f'marker seen only {len(pts)}/{args.frames} frames - retry'
    pts = np.array(pts)
    # 곡면·반사 때문에 깊이가 가끔 튀는 프레임은 버림 (중앙값에서 5mm 넘게 떨어진 것)
    med = np.median(pts, 0)
    good = pts[np.linalg.norm(pts - med, axis=1) < 0.005]
    if len(good) < max(5, len(pts) * 0.5):
        return f'marker depth jumps too much ({len(good)}/{len(pts)} stable) - face marker to camera'
    sd = good.std(0).max() * 1000
    if sd > 2.0:
        return f'marker position unstable ({sd:.1f}mm) - retry'
    samples.append({'posx': q2, 'p_cam': np.median(good, 0).tolist(), 'std_mm': round(sd, 2),
                    'inliers': f'{len(good)}/{len(pts)}',
                    'time': time.strftime('%H:%M:%S')})
    save_samples(samples)
    return f'saved #{len(samples)}  posx {np.round(q2, 1).tolist()}  std {sd:.1f}mm'


if __name__ == '__main__':
    main()
