"""자동 피킹 한 사이클 (auto_pick CLI · 두산 노드 공용).

  관찰 자세에서 인식 → 진열대 앞 수평 자세(front) → 컵 정면 approach 앞 → 천천히 수평 전진
  → 잡기 → 들기 → 수평으로 빼기 → front → (핑키 자세가 저장돼 있으면) 내려놓기 → 관찰 자세
  핑키 자세가 없을 때: no_place='hold' 면 front 에서 들고 대기, 'return' 이면 진열대 제자리에 도로 놓음
  주문의 k번째 품목은 place_k 자세(1번째는 place)에 놓는다. 자리가 모자라면 움직이기 전에 거부.

필요한 것
  - bringup, gripper_service_a, 비전 노드(shelf_vision, 캘리브레이션 적용)
  - ~/couping_ws/calib/grasp_offset.json  (shelf_pick_test --teach 로 저장한 플랜지→잡는 점)
  - ~/couping_ws/calib/poses.json        (teach_pose 로 저장한 front, [place_above, place])
움직이기 전에 모든 단계의 도달거리·높이·방향을 먼저 검사하고, 하나라도 걸리면 아무것도 하지 않는다.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from std_srvs.srv import Trigger

from couping import poses
from couping.handeye import zyz_to_R

OFFSET_FILE = os.path.expanduser('~/couping_ws/calib/grasp_offset.json')

DEFAULTS = {
    'open': 375,            # 접근할 때 그리퍼 열림 (0 닫힘 ~ 750 완전 열림)
    'release': 375,         # 내려놓을 때 여는 정도
    'force': 300,           # 잡는 힘
    'grip_wait': 3.0,       # 그리퍼를 닫은 뒤 다 잡을 때까지 기다리는 시간 s (이후에 들어올림)
    'approach': 150.0,      # 컵 정면 대기 거리 mm
    'depth': 7.0,           # 컵 중심 대비 추가 전진 mm (10/15 tide 시험: 7mm 더 들어가야 잘 잡힘)
    'dz': 0.0,              # 목표 높이 보정 mm
    'lift': 15.0,           # 잡은 뒤 들어올림 mm
    'vel': 60.0, 'acc': 60.0,           # 일반 직선 이동 mm/s
    'vel_slow': 20.0, 'acc_slow': 20.0,  # 컵 근처 이동 mm/s
    'velj': 30.0, 'accj': 30.0,         # 관절 이동 deg/s
    'shelf_ymin': 380.0,    # 이보다 로봇 쪽 검출은 진열대 밖(그리퍼 오인식)으로 보고 제외
    'reach_min': 200.0, 'reach_max': 850.0, 'z_min': 50.0,
    'retries': 2,           # 상자에 넣은 뒤 카메라로 확인해 실패면 다시 시도하는 횟수
    'verify_radius': 50.0,  # 집은 자리에서 이 거리 안에 같은 품목이 또 보이면 못 집은 것으로 판단 (mm)
}


class PickError(Exception):
    pass


class NotPickable(PickError):
    """이 품목만 못 집는 경우(진열대에 없음·도달 불가). 로봇은 관찰 자세라 다음 품목은 계속할 수 있다."""


class PickFailed(NotPickable):
    """다시 시도해도 카메라 확인에서 계속 실패. 로봇은 관찰 자세라 다음 품목은 계속할 수 있다."""


# ---------- 인식 ----------
def detect(node, shelf_ymin, timeout=120.0):
    cli = node.create_client(Trigger, '/couping/vision/detect')
    if not cli.wait_for_service(timeout_sec=5.0):
        raise PickError('비전 노드가 없습니다 (ros2 launch couping vision.launch.py)')
    fut = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None or not res.success:
        raise PickError(f'비전 인식 실패: {None if res is None else res.message}')
    data = json.loads(res.message)
    if not data.get('calibrated'):
        raise PickError('비전 노드에 캘리브레이션이 적용되지 않았습니다')
    return [it for it in data['items'] if it['base_mm'] and it['base_mm'][1] >= shelf_ymin]


def choose(items, cls, row=None, col=None):
    """주문 품목 중 로봇에 가장 가까운(도달 여유가 큰) 것."""
    c = [it for it in items if it['cls'] == cls
         and (row is None or it['row'] == row) and (col is None or it['col'] == col)]
    if not c:
        raise NotPickable(f'{cls} 를 진열대에서 찾지 못했습니다 (인식: {sorted({it["cls"] for it in items})})')
    return min(c, key=lambda it: math.hypot(it['base_mm'][0], it['base_mm'][1]))


# ---------- 계획 ----------
def plan(target, p, slot=1, no_place='hold'):
    """목표 컵 중심(base mm) → (실행 단계 목록, 'placed'|'returned'|'holding').
    문제가 있으면 아무것도 움직이기 전에 PickError."""
    front = poses.get('front')
    if front is None:
        raise PickError('front 자세가 없습니다 → python3 -m couping.teach_pose front')
    if not os.path.exists(OFFSET_FILE):
        raise PickError('잡는 점 오프셋이 없습니다 → shelf_pick_test --teach')
    off_data = json.load(open(OFFSET_FILE))
    tool_off = np.array(off_data['offset_tool_mm'], float)
    # 현장에서 본 어긋남 보정 (base mm, 예: [0, 0, -30] = 30mm 낮게 잡기)
    adjust = np.array(off_data.get('target_adjust_mm', [0.0, 0.0, 0.0]), float)

    fx = np.array(front['posx'], float)
    R = zyz_to_R(*fx[3:6])
    tz = R[:, 2]
    if abs(math.degrees(math.asin(max(-1.0, min(1.0, tz[2]))))) > 15:
        raise PickError('front 자세의 그리퍼가 수평이 아닙니다 (다시 teach_pose front)')
    d = np.array([tz[0], tz[1], 0.0])
    d /= np.linalg.norm(d)                      # 수평 접근 방향
    off = R @ tool_off                          # 플랜지 → 잡는 점 (base)
    ori = list(fx[3:6])                         # 모든 직선 이동은 front 와 같은 방향

    target = np.asarray(target, float) + adjust
    grasp = target + d * p['depth'] + np.array([0, 0, p['dz']])
    pre = grasp - d * p['approach']
    near = grasp - d * 40.0
    up = np.array([0, 0, p['lift']])
    front_tip = fx[:3] + off
    if np.dot(pre - front_tip, d) < -1.0:
        raise PickError(f'front 자세가 진열대에 너무 가깝습니다 (대기선보다 {-np.dot(pre - front_tip, d):.0f}mm 앞). '
                        '진열대에서 더 떨어진 곳에서 다시 teach_pose front')

    def L(desc, tip, slow=False):
        return {'kind': 'movel', 'desc': desc, 'tip': np.asarray(tip, float), 'flange': np.asarray(tip, float) - off,
                'ori': ori, 'slow': slow}

    steps = [
        {'kind': 'movej', 'desc': '진열대 앞 수평 자세(front)', 'posj': front['posj']},
        L(f"컵 정면 {p['approach']:.0f}mm 앞 (높이·좌우 맞춤)", pre),
        {'kind': 'grip', 'desc': f"그리퍼 열기 ({p['open']})", 'pos': p['open'], 'cur': 200},
        L('컵 40mm 앞까지 전진', near, slow=True),
        L('컵 위치까지 전진', grasp, slow=True),
        {'kind': 'grip', 'desc': f"그리퍼 닫기 (힘 {p['force']}, {p['grip_wait']:.1f}초 대기)", 'pos': 0, 'cur': p['force'],
         'wait': p['grip_wait']},  # 다 잡은 뒤에 들어올림 (닫으면서 올라가지 않게)
        L(f"{p['lift']:.0f}mm 들어올리기", grasp + up, slow=True),
        L('수평으로 빼기', pre + up, slow=True),
    ]
    to_front = {'kind': 'movel_x', 'desc': 'front 로 복귀', 'x': list(fx)}
    to_observe = {'kind': 'movej', 'desc': '관찰 자세로 복귀', 'posj': poses.get('observe')['posj']}
    release = {'kind': 'grip', 'desc': f"그리퍼 열어 내려놓기 ({p['release']})", 'pos': p['release'], 'cur': 200}
    above, place = poses.get('place_above'), poses.place_for(slot)
    if above and poses.get('place'):
        if place is None:  # 앞 품목 위에 겹쳐 놓지 않도록
            raise PickError(f'{slot}번째 품목을 놓을 place_{slot} 자세가 없습니다 → teach_pose place_{slot}')
        if above['posx'][2] < place['posx'][2] + 20:
            raise PickError('place_above 가 place 보다 20mm 이상 높아야 합니다 (teach_pose 다시)')
        over = list(place['posx'])
        over[2] = above['posx'][2]  # 놓을 자리 바로 위 (place_above 높이)에서 수직으로 내림
        steps += [
            to_front,
            {'kind': 'movej', 'desc': '핑키 위로 이동', 'posj': above['posj']},
            {'kind': 'movel_x', 'desc': f'{slot}번 자리 위로', 'x': over},
            {'kind': 'movel_x', 'desc': f'{slot}번 자리에 내리기', 'x': place['posx'], 'slow': True},
            release,
            {'kind': 'movel_x', 'desc': '위로 빠지기', 'x': over, 'slow': True},
            to_observe,
        ]
        result = 'placed'
    elif no_place == 'return':  # 핑키 위치 미정: 진열대 제자리에 도로 놓고 관찰 자세로 (반복 시험용)
        steps += [
            L('진열대에 도로 넣기', grasp + up, slow=True),
            L(f"{p['lift']:.0f}mm 내리기", grasp, slow=True),
            release,
            L('수평으로 빼기', pre, slow=True),
            to_front,
            to_observe,
        ]
        result = 'returned'
    else:  # 핑키 위치 미정: front 에서 들고 대기
        steps.append(to_front)
        result = 'holding'

    for s in steps:  # 안전 검사 (움직이기 전에 전부)
        if s['kind'] != 'movel':
            continue
        f = s['flange']
        reach = math.hypot(f[0], f[1])
        if not p['reach_min'] < reach < p['reach_max']:
            raise NotPickable(f"[{s['desc']}] 도달거리 {reach:.0f}mm 범위 밖")
        if s['tip'][2] < p['z_min']:
            raise NotPickable(f"[{s['desc']}] 그리퍼 끝 높이 {s['tip'][2]:.0f}mm < {p['z_min']:.0f}")
    return steps, result


def check_slots(n):
    """핑키에 놓는 모드면 주문 품목 n개를 놓을 자리(place, place_2..)가 다 있는지 움직이기 전에 확인."""
    if poses.get('place_above') and poses.get('place'):
        missing = [f'place_{k}' for k in range(2, n + 1) if poses.get(f'place_{k}') is None]
        if missing:
            raise PickError(f'품목 {n}개를 놓을 자리가 모자랍니다: {", ".join(missing)} 없음 '
                            '(python3 -m couping.teach_pose place_2 …)')


def describe(steps):
    lines = []
    for i, s in enumerate(steps, 1):
        if s['kind'] == 'movel':
            lines.append(f"  {i:2d}. {s['desc']:30s} 그리퍼끝 {np.round(s['tip'])}  플랜지 {np.round(s['flange'])}")
        elif s['kind'] == 'movej':
            lines.append(f"  {i:2d}. {s['desc']:30s} 관절 {np.round(s['posj'], 1)}")
        elif s['kind'] == 'movel_x':
            lines.append(f"  {i:2d}. {s['desc']:30s} TCP {np.round(s['x'][:3])}")
        else:
            lines.append(f"  {i:2d}. {s['desc']}")
    return '\n'.join(lines)


# ---------- 실행 ----------
def execute(dsr, steps, p, log=print, confirm=None):
    from dsr_gripper import gripper_cmd
    for i, s in enumerate(steps, 1):
        if confirm:
            confirm(f"{i}. {s['desc']}")
        log(f"[{i}/{len(steps)}] {s['desc']}")
        if s['kind'] == 'grip':
            if not gripper_cmd(int(s['pos']), int(s['cur'])):
                raise PickError(f"그리퍼 명령 실패: {s['desc']}")
            time.sleep(s.get('wait', 1.5))
            continue
        if s['kind'] == 'movej':
            ret = dsr.movej(dsr.posj(*s['posj']), vel=p['velj'], acc=p['accj'])
        else:
            x = list(s['flange']) + list(s['ori']) if s['kind'] == 'movel' else list(s['x'])
            slow = s.get('slow')
            ret = dsr.movel(dsr.posx(*[float(v) for v in x]),
                            vel=p['vel_slow'] if slow else p['vel'], acc=p['acc_slow'] if slow else p['acc'])
        if ret != 0:
            raise PickError(f"이동 실패 (ret={ret}): {s['desc']}")


def run_pick(node, dsr, item, p=None, row=None, col=None, log=print, confirm=None, dry_run=False,
             slot=1, no_place='hold'):
    """주문 품목 하나를 집는다. 성공 시 결과 dict, 실패 시 PickError (그 품목만 못 집으면 NotPickable)."""
    p = {**DEFAULTS, **(p or {})}
    if not dry_run:
        dsr.set_robot_mode(dsr.ROBOT_MODE_AUTONOMOUS)
        time.sleep(0.5)
        log('관찰 자세로 이동')
        if dsr.movej(dsr.posj(*poses.get('observe')['posj']), vel=p['velj'], acc=p['accj']) != 0:
            raise PickError('관찰 자세 이동 실패')
        time.sleep(0.5)  # 흔들림 가라앉기
    items = detect(node, p['shelf_ymin'])
    tgt = choose(items, item, row, col)
    log(f"목표: r{tgt['row']}c{tgt['col']} {tgt['cls']} 컵 중심 {np.round(tgt['base_mm'], 1)} (신뢰도 {tgt['conf']:.2f})")
    steps, result = plan(tgt['base_mm'], p, slot, no_place)
    log(describe(steps))
    if result == 'holding':
        log('※ 핑키 자세(place_above, place)가 아직 없어 front 에서 들고 대기합니다')
    elif result == 'returned':
        log('※ 핑키 자세(place_above, place)가 아직 없어 진열대 제자리에 도로 놓습니다')
    out = {'target': tgt, 'result': result, 'placed': result == 'placed', 'before': items}
    if dry_run:
        return {**out, 'dry_run': True}
    execute(dsr, steps, p, log, confirm)
    return out


# ---------- 카메라로 확인하고 다시 시도 ----------
def verify_pick(node, res, p, log=print):
    """상자에 넣고 관찰 자세로 돌아온 뒤 다시 인식해서, 집은 컵이 진열대에서 빠졌는지 확인.
    실패 판단: (1) 집은 자리 verify_radius 안에 같은 품목이 아직 있음, (2) 같은 품목 개수가 줄지 않음."""
    tgt, before = res['target'], res['before']
    time.sleep(0.5)  # 흔들림 가라앉기
    after = detect(node, p['shelf_ymin'])
    same = [it for it in after if it['cls'] == tgt['cls']]
    t = np.asarray(tgt['base_mm'], float)
    near = [float(np.linalg.norm(np.asarray(it['base_mm'], float) - t)) for it in same]
    if near and min(near) < p['verify_radius']:
        return False, f"집은 자리에 아직 있음 ({min(near):.0f}mm)"
    n_before = sum(1 for it in before if it['cls'] == tgt['cls'])
    if len(same) >= n_before:
        return False, f"진열대 개수가 그대로 ({n_before}개 → {len(same)}개)"
    log(f"확인 OK: {tgt['cls']} {n_before}개 → {len(same)}개")
    return True, ''


def pick_verified(node, dsr, item, p=None, row=None, col=None, log=print, confirm=None, dry_run=False,
                  slot=1, no_place='hold', on_retry=None):
    """run_pick + 카메라 확인. 상자에 넣었는데 진열대에 그대로 있으면 retries 번까지 다시 시도.
    끝까지 실패하면 PickFailed (로봇은 관찰 자세)."""
    p = {**DEFAULTS, **(p or {})}
    tries = int(p['retries']) + 1
    why = ''
    for k in range(1, tries + 1):
        res = run_pick(node, dsr, item, p, row, col, log, confirm, dry_run, slot, no_place)
        if dry_run or res['result'] != 'placed':  # 도로 놓기·들고 대기 모드는 확인할 수 없음
            return res
        ok, why = verify_pick(node, res, p, log)
        if ok:
            return {**res, 'attempts': k}
        log(f'확인 실패 ({k}/{tries}): {why}')
        if k < tries and on_retry:
            on_retry(k, tries - 1, why)
    raise PickFailed(f'{tries}번 시도했지만 실패 - {why}')
