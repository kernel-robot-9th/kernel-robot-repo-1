"""진열대 수평 집기 테스트: 비전(base_mm) 위치로 앞에서 수평 접근 → (선택) 집기 → 뒤로 빼기.

강의 클릭 이동 테스트는 위에서 내려가는 방식이라 선반에는 못 쓴다. 이 스크립트는
  ① 사람이 직접교시로 그리퍼를 '진열대를 향해 수평'으로 맞춰 둔 그 방향을 그대로 쓰고
  ② 먼저 뒤로 뺀 뒤 → 컵 정면 approach(기본 150mm) 앞에서 높이·좌우를 맞추고
  ③ 그리퍼 방향(수평)으로만 직선 전진한다.
모든 이동 전에 Enter 확인, 저속(기본 50mm/s). 'q' 입력 시 중단.

준비
  - bringup + gripper_service (ros2 run dsr_gripper gripper_service)
  - 비전 노드: ros2 launch couping vision.launch.py   (캘리브레이션 결과를 읽음)
  - 직접교시로 그리퍼를 진열대 쪽 수평으로, 진열대에서 20cm 이상 떨어진 곳에 둔다
실행 (ROS 환경에서, 시스템 python3)
  python3 -m couping.shelf_pick_test --list                 # 비전 결과만 보기
  python3 -m couping.shelf_pick_test --teach --row 1 --col 3   # 손으로 잡는 위치에 맞춘 뒤 오프셋 저장 (먼저 1번)
  python3 -m couping.shelf_pick_test --row 1 --col 1 --dry-run   # 계획만 출력 (로봇 안 움직임)
  python3 -m couping.shelf_pick_test --row 1 --col 1        # 컵 앞까지 갔다가 다시 빠짐 (안 집음)
  python3 -m couping.shelf_pick_test --cls tide --grip      # 집어서 살짝 들고 빼기
좌표
  - posx 는 플랜지 기준이라 그리퍼 끝(tcp-z) 만큼 보정한다. 불확실하면 tcp-z 를 실제보다 '짧게'
    (= 덜 들어감) 잡는 게 안전 — 수평 접근이라 강의(위→아래)와 반대.
  - --depth 로 컵 중심 대비 얼마나 더/덜 들어갈지(+면 더 깊이) 조절.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from std_srvs.srv import Trigger

from couping.handeye import zyz_to_R

# 플랜지 → 잡는 점(손가락 가운데) 오프셋, 플랜지(툴) 좌표계 mm. --teach 로 저장
OFFSET_FILE = os.path.expanduser('~/couping_ws/calib/grasp_offset.json')
# 팔이 컵을 가리기 전에 찍은 인식 결과를 재사용하기 위해 마지막 결과를 저장 (--cached)
LAST_FILE = os.path.expanduser('~/couping_ws/calib/last_targets.json')


def ask(msg):
    r = input(f'\n{msg}  [Enter=진행, q=중단] ').strip().lower()
    if r == 'q':
        raise KeyboardInterrupt


def fetch_targets(node):
    cli = node.create_client(Trigger, '/couping/vision/detect')
    if not cli.wait_for_service(timeout_sec=5.0):
        sys.exit('비전 노드가 없습니다 → ros2 launch couping vision.launch.py')
    fut = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=20.0)
    res = fut.result()
    if res is None or not res.success:
        sys.exit(f'비전 인식 실패: {None if res is None else res.message}')
    data = json.loads(res.message)
    if not data.get('calibrated'):
        sys.exit('비전 노드에 캘리브레이션이 적용되지 않았습니다 (handeye_file 확인)')
    return data['items']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--row', type=int)
    ap.add_argument('--col', type=int)
    ap.add_argument('--cls', help='종류로 고르기 (여러 개면 첫 번째)')
    ap.add_argument('--list', action='store_true', help='비전 결과만 출력')
    ap.add_argument('--cached', action='store_true',
                    help='새로 찍지 않고 마지막 인식 결과 사용 (팔이 컵을 가리고 있을 때)')
    ap.add_argument('--shelf-ymin', type=float, default=380.0,
                    help='이보다 로봇 쪽(y 작음) 검출은 진열대 밖으로 보고 제외 (mm)')
    ap.add_argument('--target', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                    help='비전 대신 컵 중심 base 좌표(mm)를 직접 지정')
    ap.add_argument('--dry-run', action='store_true', help='계획만 출력, 로봇 안 움직임')
    ap.add_argument('--grip', action='store_true', help='컵 위치에서 그리퍼를 닫고 들어서 빼기')
    ap.add_argument('--tcp-z', type=float, default=200.0, help='플랜지→그리퍼 끝(손가락 가운데) 거리 mm')
    ap.add_argument('--approach', type=float, default=150.0, help='컵 앞 대기 거리 mm')
    ap.add_argument('--depth', type=float, default=0.0, help='컵 중심 대비 추가 전진 mm (+ 더 깊이)')
    ap.add_argument('--dz', type=float, default=0.0, help='목표 높이 보정 mm')
    ap.add_argument('--lift', type=float, default=15.0, help='집은 뒤 들어올릴 높이 mm')
    ap.add_argument('--force', type=int, default=300, help='그리퍼 힘(전류)')
    ap.add_argument('--open', type=int, default=450, help='그리퍼 여는 정도 0~750 (끝까지 열면 선반 다리에 걸림)')
    ap.add_argument('--step', type=float, default=20.0, help='마지막 접근을 이 간격(mm)으로 나눠 한 칸씩 확인')
    ap.add_argument('--teach', action='store_true',
                    help='직접교시로 그리퍼를 컵을 잡을 위치에 둔 상태에서 실행 → 플랜지→잡는 점 오프셋 저장 (로봇 안 움직임)')
    ap.add_argument('--carry-center', action='store_true',
                    help='집은 뒤 진열대 앞 가운데(좌우 가운데, 컵 앞 approach+hold-back)로 수평 유지한 채 이동')
    ap.add_argument('--hold-back', type=float, default=100.0, help='가운데 대기 위치를 대기선보다 더 뒤로 뺄 거리 mm')
    ap.add_argument('--vel', type=float, default=20.0, help='직선 속도 mm/s')
    ap.add_argument('--acc', type=float, default=20.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('shelf_pick_test', namespace='dsr01')
    if args.target:
        items = [{'cls': 'manual', 'row': 0, 'col': 0, 'conf': 1.0, 'base_mm': list(args.target)}]
    elif args.cached:
        if not os.path.exists(LAST_FILE):
            sys.exit('저장된 인식 결과가 없습니다 (팔을 치우고 --list 를 한 번 실행)')
        items = json.load(open(LAST_FILE))
        print(f'마지막 인식 결과 사용: {LAST_FILE} ({time.ctime(os.path.getmtime(LAST_FILE))})')
    else:
        items = fetch_targets(node)
        # 진열대가 아닌 로봇 쪽(y 가 작음) 검출은 그리퍼·팔을 물건으로 잘못 본 것 → 제외
        bad = [it for it in items if it['base_mm'] and it['base_mm'][1] < args.shelf_ymin]
        for it in bad:
            print(f"  제외: r{it['row']}c{it['col']} {it['cls']} y={it['base_mm'][1]:.0f} < {args.shelf_ymin:.0f} (진열대 밖)")
        items = [it for it in items if it not in bad]
        if args.list:  # 팔을 치운 상태에서 --list 할 때만 저장 (팔이 가린 결과로 덮어쓰지 않게)
            os.makedirs(os.path.dirname(LAST_FILE), exist_ok=True)
            with open(LAST_FILE, 'w') as f:
                json.dump(items, f, ensure_ascii=False)
            print(f'인식 결과 저장: {LAST_FILE} (이후 --cached 로 사용)')
    print('\n비전 결과 (base_mm = 컵 중심, 두산 베이스 mm)')
    for it in items:
        print(f"  r{it['row']}c{it['col']}  {it['cls']:12s} conf {it['conf']:.2f}  base_mm {it['base_mm']}")
    if args.list:
        return

    cands = [it for it in items if it['base_mm']
             and (args.cls is None or args.target or it['cls'] == args.cls)
             and (args.row is None or it['row'] == args.row)
             and (args.col is None or it['col'] == args.col)]
    if not cands:
        sys.exit('조건에 맞고 깊이가 유효한 컵이 없습니다 (--list 로 확인)')
    tgt = cands[0]
    target = np.array(tgt['base_mm'], float)
    print(f"\n목표: r{tgt['row']}c{tgt['col']} {tgt['cls']}  컵 중심 {np.round(target, 1)}")

    from couping.dsr_util import connect
    dsr = connect(node)  # 서비스 발견 전 호출하면 멈추는 문제 방지

    start_mode = dsr.get_robot_mode()
    if not args.dry_run and not args.teach:
        ask('자동 모드로 바꿉니다. 로봇에서 손을 떼세요.')
        dsr.set_robot_mode(dsr.ROBOT_MODE_AUTONOMOUS)
        time.sleep(0.5)
    if args.teach:
        # 비전은 팔이 컵을 가리기 전에 이미 찍었음 → 이제 손으로 잡는 위치에 맞춤
        ask(f"비전 인식 완료. 이제 직접교시로 그리퍼를 r{tgt['row']}c{tgt['col']} {tgt['cls']} 를 "
            '실제로 잡을 위치(손가락 사이 가운데에 컵)에 두고, 버튼에서 손을 뗀 뒤')
    pose, _ = dsr.get_current_posx()
    pose = [float(v) for v in pose]
    R = zyz_to_R(*pose[3:6])
    tool_z = R[:, 2]
    tilt = math.degrees(math.asin(max(-1.0, min(1.0, tool_z[2]))))
    print(f'현재 플랜지 {np.round(pose[:3], 1)}  방향(ZYZ) {np.round(pose[3:], 1)}  그리퍼 축 기울기 {tilt:+.1f}도')
    if args.teach:
        # 그리퍼가 지금 '컵을 잡는 위치'에 있다고 보고, 플랜지→컵 중심을 툴 좌표계로 저장
        off = R.T @ (target - np.array(pose[:3]))
        # 잘못 저장 방지: 수평이 아니거나, 컵이 그리퍼 앞 100~350mm / 옆 120mm 안에 없으면 거부
        if abs(tilt) > 15 or not (100 < off[2] < 350) or np.hypot(off[0], off[1]) > 120:
            sys.exit(f'저장 안 함: 그리퍼 기울기 {tilt:+.0f}도, 컵까지(툴 좌표) {np.round(off)}mm.\n'
                     '그리퍼를 수평으로, 컵이 손가락 사이에 오도록 둔 뒤 다시 실행하세요.')
        os.makedirs(os.path.dirname(OFFSET_FILE), exist_ok=True)
        with open(OFFSET_FILE, 'w') as f:
            json.dump({'offset_tool_mm': off.round(1).tolist(), 'cup': tgt['cls'],
                       'flange_posx': [round(v, 2) for v in pose], 'cup_base_mm': tgt['base_mm'],
                       'time': time.strftime('%Y-%m-%d %H:%M')}, f, indent=2, ensure_ascii=False)
        print(f'\n저장: {OFFSET_FILE}\n플랜지→잡는 점 (툴 좌표 mm): {np.round(off, 1)}'
              f'  ← 기존 가정 [0, 0, {args.tcp_z:.0f}]')
        print('이 값은 비전 오차(약 1cm)까지 함께 흡수합니다. 다른 컵 1~2개로 한 번 더 확인하는 걸 권장.')
        return
    if os.path.exists(OFFSET_FILE):
        tool_off = np.array(json.load(open(OFFSET_FILE))['offset_tool_mm'], float)
        print(f'잡는 점 오프셋: {OFFSET_FILE} → {np.round(tool_off, 1)} (툴 좌표 mm)')
    else:
        tool_off = np.array([0.0, 0.0, args.tcp_z])
        print(f'잡는 점 오프셋 파일이 없어 [0, 0, {args.tcp_z:.0f}] 가정 (--teach 로 먼저 맞추는 걸 권장)')
    if abs(tilt) > 15:
        sys.exit('그리퍼가 수평이 아닙니다 (±15도 초과). 직접교시로 진열대를 향해 수평으로 맞춘 뒤 다시 실행하세요.')
    d = np.array([tool_z[0], tool_z[1], 0.0])
    d /= np.linalg.norm(d)  # 수평 접근 방향
    to_cup = target[:2] - np.array(pose[:2])
    if np.dot(d[:2], to_cup) < 0:
        sys.exit('그리퍼가 진열대 반대쪽을 보고 있습니다.')

    grasp_tip = target + d * args.depth + np.array([0, 0, args.dz])
    offset = R @ tool_off                            # 플랜지 → 잡는 점 (base 좌표)

    def flange(tip):
        return tip - offset

    def tip_now():
        p, _ = dsr.get_current_posx()
        return np.array(p[:3], float) + offset

    pre_tip = grasp_tip - d * args.approach
    cur_tip = np.array(pose[:3]) + offset
    # 1) 지금 위치가 대기선보다 진열대 쪽이면 먼저 뒤로 빼기 (높이·좌우 유지)
    back = np.dot(pre_tip - cur_tip, d)
    retreat_tip = cur_tip + d * back if back < 0 else None

    plan = []
    if retreat_tip is not None:
        plan.append(('뒤로 빼기 (방향 그대로)', retreat_tip))
    plan += [(f'컵 정면 {args.approach:.0f}mm 앞으로 (높이·좌우 맞춤)', pre_tip),
             (f'그리퍼 열기 ({args.open})', None)]
    n = max(1, math.ceil(args.approach / args.step))
    for k in range(1, n + 1):  # 한 칸씩 전진 (매번 확인)
        rest = args.approach * (1 - k / n)
        plan.append((f'전진 {k}/{n} (컵까지 {rest:.0f}mm 남음)', grasp_tip - d * rest))
    if args.grip:
        plan += [('그리퍼 닫기', None),
                 (f'{args.lift:.0f}mm 들어올리기', grasp_tip + [0, 0, args.lift]),
                 ('뒤로 빼기', pre_tip + [0, 0, args.lift])]
        if args.carry_center:
            # 진열대 좌우 가운데 = 인식된 컵들의 x 범위 가운데, 높이는 그대로, 대기선보다 hold-back 만큼 뒤
            xs = [it['base_mm'][0] for it in items if it['base_mm']]
            center = pre_tip + [0, 0, args.lift] - d * args.hold_back
            center[0] = (min(xs) + max(xs)) / 2
            plan += [(f'진열대 앞 가운데로 이동 (수평 유지, x={center[0]:.0f})', center)]
    else:
        plan += [('뒤로 빼기 (안 집음)', pre_tip)]

    print(f'\n접근 방향(수평) {np.round(d, 3)}  tcp-z {args.tcp_z}mm  속도 {args.vel}mm/s')
    for i, (name, tip) in enumerate(plan, 1):
        if tip is None:
            print(f'  {i}. {name}')
            continue
        f = flange(tip)
        reach = math.hypot(f[0], f[1])
        warn = []
        if not 200 < reach < 850:
            warn.append(f'도달거리 {reach:.0f}mm 범위 밖')
        if tip[2] < 50:
            warn.append('그리퍼 끝 50mm 미만')
        print(f"  {i}. {name:28s} 그리퍼끝 {np.round(tip, 0)}  플랜지 {np.round(f, 0)}  {' / '.join(warn)}")
        if warn:
            sys.exit('안전 범위를 벗어난 단계가 있어 중단합니다.')
    if args.dry_run:
        print('\n--dry-run: 로봇은 움직이지 않았습니다.')
        return

    from dsr_gripper import gripper_cmd
    try:
        for name, tip in plan:
            ask(f'다음: {name}')
            if tip is None:
                ok = gripper_cmd(args.open, 200) if '열기' in name else gripper_cmd(0, args.force)
                time.sleep(1.5)
                print('  그리퍼', '완료' if ok else '실패')
                continue
            f = flange(tip)
            ret = dsr.movel(dsr.posx(*f.tolist(), *pose[3:6]), vel=args.vel, acc=args.acc)
            if ret != 0:
                raise RuntimeError(f'movel 실패 (ret={ret})')
            err = np.linalg.norm(tip_now() - tip)
            print(f'  도착. 그리퍼 끝 {np.round(tip_now(), 0)} (목표와 {err:.1f}mm)')
        print('\n완료. 그리퍼 끝이 컵 중심에 맞았는지, 좌우·높이가 얼마나 어긋났는지 알려주세요.')
    except KeyboardInterrupt:
        print('\n중단했습니다. 로봇은 현재 위치에 멈춰 있습니다.')
    finally:
        if start_mode == dsr.ROBOT_MODE_MANUAL:
            dsr.set_robot_mode(dsr.ROBOT_MODE_MANUAL)
            print('수동 모드로 되돌렸습니다.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
