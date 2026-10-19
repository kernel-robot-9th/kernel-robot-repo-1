"""주문 품목 자동 피킹 (Enter 없이 한 번에).

  python3 -m couping.auto_pick tomato --dry-run    # 인식 + 계획만 (로봇 안 움직임, 지금 화면 그대로 인식)
  python3 -m couping.auto_pick tomato --step       # 단계마다 Enter (처음 시험용)
  python3 -m couping.auto_pick tomato              # 자동 (관찰 자세 → 인식 → 집기 → [핑키에 놓기])
  python3 -m couping.auto_pick bath_tissue --row 2 # 같은 품목이 여러 개면 층·칸 지정 가능
  python3 -m couping.auto_pick tide --no-place return  # 핑키 자세가 없을 때 진열대에 도로 놓기 (기본: 들고 대기)
품목: tomato, carrot, salad, bath_tissue, tide, paper_towel
처음 한 번: python3 -m couping.teach_pose front  (진열대 앞 수평 자세 저장)
"""
import argparse
import sys

import rclpy

from couping import pick_core
from couping.dsr_util import connect


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('item')
    ap.add_argument('--row', type=int)
    ap.add_argument('--col', type=int)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--step', action='store_true', help='단계마다 Enter 확인')
    ap.add_argument('--slot', type=int, default=1, help='핑키 몇 번 자리에 놓을지 (place_<slot>)')
    ap.add_argument('--no-place', choices=('hold', 'return'), default='hold',
                    help='핑키 자세가 없을 때: hold=front 에서 들고 대기, return=진열대에 도로 놓기')
    for k in ('open', 'force', 'release'):
        ap.add_argument(f'--{k}', type=int, default=pick_core.DEFAULTS[k])
    ap.add_argument('--retries', type=int, default=pick_core.DEFAULTS['retries'], help='카메라 확인 실패 시 다시 시도 횟수')
    for k in ('vel', 'vel_slow', 'velj', 'approach', 'depth', 'dz', 'lift'):
        ap.add_argument(f"--{k.replace('_', '-')}", dest=k, type=float, default=pick_core.DEFAULTS[k])
    args = ap.parse_args()
    p = {k: getattr(args, k) for k in ('open', 'force', 'release', 'vel', 'vel_slow', 'velj',
                                        'approach', 'depth', 'dz', 'lift', 'retries')}

    def confirm(desc):
        if input(f'\n다음: {desc}  [Enter=진행, q=중단] ').strip().lower() == 'q':
            raise KeyboardInterrupt

    rclpy.init()
    node = rclpy.create_node('auto_pick', namespace='dsr01')
    dsr = connect(node)
    start_mode = dsr.get_robot_mode()
    code = 0
    try:
        res = pick_core.pick_verified(node, dsr, args.item, p, args.row, args.col,
                                 confirm=confirm if args.step else None, dry_run=args.dry_run,
                                 slot=args.slot, no_place=args.no_place)
        done = {'placed': '핑키에 내려놓음', 'returned': '진열대에 도로 놓음', 'holding': 'front 에서 들고 대기'}
        msg = done[res['result']]
        print(f'\n--dry-run: 로봇은 움직이지 않았습니다 (계획: {msg})' if args.dry_run else f'\n완료: {msg}')
    except pick_core.PickError as e:
        print(f'\n중단: {e}')
        code = 1
    except KeyboardInterrupt:
        print('\n사용자 중단. 로봇은 현재 위치에 멈춰 있습니다.')
        code = 1
    finally:
        if start_mode == dsr.ROBOT_MODE_MANUAL and not args.dry_run:
            dsr.set_robot_mode(dsr.ROBOT_MODE_MANUAL)
            print('수동 모드로 되돌렸습니다.')
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
