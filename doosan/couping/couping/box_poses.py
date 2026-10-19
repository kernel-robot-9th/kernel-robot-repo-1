"""파란 박스(핑키 적재 상자) 내려놓기 자세를 하드코딩 값으로 만들어 poses.json 에 저장.

  ros2 run couping box_poses          # 로봇 연결 필요 (역기구학 계산만, 로봇은 안 움직임)
  ros2 run couping box_poses --show   # 계산만 하고 저장 안 함

박스는 위치가 고정이라 직접교시 대신 아래 BOX 값으로 정한다. 박스를 옮기면 BOX 만 고치고 다시 실행.
동작: 진열대에서 수평으로 빼서 front → place_above (박스 위 높은 곳, 그리퍼 아래 방향) 로 관절 이동
      → 수직으로 place 까지 내려가 그리퍼를 열어 떨어뜨림 → 다시 위로 → 관찰 자세.
컵은 그리퍼가 아래를 향하면서 옆으로 누운 채 박스에 들어간다. 위에서 떨어뜨리므로 여러 개도 같은 자리.
"""
import argparse
import math
import sys

import numpy as np
import rclpy

from couping import poses
from couping.dsr_util import connect

BOX = {
    # 10/15 15:47 박스 가운데에 그리퍼를 아래로 향하게 대고 잰 플랜지 자세 (mm, deg ZYZ) 와 그때 관절
    'flange_posx': [495.5, -29.4, 322.7, 132.7, -170.6, 131.8],
    'posj': [-1.0, 17.1, 83.4, -7.1, 73.1, -0.3],
    'release_up': 120.0,   # 잰 높이보다 이만큼 위에서 그리퍼를 열어 떨어뜨림 (박스 벽보다 높게)
    'above_up': 250.0,     # 잰 높이보다 이만큼 위: 여기서 아래 방향으로 돌린 뒤 수직으로 내려감
    'slots': 6,            # 같은 자리를 place, place_2 … place_6 으로 저장 (주문 품목 최대 개수)
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--show', action='store_true', help='계산만 하고 저장 안 함')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('box_poses', namespace='dsr01')
    dsr = connect(node)
    sol = int(dsr.get_solution_space(dsr.posj(*BOX['posj'])))
    out = {}
    for name, up in (('place_above', BOX['above_up']), ('place', BOX['release_up'])):
        x = list(BOX['flange_posx'])
        x[2] += up
        q = dsr.ikin(dsr.posx(*x), sol)
        if isinstance(q, int) or q is None:
            sys.exit(f'{name}: 역기구학 실패 (도달 불가). above_up/release_up 을 줄이세요')
        q = [float(v) for v in q]
        if max(abs(a - b) for a, b in zip(q, BOX['posj'])) > 60:
            sys.exit(f'{name}: 관절이 잰 자세와 너무 다릅니다 {np.round(q, 1)} (다른 해). 저장 안 함')
        out[name] = (q, x)
        print(f'{name:12s} 관절 {np.round(q, 1)}  플랜지 {np.round(x, 1)}  (잰 높이 +{up:.0f}mm, '
              f'도달 {math.hypot(x[0], x[1]):.0f}mm)')
    if not args.show:
        poses.save('place_above', *out['place_above'], note=f"box 하드코딩 +{BOX['above_up']:.0f}")
        for k in range(1, BOX['slots'] + 1):
            poses.save('place' if k == 1 else f'place_{k}', *out['place'],
                       note=f"box 하드코딩 +{BOX['release_up']:.0f} (모든 품목 같은 자리에 떨어뜨림)")
        print(f'저장: {poses.POSE_FILE} (place_above, place, place_2~{BOX["slots"]})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
