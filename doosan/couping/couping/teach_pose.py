"""직접교시로 맞춘 현재 자세를 이름 붙여 저장 (로봇은 움직이지 않음).

  python3 -m couping.teach_pose front          # 진열대 앞 수평 자세
  python3 -m couping.teach_pose observe        # 관찰 자세 (기본은 준비자세라 보통 불필요)
  python3 -m couping.teach_pose place_above    # 핑키 위 (내려놓기 전 대기)
  python3 -m couping.teach_pose place          # 핑키에 내려놓는 위치
  python3 -m couping.teach_pose --show         # 저장된 자세 보기

수동 모드에서 직접교시 버튼으로 자세를 맞추고, 버튼에서 손을 뗀 뒤 실행한다.
"""
import argparse
import math
import sys

import numpy as np
import rclpy

from couping import poses
from couping.dsr_util import connect
from couping.handeye import zyz_to_R


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name', nargs='?', choices=poses.NAMES)
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args()
    if args.show or not args.name:
        for n in poses.NAMES:
            p = poses.get(n)
            print(f'{n:12s}', '(없음)' if p is None else f"posj {p['posj']}  posx {p['posx']}  {p.get('time', '')} {p.get('note', '')}")
        return

    rclpy.init()
    node = rclpy.create_node('teach_pose', namespace='dsr01')
    dsr = connect(node)
    q = [float(v) for v in dsr.get_current_posj()]
    x, _ = dsr.get_current_posx()
    x = [float(v) for v in x]
    tool_z = zyz_to_R(*x[3:6])[:, 2]
    tilt = math.degrees(math.asin(max(-1.0, min(1.0, tool_z[2]))))
    print(f'현재 관절 {np.round(q, 1)}\n현재 TCP  {np.round(x, 1)}  그리퍼 축 기울기 {tilt:+.1f}도')

    if args.name == 'front':
        # 진열대(+Y 방향)를 향해 수평이어야 함
        if abs(tilt) > 15 or tool_z[1] < 0.8:
            sys.exit('저장 안 함: front 는 그리퍼가 진열대(+Y)를 향해 수평이어야 합니다 '
                     f'(기울기 {tilt:+.0f}도, 진열대 방향 성분 {tool_z[1]:.2f}).')
    saved = poses.save(args.name, q, x, note=f'tilt {tilt:+.1f}')
    print(f"저장: {poses.POSE_FILE} [{args.name}] {saved['time']}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
