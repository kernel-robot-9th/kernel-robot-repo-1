"""RViz 에서 찍은 위치를 핑키 목표 좌표로 저장.

  python3 pinky_waypoint.py            # 찍을 때마다 좌표만 출력
  python3 pinky_waypoint.py --save pick_pose   # 창고(두산 앞 도킹 구역)
  python3 pinky_waypoint.py --save pack_pose   # OMX 포장 구역
  python3 pinky_waypoint.py --save home_pose   # 주차 구역

RViz(nav2_view) 상단의 '2D Goal Pose' 로 위치·방향을 찍으면 /goal_pose 로 들어온다.
(2D Goal Pose 를 찍으면 핑키가 실제로 그 위치로 출발하므로, 갈 수 있는 곳만 찍을 것)
'Publish Point' 로 찍으면 /clicked_point 로 들어오며 방향(yaw)은 0 으로 둔다.
"""
import argparse
import math
import os
import re

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped

CONFIG = os.path.expanduser(
    '~/team/kernel-robot-repo-1/integration/couping_integration/config/integration.yaml')


def save(name, x, y, yaw):
    s = open(CONFIG).read()
    new, n = re.subn(rf'(\n\s+{name}:\s*)\[[^\]]*\]', rf'\g<1>[{x:.3f}, {y:.3f}, {yaw:.3f}]', s, count=1)
    if n != 1:
        raise SystemExit(f'{CONFIG} 에서 {name} 를 찾지 못함')
    open(CONFIG, 'w').write(new)
    print(f'저장: {name} = [{x:.3f}, {y:.3f}, {yaw:.3f}]  ({CONFIG})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--save', choices=('pick_pose', 'pack_pose', 'home_pose'))
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('pinky_waypoint')
    done = {'v': False}

    def handle(x, y, yaw, src):
        print(f'[{src}] x={x:.3f} y={y:.3f} yaw={yaw:.3f} rad ({math.degrees(yaw):.0f}°)', flush=True)
        if args.save:
            save(args.save, x, y, yaw)
            done['v'] = True

    def on_goal(m):
        q = m.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        handle(m.pose.position.x, m.pose.position.y, yaw, '2D Goal Pose')

    node.create_subscription(PoseStamped, '/goal_pose', on_goal, 10)
    node.create_subscription(PointStamped, '/clicked_point',
                             lambda m: handle(m.point.x, m.point.y, 0.0, 'Publish Point'), 10)
    print('RViz 에서 위치를 찍으세요 (Ctrl+C 종료)', flush=True)
    try:
        while rclpy.ok() and not done['v']:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
