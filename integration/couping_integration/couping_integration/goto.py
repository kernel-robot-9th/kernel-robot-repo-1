"""핑키를 integration.yaml 에 적힌 이름의 위치로 한 번 보낸다 (현장 좌표 확인용).

  ros2 run couping_integration goto            # 갈 수 있는 위치 목록
  ros2 run couping_integration goto daily      # 생활용품 진열대 (pick_zones 의 구역 이름)
  ros2 run couping_integration goto pack       # 포장대 (pack_pose)
  ros2 run couping_integration goto home       # 대기 위치 (home_pose)
  ros2 run couping_integration goto 0.5 0.3 0  # 맵 좌표 직접 [x y yaw]

좌표는 설치된 config/integration.yaml 의 orchestrator 설정에서 읽는다 (--config <파일> 로 바꿀 수 있음).
도착하면 성공/실패와 걸린 시간을 출력한다. Ctrl+C 를 누르면 주행을 취소한다.
"""
import argparse
import math
import os
import sys
import time

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

STATUS = {GoalStatus.STATUS_SUCCEEDED: '도착', GoalStatus.STATUS_ABORTED: '실패(Nav2 가 포기)',
          GoalStatus.STATUS_CANCELED: '취소됨'}


def load_stops(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    p = next(v['ros__parameters'] for k, v in cfg.items() if k.endswith('orchestrator'))
    stops = {'home': p['home_pose'], 'pack': p['pack_pose']}
    for zone in p.get('pick_zones', []):
        stops[zone] = p[zone]['pose']
    return stops, p.get('nav_action', '/navigate_to_pose'), p.get('map_frame', 'map')


def main():
    default_cfg = os.path.join(get_package_share_directory('couping_integration'), 'config', 'integration.yaml')
    ap = argparse.ArgumentParser(description='핑키를 이름 또는 좌표로 보낸다')
    ap.add_argument('target', nargs='*', help='위치 이름(home, pack, 구역 이름) 또는 x y yaw')
    ap.add_argument('--config', default=default_cfg)
    ap.add_argument('--timeout', type=float, default=120.0)
    args, ros_args = ap.parse_known_args()
    stops, action, frame = load_stops(args.config)

    if not args.target:
        print('갈 수 있는 위치 (map 좌표 [x, y, yaw]):')
        for name, pose in stops.items():
            print(f'  {name:6s} {pose}')
        return
    if len(args.target) == 1 and args.target[0] in stops:
        name, (x, y, yaw) = args.target[0], stops[args.target[0]]
    elif len(args.target) == 3:
        name, (x, y, yaw) = '좌표', map(float, args.target)
    else:
        sys.exit(f"'{' '.join(args.target)}' 를 모르겠어요. 위치 이름({', '.join(stops)}) 또는 x y yaw 를 주세요")

    rclpy.init(args=ros_args)
    node = rclpy.create_node('couping_goto')
    client = ActionClient(node, NavigateToPose, action)
    print(f'{name} ({x:.2f}, {y:.2f}, yaw {yaw:.2f}) 로 이동합니다. Nav2 연결 중...')
    if not client.wait_for_server(timeout_sec=15.0):
        sys.exit(f'Nav2 액션 서버({action})가 없어요. 핑키 Nav2 와 ROS_DOMAIN_ID 를 확인하세요')

    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = frame
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    last = {'t': 0.0}

    def on_feedback(msg):
        if time.monotonic() - last['t'] > 1.0:
            last['t'] = time.monotonic()
            print(f'  남은 거리 {msg.feedback.distance_remaining:.2f} m')

    send = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send, timeout_sec=10.0)
    handle = send.result()
    if handle is None or not handle.accepted:
        sys.exit('Nav2 가 목표를 받지 않았어요 (위치 추정이 안 됐거나 목표가 장애물 안일 수 있어요)')
    t0 = time.monotonic()
    result = handle.get_result_async()
    try:
        while not result.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - t0 > args.timeout:
                print(f'{args.timeout:.0f}초 안에 도착하지 못해 취소합니다')
                rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=5.0)
                break
    except KeyboardInterrupt:
        print('\nCtrl+C: 주행을 취소합니다')
        rclpy.spin_until_future_complete(node, handle.cancel_goal_async(), timeout_sec=5.0)
    if result.done():
        status = result.result().status
        print(f'{name}: {STATUS.get(status, f"status={status}")} ({time.monotonic() - t0:.1f}초)')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
