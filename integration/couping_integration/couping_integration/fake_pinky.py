"""가짜 핑키 — 실제 핑키 없이 Nav2 NavigateToPose 액션 서버를 흉내 낸다 (연동 흐름 시험용).

  ros2 run couping_integration fake_pinky
  ros2 launch couping_integration integration.launch.py ... fake:=true fake_pinky:=true

목표 좌표를 받으면 지금 위치에서의 거리 / speed 초 동안 '이동'한 뒤 성공을 돌려준다.
시작 위치는 start_pose (보통 home_pose). 이동 중 취소(/couping/stop)도 받는다.
실패 시험: ros2 param set /fake_pinky fail true → 다음 목표부터 실패(ABORTED).

실제 핑키 Nav2 와 같은 ROS_DOMAIN_ID 에서 함께 띄우지 말 것 (같은 액션 이름이 두 개가 됨).
"""
import math
import signal
import time

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FakePinky(Node):

    def __init__(self):
        super().__init__('fake_pinky')
        self.declare_parameter('nav_action', '/navigate_to_pose')
        self.declare_parameter('speed', 0.2)        # m/s
        self.declare_parameter('min_seconds', 1.0)  # 아주 가까워도 최소 이 시간은 걸림
        self.declare_parameter('start_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('fail', False)
        self.x, self.y = self.get_parameter('start_pose').value[:2]
        ActionServer(self, NavigateToPose, self.get_parameter('nav_action').value, self.execute,
                     cancel_callback=lambda _: CancelResponse.ACCEPT,
                     callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'fake_pinky 준비 완료 (위치 {self.x:.2f}, {self.y:.2f}). 목표 대기 중')

    def execute(self, goal_handle):
        pose = goal_handle.request.pose.pose
        gx, gy = pose.position.x, pose.position.y
        yaw = 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)
        dist = math.hypot(gx - self.x, gy - self.y)
        sec = max(self.get_parameter('min_seconds').value, dist / self.get_parameter('speed').value)
        self.get_logger().info(f'목표 ({gx:.2f}, {gy:.2f}, yaw {yaw:.2f}) 수신 → {dist:.2f} m, {sec:.1f}초 이동')

        deadline = time.monotonic() + sec
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('이동 취소됨')
                return NavigateToPose.Result()
            time.sleep(0.1)

        if self.get_parameter('fail').value:
            goal_handle.abort()
            self.get_logger().warn('fail=true → 도착 실패로 응답')
            return NavigateToPose.Result()
        self.x, self.y = gx, gy
        goal_handle.succeed()
        self.get_logger().info(f'도착 ({gx:.2f}, {gy:.2f})')
        return NavigateToPose.Result()


def main(args=None):
    rclpy.init(args=args)
    node = FakePinky()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
