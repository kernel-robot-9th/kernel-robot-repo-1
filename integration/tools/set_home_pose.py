"""핑키를 P 에 정해진 방향으로 두고 시작할 때: 라이다 맞춤 없이 고정 좌표를 AMCL 초기 위치로 바로 넣는다 (10/27).

  python3 integration/tools/set_home_pose.py            # 약 1초
P 좌표는 10/27 라이다 맞춤(home_pose.py) 결과 평균. 핑키를 다른 곳에 두었으면 home_pose.py 를 쓸 것.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

X, Y, YAW = 0.23, 0.03, 0.0   # 10/28: 새 P (모서리에서 10cm 앞)


def main():
    rclpy.init()
    n = rclpy.create_node('set_home_pose')
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    end = time.time() + 5
    while pub.get_subscription_count() == 0 and time.time() < end:   # AMCL 이 보일 때까지
        rclpy.spin_once(n, timeout_sec=0.05)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.pose.pose.position.x, m.pose.pose.position.y = X, Y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(YAW / 2), math.cos(YAW / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.03 ** 2
    m.pose.covariance[35] = math.radians(3) ** 2
    for _ in range(2):
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        time.sleep(0.1)
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
