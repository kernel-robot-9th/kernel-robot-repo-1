"""라이다에 찍힌 핑키 자기 몸(적재판·적재물) 점을 지워 /scan_filtered 로 내보낸다 (10/28).

  python3 integration/tools/scan_self_filter.py          # 노트북에서, 핑키와 같은 ROS_DOMAIN_ID 로
로컬 코스트맵(주행 제어기 충돌 감지용)만 /scan_filtered 를 쓴다. AMCL·도킹은 원래 /scan.
지우는 영역 = Nav2 footprint(T자: 몸체 ±0.075 m, 적재판 x -0.075 ~ -0.175 m, ±0.14 m) + MARGIN.
"""
import math

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

MARGIN = 0.01
FRONT, BODY_HALF, PLATE_START, REAR, PLATE_LEFT, PLATE_RIGHT = 0.08, 0.065, 0.00, -0.17, 0.145, 0.115   # 10/28 비대칭 적재판 실측 (+y 왼쪽 140mm, -y 오른쪽 110mm, 앞모서리 bx=-0.015 포함)


def main():
    rclpy.init()
    n = rclpy.create_node('scan_self_filter')
    tf = Buffer()
    TransformListener(tf, n, spin_thread=True)
    pub = n.create_publisher(LaserScan, '/scan_filtered', qos_profile_sensor_data)
    mount = {}   # 라이다 -> base_footprint (x, y, yaw), 처음 한 번만 읽음

    def on_scan(m):
        if not mount:
            try:
                t = tf.lookup_transform('base_footprint', m.header.frame_id, rclpy.time.Time()).transform
            except Exception:
                return
            mount.update(x=t.translation.x, y=t.translation.y,
                         yaw=2 * math.atan2(t.rotation.z, t.rotation.w))
            n.get_logger().info(f"라이다 장착: x={mount['x']:.3f} y={mount['y']:.3f} yaw={math.degrees(mount['yaw']):.0f}deg")
        r = np.asarray(m.ranges, dtype=np.float32)
        a = m.angle_min + np.arange(len(r)) * m.angle_increment + mount['yaw']
        bx = mount['x'] + r * np.cos(a)
        by = mount['y'] + r * np.sin(a)
        left_lim = np.where(bx < PLATE_START, PLATE_LEFT, BODY_HALF) + MARGIN
        right_lim = np.where(bx < PLATE_START, PLATE_RIGHT, BODY_HALF) + MARGIN
        self_hit = (bx < FRONT + MARGIN) & (bx > REAR - MARGIN) & (by < left_lim) & (by > -right_lim)
        r[self_hit] = np.inf
        out = m
        out.ranges = r.tolist()
        pub.publish(out)

    n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
