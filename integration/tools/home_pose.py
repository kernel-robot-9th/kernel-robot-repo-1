"""핑키를 P(주차) 근처에 두고 실행: 라이다 스캔을 맵 벽에 맞춰 실제 위치를 찾아 AMCL 초기 위치로 넣는다.

  python3 integration/tools/home_pose.py      # 2D Pose Estimate 대신
P 주변(x -0.05~0.35, y -0.15~0.35, ±30°)을 찾아 라이다 점이 맵 벽과 5cm 안에서 가장 많이 겹치는 위치를 고른다.
85% 미만이면 P 근처가 아니라고 보고 아무것도 하지 않는다. (팀원 pack_run.py check_home 과 같은 방식)
"""
import math
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'amr', 'pinky_navigation', 'map', 'arena_map.pgm')
RES, OX, OY = 0.02, -0.196, -0.377      # arena_map.yaml


def main():
    im = cv2.imread(MAP, 0)
    h, w = im.shape
    dist = cv2.distanceTransform(((im >= 50) * 255).astype(np.uint8), cv2.DIST_L2, 3) * RES
    rclpy.init()
    n = rclpy.create_node('home_pose')
    scan, got = [None], []
    n.create_subscription(LaserScan, '/scan', lambda m: scan.__setitem__(0, m), qos_profile_sensor_data)
    n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: got.append(m), 10)
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    end = time.time() + 6
    while scan[0] is None and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if scan[0] is None:
        raise SystemExit('라이다(/scan) 없음')
    s = scan[0]
    r = np.array(s.ranges)
    a = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = np.isfinite(r) & (r > 0.18) & (r < 3.0)
    a, r = a[ok], r[ok]
    bx, by = -0.017 + r * np.cos(a + math.pi), r * np.sin(a + math.pi)   # 핑키 라이다는 180도 돌아 있음

    def fit(x, y, yaw):
        mx = x + bx * math.cos(yaw) - by * math.sin(yaw)
        my = y + bx * math.sin(yaw) + by * math.cos(yaw)
        c = ((mx - OX) / RES).astype(int)
        rw = (h - 1 - (my - OY) / RES).astype(int)
        ins = (c >= 0) & (c < w) & (rw >= 0) & (rw < h)
        d = np.full(len(mx), 1.0)
        d[ins] = dist[rw[ins], c[ins]]
        return np.mean(d < 0.05)

    f, X, Y, T = max((fit(x, y, math.radians(t)), x, y, t)
                     for x in np.arange(-0.05, 0.35, 0.02) for y in np.arange(-0.15, 0.35, 0.02) for t in range(-30, 31, 2))
    print(f'라이다 맞춤 위치: ({X:.2f}, {Y:.2f}) {T:+d}° → {f * 100:.0f}% 일치')
    if f < 0.85:
        raise SystemExit('85% 미만: 핑키가 P 근처에 없는 것 같음. 위치 설정 안 함')
    T = math.radians(T)
    while pub.get_subscription_count() == 0:
        rclpy.spin_once(n, timeout_sec=0.1)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.pose.pose.position.x, m.pose.pose.position.y = float(X), float(Y)
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(T / 2), math.cos(T / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.03 ** 2
    m.pose.covariance[35] = math.radians(3) ** 2
    for _ in range(4):
        got.clear()
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        end = time.time() + 1.5
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.1)
        if got and math.hypot(got[-1].pose.pose.position.x - X, got[-1].pose.pose.position.y - Y) < 0.03:
            break
    if got:
        p = got[-1].pose.pose
        print(f'AMCL 설정: ({p.position.x:.3f}, {p.position.y:.3f}) '
              f'{math.degrees(2 * math.atan2(p.orientation.z, p.orientation.w)):+.1f}°')
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
