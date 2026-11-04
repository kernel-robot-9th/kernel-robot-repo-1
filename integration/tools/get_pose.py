#!/usr/bin/env python3
import math
import sys
import time
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def main():
    rclpy.init()
    node = Node('get_pose_tool')
    tf = Buffer()
    TransformListener(tf, node, spin_thread=True)
    time.sleep(1.0)
    try:
        t = None
        for _ in range(30):
            try:
                t = tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
                break
            except Exception:
                time.sleep(0.1)
        if t is not None:
            x = t.translation.x
            y = t.translation.y
            yaw = yaw_of(t.rotation)
            deg = math.degrees(yaw)
            print(f"\n[Current Robot Pose]")
            print(f"  Tuple: ({x:.4f}, {y:.4f}, {yaw:.4f})")
            print(f"  Details: x={x:.4f}m, y={y:.4f}m, yaw={yaw:.4f} rad ({deg:+.1f}°)\n")
        else:
            print("Failed to lookup transform after 3s")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()

