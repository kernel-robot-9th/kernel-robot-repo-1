#!/usr/bin/python3
"""실제 OMX(LeRobot) -> 가상 OMX 연결.

LeRobot 이 UDP 로 보낸 값(정규화 -100~100, 그리퍼 0~100)을 받아
  /omx_f_1/target_joint_states , /omx_f_2/target_joint_states  (sensor_msgs/JointState, rad)
로 발행한다. 그 뒤는 omx_sim_cmd.py 가 가상 OMX 관절로 전달한다.

변환: 각도(rad) = sign * (값 / 100) * pi + offset      (모터 눈금 2048 = 0, 4096 = 한 바퀴 가정)
      gripper_joint_1 = sign * (값 / 100) * gripper_max_rad
관절 순서: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll -> joint1..joint5
"""
import json
import math
import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"]


def convert(side_values, signs, offsets, grip_sign, grip_max_rad):
    """side_values: {'shoulder_pan': v, ...} -> (names, positions[rad]). 없는 값은 건너뜀."""
    names, pos = [], []
    for motor, joint, sg, off in zip(ARM_MOTORS, ARM_JOINTS, signs, offsets):
        v = side_values.get(motor)
        if v is None or v != v:
            continue
        names.append(joint)
        pos.append(sg * (v / 100.0) * math.pi + off)
    g = side_values.get("gripper")
    if g is not None and g == g:
        names.append("gripper_joint_1")
        pos.append(grip_sign * (g / 100.0) * grip_max_rad)
    return names, pos


class OmxUdpBridge(Node):
    def __init__(self):
        super().__init__("omx_udp_bridge")
        self.declare_parameter("port", 5005)
        self.declare_parameter("bind_host", "127.0.0.1")  # 다른 PC 에서 받으려면 0.0.0.0
        self.declare_parameter("left_arm", "omx_f_1")
        self.declare_parameter("right_arm", "omx_f_2")
        # 관절별 방향(+1/-1)과 0점 보정(rad). 실제 팔을 움직여 보며 맞춘다.
        self.declare_parameter("left_signs", [1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("right_signs", [1.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("left_offsets", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("right_offsets", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("gripper_sign", 1.0)
        self.declare_parameter("gripper_max_rad", 1.0)  # 그리퍼 100(완전 열림)일 때의 gripper_joint_1 값 (임시값)
        self.declare_parameter("print_raw", False)

        p = lambda n: self.get_parameter(n).value
        self.cfg = {
            "left": (p("left_arm"), p("left_signs"), p("left_offsets")),
            "right": (p("right_arm"), p("right_signs"), p("right_offsets")),
        }
        self.grip_sign, self.grip_max = p("gripper_sign"), p("gripper_max_rad")
        self.print_raw = p("print_raw")
        self.pubs = {s: self.create_publisher(JointState, f"/{c[0]}/target_joint_states", 10)
                     for s, c in self.cfg.items()}

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((p("bind_host"), p("port")))
        self.sock.setblocking(False)
        self.count, self.last_report = 0, self.get_clock().now()
        self.create_timer(0.01, self.poll)
        self.get_logger().info(f"UDP {p('bind_host')}:{p('port')} 수신 대기 (left->{p('left_arm')}, right->{p('right_arm')})")

    def poll(self):
        latest = None
        while True:  # 쌓인 패킷은 최신 것만 사용
            try:
                data, _ = self.sock.recvfrom(65535)
                latest = data
            except BlockingIOError:
                break
        if latest is None:
            return
        try:
            msg = json.loads(latest)
        except ValueError:
            return
        self.count += 1
        for side, (arm, signs, offsets) in self.cfg.items():
            values = msg.get(side)
            if not values:
                continue
            names, pos = convert(values, signs, offsets, self.grip_sign, self.grip_max)
            if not names:
                continue
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name, js.position = names, pos
            self.pubs[side].publish(js)
            if self.print_raw:
                self.get_logger().info(
                    f"{side}: raw={ {k: round(v, 1) for k, v in values.items()} } -> rad={[round(x, 2) for x in pos]}")
        now = self.get_clock().now()
        if (now - self.last_report).nanoseconds > 5e9:
            self.get_logger().info(f"최근 5초 수신 {self.count}회")
            self.count, self.last_report = 0, now


def main():
    rclpy.init()
    node = OmxUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
