#!/usr/bin/python3
"""
mirror_dsr_arm.py : 실제 Doosan e0509 의 관절각을 Pinky 위 축소 팔(Gazebo)에 그대로 따라 하게 하는 중계 노드.

    실제 e0509 --(/dsr01/joint_states)--> 이 노드 --(/arm_position_controller/commands)--> 가상 팔

* 방향은 실제 -> 가상 한쪽뿐이다. 실제 로봇 쪽 토픽에는 아무것도 publish 하지 않는다.
* 관절 '이름'으로 값을 골라 오므로, 순서가 달라도 / 다른 관절(예: Pinky 바퀴)이 섞여 있어도 된다.

실행 (새 터미널):
    source ~/pinky_pro/install/setup.bash
    /usr/bin/python3 ~/pinky_pro/src/pinky_pro/pinky_gz_sim/scripts/mirror_dsr_arm.py

옵션 예:
    ... mirror_dsr_arm.py --ros-args -p degrees:=true        # 입력이 '도' 단위일 때
    ... mirror_dsr_arm.py --ros-args -p max_rate_hz:=10.0    # 전달 속도 제한
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


def extract_positions(names, positions, wanted):
    """JointState 에서 wanted 이름 순서대로 위치를 뽑는다. 하나라도 없거나 NaN/inf 이면 None."""
    table = dict(zip(names, positions))
    out = []
    for w in wanted:
        v = table.get(w)
        if v is None or math.isnan(v) or math.isinf(v):
            return None
        out.append(float(v))
    return out


class DsrArmMirror(Node):
    def __init__(self):
        super().__init__('dsr_arm_mirror')
        self.declare_parameter('input_topic', '/dsr01/joint_states')
        self.declare_parameter('output_topic', '/arm_position_controller/commands')
        self.declare_parameter('joint_names',
                               ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
        self.declare_parameter('degrees', False)
        self.declare_parameter('max_rate_hz', 30.0)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.wanted = list(self.get_parameter('joint_names').value)
        self.degrees = bool(self.get_parameter('degrees').value)
        rate = float(self.get_parameter('max_rate_hz').value)
        self.min_period = 1.0 / rate if rate > 0.0 else 0.0

        self.received = 0
        self.published = 0
        self.skipped_bad = 0
        self.last_pub_time = -1e9
        self.warned_missing = False

        # best-effort 로 구독하면 실제 쪽 QoS 가 reliable 이든 best-effort 든 연결된다
        self.sub = self.create_subscription(JointState, in_topic, self.on_state, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float64MultiArray, out_topic, 10)
        self.create_timer(5.0, self.report)

        self.get_logger().info(
            f'{in_topic} -> {out_topic} | joints={self.wanted} | '
            f'단위={"도(deg)" if self.degrees else "라디안(rad)"} | 최대 {rate:g} Hz')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_state(self, msg):
        self.received += 1
        pos = extract_positions(msg.name, msg.position, self.wanted)
        if pos is None:
            self.skipped_bad += 1
            if not self.warned_missing:
                self.warned_missing = True
                self.get_logger().warn(
                    f'필요한 관절 {self.wanted} 이(가) 메시지에 없거나 값이 NaN 입니다. 받은 이름: {list(msg.name)}')
            return

        t = self.now_sec()
        if t - self.last_pub_time < self.min_period:
            return
        self.last_pub_time = t

        if self.degrees:
            pos = [math.radians(p) for p in pos]

        out = Float64MultiArray()
        out.data = pos
        self.pub.publish(out)
        self.published += 1

    def report(self):
        if self.received == 0:
            self.get_logger().warn('입력 토픽에서 아직 메시지를 받지 못했습니다. 실제 로봇 bringup / 토픽 이름을 확인하세요.')
        else:
            self.get_logger().info(
                f'받음 {self.received}, 가상 팔로 전달 {self.published}, 건너뜀 {self.skipped_bad}')


def main():
    rclpy.init()
    node = DsrArmMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
