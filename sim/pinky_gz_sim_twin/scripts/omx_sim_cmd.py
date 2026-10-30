#!/usr/bin/python3
"""가상 OMX(omx_f_1, omx_f_2)에 관절 목표(rad)를 전달하는 노드.

입력 : /omx_f_1/target_joint_states , /omx_f_2/target_joint_states  (sensor_msgs/JointState)
        name 에 joint1~joint5, gripper_joint_1 중 보낼 것만 넣으면 됨. position 은 rad.
출력 : /omx_f_N/<joint>/cmd_pos (std_msgs/Float64) -> omx_bridge 가 Gazebo 로 전달
        gripper_joint_2 는 gripper_joint_1 의 부호 반대 값(-1배)으로 자동 생성.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

ARMS = ['omx_f_1', 'omx_f_2']
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint_1']


class OmxSimCmd(Node):
    def __init__(self):
        super().__init__('omx_sim_cmd')
        self.pubs = {}
        for arm in ARMS:
            for j in JOINTS + ['gripper_joint_2']:
                self.pubs[(arm, j)] = self.create_publisher(Float64, f'/{arm}/{j}/cmd_pos', 10)
            self.create_subscription(JointState, f'/{arm}/target_joint_states',
                                     lambda msg, a=arm: self.on_target(a, msg), 10)
        self.get_logger().info('listening: ' + ', '.join(f'/{a}/target_joint_states' for a in ARMS))

    def send(self, arm, joint, value):
        self.pubs[(arm, joint)].publish(Float64(data=float(value)))

    def on_target(self, arm, msg):
        for name, pos in zip(msg.name, msg.position):
            if name not in JOINTS or pos != pos:  # 모르는 이름/NaN 은 무시
                continue
            self.send(arm, name, pos)
            if name == 'gripper_joint_1':
                self.send(arm, 'gripper_joint_2', -pos)


def main():
    rclpy.init()
    node = OmxSimCmd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
