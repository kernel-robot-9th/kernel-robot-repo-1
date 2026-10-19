"""가짜 주문 흐름으로 Gazebo 디지털 트윈의 두산 · OMX 를 움직인다 (fake:=true 시험용).

실제 로봇이 디지털 트윈에 보내던 신호를 흉내 낸다. 디지털 트윈 쪽 코드는 그대로 쓴다.
  두산: /couping/picker/status 의 'progress:... 집는 중' 마다 집는 동작의 관절값을
        /dsr01/joint_states (rad) 로 발행 → domain_bridge → mirror_dsr_arm → 가상 팔
  OMX : /couping/packer/status 가 running 인 동안 박스 닫는 흉내 동작을 LeRobot 과 같은 UDP(JSON)로
        omx_udp_host:omx_udp_port 에 보냄 → omx_udp_bridge → 가상 OMX
핑키는 fake_pinky 가 /amcl_pose · /odom 을 직접 낸다.

  ros2 launch couping_integration integration.launch.py ... fake:=true fake_pinky:=true fake_twin:=true
  ros2 launch pinky_gz_sim digital_twin.launch.xml real_domain:=<연동 ROS_DOMAIN_ID>

실제 두산 · OMX 가 있을 때는 띄우지 말 것 (같은 /dsr01/joint_states 를 두 곳에서 내게 됨).
"""
import json
import math
import signal
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

DSR_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
# 두산 집는 동작 키프레임 (deg, 시간 비율 0~1): 대기 → 진열대 앞 → 집기 → 들기 → 상자 쪽으로 돌기 → 놓기 → 대기
DSR_HOME = [0, 0, 90, 0, 90, 0]
DSR_PICK = [(0.0, DSR_HOME), (0.2, [0, 20, 70, 0, 90, 0]), (0.35, [0, 35, 60, 0, 85, 0]),
            (0.5, [0, 20, 70, 0, 90, 0]), (0.7, [90, 20, 70, 0, 90, 0]), (0.85, [90, 35, 60, 0, 85, 0]),
            (1.0, DSR_HOME)]
OMX_MOTORS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']


def keyframe(frames, u):
    """frames=[(t, [값...]), ...] 를 u(0~1) 에서 선형 보간."""
    u = min(max(u, 0.0), 1.0)
    for (t0, a), (t1, b) in zip(frames, frames[1:]):
        if u <= t1:
            k = 0.0 if t1 == t0 else (u - t0) / (t1 - t0)
            return [x + (y - x) * k for x, y in zip(a, b)]
    return list(frames[-1][1])


def omx_pose(t):
    """박스 닫는 흉내: 두 팔이 앞으로 숙였다 들었다 (LeRobot 정규화 값 -100~100, 그리퍼 0~100)."""
    s = 0.5 - 0.5 * math.cos(2 * math.pi * t / 4.0)  # 4초 주기 0→1→0
    arm = {'shoulder_pan': 0.0, 'shoulder_lift': 25.0 * s, 'elbow_flex': -20.0 * s,
           'wrist_flex': 15.0 * s, 'wrist_roll': 0.0, 'gripper': 30.0 * (1 - s)}
    right = dict(arm, shoulder_pan=-arm['shoulder_pan'])
    return {'left': arm, 'right': right}


class FakeTwin(Node):

    def __init__(self):
        super().__init__('fake_twin')
        self.declare_parameter('pick_seconds', 2.0)     # 두산 fake 가 품목 하나에 쓰는 시간과 같게
        self.declare_parameter('joint_states_topic', '/dsr01/joint_states')
        self.declare_parameter('omx_udp_host', '127.0.0.1')
        self.declare_parameter('omx_udp_port', 5005)
        get = lambda n: self.get_parameter(n).value  # noqa: E731
        self.js_pub = self.create_publisher(JointState, get('joint_states_topic'), 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_addr = (get('omx_udp_host'), int(get('omx_udp_port')))
        self.pick_start = None   # 두산 동작 시작 시각
        self.pack_start = None   # OMX 동작 시작 시각
        self.create_subscription(String, '/couping/picker/status', self.on_picker, 10)
        self.create_subscription(String, '/couping/packer/status', self.on_packer, 10)
        self.create_timer(0.05, self.tick)  # 20Hz
        self.get_logger().info(f'fake_twin 준비 완료: 두산 → {get("joint_states_topic")}, '
                               f'OMX → UDP {self.udp_addr[0]}:{self.udp_addr[1]}')

    def on_picker(self, msg):
        if msg.data.startswith('progress:') and msg.data.endswith('집는 중'):
            self.pick_start = time.monotonic()

    def on_packer(self, msg):
        if msg.data == 'running':
            self.pack_start = time.monotonic()
        elif msg.data == 'done' or msg.data.startswith('fail'):
            self.pack_start = None
            self.send_omx({'left': dict.fromkeys(OMX_MOTORS, 0.0), 'right': dict.fromkeys(OMX_MOTORS, 0.0)})

    def tick(self):
        now = time.monotonic()
        deg = DSR_HOME
        if self.pick_start is not None:
            u = (now - self.pick_start) / self.get_parameter('pick_seconds').value
            deg = keyframe(DSR_PICK, u)
            if u >= 1.0:
                self.pick_start = None
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name, js.position = DSR_JOINTS, [math.radians(d) for d in deg]
        self.js_pub.publish(js)
        if self.pack_start is not None:
            self.send_omx(omx_pose(now - self.pack_start))

    def send_omx(self, msg):
        try:
            self.sock.sendto(json.dumps(msg).encode(), self.udp_addr)
        except OSError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FakeTwin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
