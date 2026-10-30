"""Gazebo 디지털 트윈 카메라 → 관리자 화면.

디지털 트윈(pinky_gz_sim digital_twin.launch.xml)은 실제 로봇과 섞이지 않게 별도 ROS_DOMAIN_ID(sim_domain, 기본 10)
에서 돈다. 이 노드는 sim_domain 의 /camera (sensor_msgs/Image, 월드 위 내려다보는 카메라)를 받아 JPEG 로 줄이고,
연동 쪽 도메인(지금 ROS_DOMAIN_ID)의 /couping/sim/image/compressed 로 다시 낸다. web_ui 가 이것을 /sim.mjpg 로 보낸다.

  ros2 run couping_integration sim_camera --ros-args -p sim_domain:=10
  ros2 launch couping_integration integration.launch.py ... sim_view:=true

한 프로세스에서 도메인 두 개(Context 두 개)를 쓰므로 domain_bridge 가 필요 없다.
"""
import os
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

CHANNELS = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}


def to_bgr(msg):
    ch = CHANNELS.get(msg.encoding)
    if ch is None:
        return None
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * ch]
    img = img.reshape(msg.height, msg.width, ch)
    conv = {'rgb8': cv2.COLOR_RGB2BGR, 'rgba8': cv2.COLOR_RGBA2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
            'mono8': cv2.COLOR_GRAY2BGR}.get(msg.encoding)
    return cv2.cvtColor(img, conv) if conv is not None else img


def main(args=None):
    # 파라미터는 연동 쪽 노드에서 읽는다 (launch 가 넘기는 값)
    out_ctx = rclpy.Context()
    rclpy.init(args=args, context=out_ctx)
    out = Node('sim_camera', context=out_ctx)
    out.declare_parameter('sim_domain', 10)
    out.declare_parameter('input_topic', '/camera')
    out.declare_parameter('output_topic', '/couping/sim/image/compressed')
    out.declare_parameter('rate', 5.0)          # Hz, 화면용이라 낮게
    out.declare_parameter('jpeg_quality', 70)
    get = lambda n: out.get_parameter(n).value  # noqa: E731
    sim_domain = int(get('sim_domain'))
    if str(sim_domain) == os.environ.get('ROS_DOMAIN_ID', '0'):
        out.get_logger().warn('sim_domain 이 지금 ROS_DOMAIN_ID 와 같습니다 (보통 디지털 트윈은 10)')
    pub = out.create_publisher(CompressedImage, get('output_topic'), 1)
    period = 1.0 / max(0.1, float(get('rate')))
    quality = int(get('jpeg_quality'))

    in_ctx = rclpy.Context()
    rclpy.init(context=in_ctx, domain_id=sim_domain)
    sub = Node('sim_camera_in', context=in_ctx)
    state = {'last': 0.0, 'count': 0, 'warned': False}

    def on_image(msg):
        now = time.monotonic()
        if now - state['last'] < period:
            return
        state['last'] = now
        img = to_bgr(msg)
        if img is None:
            if not state['warned']:
                out.get_logger().warn(f'지원하지 않는 영상 형식: {msg.encoding}')
                state['warned'] = True
            return
        ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return
        cm = CompressedImage(format='jpeg', data=jpg.tobytes())
        cm.header = msg.header
        pub.publish(cm)
        state['count'] += 1
        if state['count'] == 1:
            out.get_logger().info(f'Gazebo 영상 수신 시작 ({msg.width}x{msg.height}) → {get("output_topic")}')

    sub.create_subscription(Image, get('input_topic'), on_image, qos_profile_sensor_data)
    out.get_logger().info(f'도메인 {sim_domain} 의 {get("input_topic")} 대기 중')

    executors = []
    for node, ctx in ((sub, in_ctx), (out, out_ctx)):
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        executors.append(ex)
    threading.Thread(target=executors[0].spin, daemon=True).start()
    try:
        executors[1].spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        for ex in executors:
            ex.shutdown()
        sub.destroy_node()
        out.destroy_node()
        rclpy.try_shutdown(context=in_ctx)
        rclpy.try_shutdown(context=out_ctx)


if __name__ == '__main__':
    main()
