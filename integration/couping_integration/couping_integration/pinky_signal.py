#!/usr/bin/env python3
"""핑키 신호 확인 노드 — 핑키(RPi)에서 실행한다. 주행하지 않음.

/couping/pinky/cmd 에 "start" 를 받으면 핑키 터미널에 수신 로그를 찍고
/couping/pinky/status 에 "done" 을 보낸다. 빌드 없이 파일 하나로 실행된다.

핑키에서:  export ROS_DOMAIN_ID=30 && python3 pinky_signal.py
"""
import signal
import socket

import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('pinky_signal')
    pub = node.create_publisher(String, '/couping/pinky/status', 10)
    host = socket.gethostname()

    def on_cmd(msg):
        if msg.data != 'start':
            return
        node.get_logger().info(f'신호 수신 (호스트 {host}) → done 전송')
        pub.publish(String(data='done'))

    node.create_subscription(String, '/couping/pinky/cmd', on_cmd, 10)
    node.get_logger().info(f'pinky_signal 준비 완료 (호스트 {host}). /couping/pinky/cmd 대기 중')
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
