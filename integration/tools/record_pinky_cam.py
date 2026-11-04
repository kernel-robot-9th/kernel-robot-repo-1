#!/usr/bin/env python3
"""
Pinky 2호기 전면 카메라 실시간 MP4 녹화 스크립트.
토픽: /camera/image_raw/compressed (기본) 또는 /docking/debug_image/compressed (도킹 오버레이 포함)
저장: pinky2_cam_YYYYMMDD_HHMMSS.mp4

사용법:
  python3 record_pinky_cam.py               # 기본 원본 카메라 녹화
  python3 record_pinky_cam.py --docking     # ArUco 마커 인식 오버레이 화면 녹화
"""
import argparse
from datetime import datetime
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class PinkyCamRecorder(Node):
    def __init__(self, topic, output_path, fps=20.0):
        super().__init__('pinky_cam_recorder')
        self.topic = topic
        self.output_path = output_path
        self.fps = fps
        self.writer = None
        self.frame_count = 0
        self.start_time = None

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(CompressedImage, topic, self.cb, qos)
        self.get_logger().info(f"[녹화 준비] 토픽: {topic}")
        self.get_logger().info(f"[저장 경로] {output_path}")
        self.get_logger().info("카메라 프레임 수신 대기 중... (종료 시 Ctrl+C)")

    def cb(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        h, w = frame.shape[:2]
        if self.writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (w, h))
            self.start_time = time.time()
            self.get_logger().info(f"▶ 녹화 시작! 해상도: {w}x{h}, {self.fps}fps")

        self.writer.write(frame)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            elapsed = time.time() - self.start_time
            sys.stdout.write(f"\r[녹화 중] {elapsed:.1f}초 경과 ({self.frame_count} 프레임 저장됨)... ")
            sys.stdout.flush()

    def close(self):
        if self.writer is not None:
            self.writer.release()
            elapsed = time.time() - self.start_time if self.start_time else 0
            print(f"\n✔ 녹화 완료! 저장 파일: {self.output_path} (총 {self.frame_count} 프레임, {elapsed:.1f}초)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--docking', action='store_true', help='도킹 인식 오버레이 화면(/docking/debug_image/compressed) 녹화')
    parser.add_argument('--out', type=str, default=None, help='저장 파일명')
    parser.add_argument('--fps', type=float, default=20.0, help='녹화 FPS')
    args = parser.parse_args()

    topic = '/docking/debug_image/compressed' if args.docking else '/camera/image_raw/compressed'
    now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = "pinky2_docking" if args.docking else "pinky2_cam"
    out_file = args.out or f"/home/omen/couping_ws/{prefix}_{now_str}.mp4"

    rclpy.init()
    recorder = PinkyCamRecorder(topic, out_file, fps=args.fps)
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

