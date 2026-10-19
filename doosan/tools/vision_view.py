"""비전 노드 결과 영상 보기 (카메라를 직접 열지 않으므로 비전 노드와 동시에 사용 가능).

  ~/venv/yolo/bin/python ~/couping_ws/tools/vision_view.py     (ROS 환경에서)
/couping/vision/image/compressed 를 받아 창에 띄운다. q/Esc: 종료
"""
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import CompressedImage

WIN = 'RealSense (shelf_vision)'


def main():
    rclpy.init()
    node = rclpy.create_node('vision_view')
    latest = {}
    node.create_subscription(CompressedImage, '/couping/vision/image/compressed',
                             lambda m: latest.__setitem__('img', m), 1)
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 720)
    blank = np.zeros((720, 1280, 3), np.uint8)
    cv2.putText(blank, 'waiting for /couping/vision/image/compressed ...', (40, 360),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            m = latest.pop('img', None)
            if m is not None:
                img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    blank = img
            try:
                cv2.imshow(WIN, blank)
                if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                    break
            except cv2.error:  # 창을 X 로 닫았을 때
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
