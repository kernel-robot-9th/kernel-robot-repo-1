"""RealSense RGB 실시간 영상 창 (librealsense 없이 V4L2 로 직접 읽음).

  python3 ~/couping_ws/tools/realsense_rgb.py [가로 세로]   예) 1280 720
  (시스템 python3 사용: LeRobot venv 의 OpenCV 는 창 띄우기 기능이 없음)
  q 또는 Esc 로 종료.

RealSense 는 여러 /dev/video* 를 만든다 (깊이 Z16, 적외선, RGB YUYV, 메타데이터).
이름이 RealSense 이고 YUYV 포맷을 내는 장치를 RGB 로 골라 연다.
"""
import glob
import os
import subprocess
import sys

import cv2


def find_rgb_device():
    for path in sorted(glob.glob('/sys/class/video4linux/video*'),
                       key=lambda p: int(p.rsplit('video', 1)[1])):
        with open(os.path.join(path, 'name')) as f:
            if 'RealSense' not in f.read():
                continue
        dev = '/dev/' + os.path.basename(path)
        fmts = subprocess.run(['v4l2-ctl', '-d', dev, '--list-formats'],
                              capture_output=True, text=True).stdout
        if "'YUYV'" in fmts:
            return dev
    return None


def main():
    width, height = (int(sys.argv[1]), int(sys.argv[2])) if len(sys.argv) > 2 else (1280, 720)
    dev = find_rgb_device()
    if dev is None:
        sys.exit('RealSense RGB 장치를 찾지 못했습니다 (USB 연결 확인)')
    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 30)
    if not cap.isOpened():
        sys.exit(f'{dev} 를 열 수 없습니다')
    print(f'RealSense RGB: {dev} {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
          f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} — q/Esc 로 종료', flush=True)
    win = 'RealSense RGB'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    fails = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            fails += 1
            if fails > 30:
                sys.exit('프레임을 받지 못했습니다 (USB 3 포트에 연결했는지 확인)')
            continue
        fails = 0
        cv2.imshow(win, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
