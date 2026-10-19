"""RealSense 실시간 상품 인식 + 3D 위치 표시.

  ~/venv/yolo/bin/python ~/couping_ws/tools/realsense_yolo_live.py [--weights best.pt] [--conf 0.5]
  s: 현재 화면 저장(~/couping_ws/captures)   q/Esc: 종료

각 박스마다 클래스·신뢰도와 함께 카메라 기준 3D 좌표(mm)를 표시한다.
3D 좌표: 박스 안쪽 80% 영역의 앞쪽 20% 분위 깊이 (투명 컵 대책, position_check.py 로 비교해 정함).
깊이 유효 비율이 15% 미만이면 신뢰할 수 없다고 보고 표시만 한다(?로 표시).
"""
import argparse
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

W, H, FPS = 1280, 720, 30
COLORS = {'tomato': (0, 0, 255), 'carrot': (0, 140, 255), 'salad': (0, 200, 0),
          'bath_tissue': (255, 120, 0), 'tide': (0, 230, 230), 'paper_towel': (200, 0, 200)}
SAVE_DIR = os.path.expanduser('~/couping_ws/captures')


def box_xyz(depth, intr, scale, x0, y0, x1, y1):
    """박스 안쪽 80% 영역의 앞쪽 20% 분위 깊이로 3D 좌표(mm)와 유효 비율 (shelf_vision 과 같은 방식)."""
    mx, my = (x1 - x0) * 0.1, (y1 - y0) * 0.1
    patch = depth[int(y0 + my):int(y1 - my) + 1, int(x0 + mx):int(x1 - mx) + 1]
    valid = patch[patch > 0]
    ratio = valid.size / max(patch.size, 1)
    if valid.size == 0:
        return None, 0.0
    z = float(np.percentile(valid, 20)) * scale
    x, y, zz = rs.rs2_deproject_pixel_to_point(intr, [(x0 + x1) / 2, (y0 + y1) / 2], z)
    return (x * 1000, y * 1000, zz * 1000), ratio


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default=os.path.expanduser('~/couping_ws/runs/shelf/yolo11s_1280_v2/weights/best.pt'))
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--min-valid', type=float, default=0.15, help='깊이 유효 비율 기준')
    args = ap.parse_args()

    model = YOLO(args.weights)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    profile = pipe.start(cfg)
    scale = profile.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    os.makedirs(SAVE_DIR, exist_ok=True)
    win = 'Shelf YOLO live'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, W, H)
    last_dump, t_prev = 0.0, time.time()
    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            cf, df = frames.get_color_frame(), frames.get_depth_frame()
            if not cf or not df:
                continue
            img = np.asanyarray(cf.get_data()).copy()
            depth = np.asanyarray(df.get_data())
            intr = cf.profile.as_video_stream_profile().intrinsics

            res = model.predict(img, imgsz=1280, conf=args.conf, verbose=False)[0]
            counts = {}
            for b in res.boxes:
                x0, y0, x1, y1 = b.xyxy[0].tolist()
                name = model.names[int(b.cls)]
                counts[name] = counts.get(name, 0) + 1
                col = COLORS.get(name, (255, 255, 255))
                xyz, ratio = box_xyz(depth, intr, scale, x0, y0, x1, y1)
                pos = '' if xyz is None else f' Z{xyz[2]:.0f}'
                ok = xyz is not None and ratio >= args.min_valid
                label = f'{name} {float(b.conf):.2f}{pos}{"" if ok else " ?"}'
                cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), col, 2)
                cv2.putText(img, label, (int(x0), int(y0) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 4)
                cv2.putText(img, label, (int(x0), int(y0) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)

            now = time.time()
            fps, t_prev = 1 / max(now - t_prev, 1e-6), now
            summary = '  '.join(f'{k}:{v}' for k, v in sorted(counts.items())) or 'no detection'
            cv2.putText(img, f'{fps:4.1f} FPS | {summary} | s=save q=quit', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.imshow(win, img)
            if now - last_dump > 2:  # 원격 확인용 화면 저장
                cv2.imwrite(os.path.join(SAVE_DIR, 'latest_yolo.jpg'), cv2.resize(img, None, fx=0.6, fy=0.6))
                last_dump = now
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord('s'):
                p = os.path.join(SAVE_DIR, 'yolo_' + time.strftime('%Y%m%d_%H%M%S') + '.jpg')
                cv2.imwrite(p, img)
                print('저장:', p, flush=True)
    finally:
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
