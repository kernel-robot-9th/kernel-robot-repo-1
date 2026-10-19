"""RealSense 깊이 확인 도구 — RGB 화면을 클릭하면 그 점의 3D 좌표(mm, 카메라 기준)를 보여준다.

  ~/venv/rs/bin/python ~/couping_ws/tools/realsense_depth_check.py

  왼쪽 클릭: 3D 좌표 측정 (주변 7x7 픽셀 깊이의 중앙값, 0 은 제외)
  c: 측정 점 지우기   s: 현재 RGB 화면 저장(~/couping_ws/captures)   q/Esc: 종료

왼쪽은 RGB, 오른쪽은 깊이(가까울수록 붉음, 검정 = 깊이 없음)다.
깊이는 RGB 에 맞춰 정렬(align)되어 있어 같은 픽셀 위치를 가리킨다.
"""
import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs

W, H, FPS = 1280, 720, 30
WIN = 'RealSense depth check'
SAVE_DIR = os.path.expanduser('~/couping_ws/captures')

points = []   # (u, v)
pending = []  # 클릭 대기열


def on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and x < W:  # 왼쪽(RGB) 영역만
        pending.append((x, y))


def measure(depth_frame, intr, u, v, scale):
    """(u, v) 주변 7x7 깊이의 중앙값으로 3D 좌표(mm)를 구한다. 깊이 없으면 None."""
    depth = np.asanyarray(depth_frame.get_data())
    patch = depth[max(v - 3, 0):v + 4, max(u - 3, 0):u + 4].astype(np.float32)
    valid = patch[patch > 0]
    if valid.size == 0:
        return None, 0.0
    z_m = float(np.median(valid)) * scale
    x, y, z = rs.rs2_deproject_pixel_to_point(intr, [u, v], z_m)
    return (x * 1000, y * 1000, z * 1000), valid.size / patch.size


def main():
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    profile = pipe.start(cfg)
    dev = profile.get_device()
    scale = dev.first_depth_sensor().get_depth_scale()
    print(f'연결: {dev.get_info(rs.camera_info.name)} (S/N {dev.get_info(rs.camera_info.serial_number)}),'
          f' 펌웨어 {dev.get_info(rs.camera_info.firmware_version)}', flush=True)
    align = rs.align(rs.stream.color)
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 0)  # Jet: 가까우면 파랑→멀면 빨강

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, W, H // 2)
    cv2.setMouseCallback(WIN, on_mouse)
    os.makedirs(SAVE_DIR, exist_ok=True)
    last_dump = 0.0
    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            color_f, depth_f = frames.get_color_frame(), frames.get_depth_frame()
            if not color_f or not depth_f:
                continue
            intr = color_f.profile.as_video_stream_profile().intrinsics
            color = np.asanyarray(color_f.get_data()).copy()
            depth_vis = np.asanyarray(colorizer.colorize(depth_f).get_data()).copy()
            depth_raw = np.asanyarray(depth_f.get_data())
            depth_vis[depth_raw == 0] = 0  # 깊이 없는 곳은 검정

            while pending:
                points.append(pending.pop(0))
            for i, (u, v) in enumerate(points[-8:]):
                xyz, ratio = measure(depth_f, intr, u, v, scale)
                if xyz is None:
                    text = f'#{i + 1} NO DEPTH'
                    col = (0, 0, 255)
                else:
                    text = f'#{i + 1} X{xyz[0]:.0f} Y{xyz[1]:.0f} Z{xyz[2]:.0f}mm ({ratio * 100:.0f}%)'
                    col = (0, 255, 0) if ratio > 0.8 else (0, 200, 255)
                for img in (color, depth_vis):
                    cv2.drawMarker(img, (u, v), col, cv2.MARKER_CROSS, 20, 2)
                    cv2.putText(img, text, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                    cv2.putText(img, text, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

            valid_pct = np.count_nonzero(depth_raw) / depth_raw.size * 100
            cv2.putText(color, f'click=3D  c=clear  s=save  q=quit | depth valid {valid_pct:.0f}%',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            view = np.hstack([color, depth_vis])
            cv2.imshow(WIN, view)
            if time.time() - last_dump > 2:  # 원격 확인용: 현재 화면을 2초마다 파일로 저장
                cv2.imwrite(os.path.join(SAVE_DIR, 'latest_view.jpg'), cv2.resize(view, None, fx=0.5, fy=0.5))
                last_dump = time.time()
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27) or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord('c'):
                points.clear()
            if key == ord('s'):
                path = os.path.join(SAVE_DIR, time.strftime('%Y%m%d_%H%M%S') + '.png')
                cv2.imwrite(path, np.asanyarray(color_f.get_data()))
                print('저장:', path)
            if points and len(points) != getattr(main, '_printed', 0):
                main._printed = len(points)
                u, v = points[-1]
                xyz, ratio = measure(depth_f, intr, u, v, scale)
                print(f'#{len(points)} 픽셀({u},{v}) → '
                      + ('깊이 없음' if xyz is None else
                         f'X={xyz[0]:.0f} Y={xyz[1]:.0f} Z={xyz[2]:.0f} mm (유효 {ratio * 100:.0f}%)'), flush=True)
    finally:
        pipe.stop()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
