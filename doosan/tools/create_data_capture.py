import pyrealsense2 as rs
import cv2
import numpy as np
import os

# ----------------------------
# 저장 폴더 생성
# ----------------------------
# 실행 위치와 상관없이 같은 폴더에 저장 (원본: 상대경로 dataset_images_test_set)
save_dir = os.path.expanduser("~/couping_ws/dataset/images")
os.makedirs(save_dir, exist_ok=True)

# 다시 실행해도 이전 사진을 덮어쓰지 않도록 기존 마지막 번호 다음부터 저장
existing = [int(f[4:8]) for f in os.listdir(save_dir)
            if f.startswith("img_") and f.endswith(".jpg") and f[4:8].isdigit()]
count = max(existing) + 1 if existing else 0

# ----------------------------
# RealSense 설정
# ----------------------------
pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.color,
    1280,
    720,
    rs.format.bgr8,
    30
)

config.enable_stream(
    rs.stream.depth,
    1280,
    720,
    rs.format.z16,
    30
)

try:
    pipeline.start(config)

    print("=================================")
    print(f"저장 폴더: {save_dir} (다음 번호 img_{count:04d})")
    print("s : 이미지 저장")
    print("q : 종료")
    print("=================================")

    while True:

        frames = pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        color_image = np.asanyarray(
            color_frame.get_data()
        )

        # 화면 표시 (저장 장수 표시는 화면에만, 저장 이미지에는 안 들어감)
        view = color_image.copy()
        cv2.putText(view, f"saved: {count}  (s=save, q=quit)", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow(
            "RealSense Color",
            view
        )

        key = cv2.waitKey(1) & 0xFF

        # 저장
        if key == ord('s'):

            filename = os.path.join(
                save_dir,
                f"img_{count:04d}.jpg"
            )

            cv2.imwrite(
                filename,
                color_image
            )

            print(f"Saved : {filename}")

            count += 1

        # 종료
        elif key == ord('q'):
            break

finally:

    pipeline.stop()
    cv2.destroyAllWindows()