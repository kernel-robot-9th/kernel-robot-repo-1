"""진열대 비전 노드 (SR_06): YOLO 로 상품을 찾고 RealSense 깊이로 3D 위치를 낸다.

요청이 오면 프레임을 여러 장(기본 5장) 찍어 흐린 프레임은 버리고, 여러 장에서 반복해서 보인
물체만 남긴다(다수결). 스트레스 테스트에서 약했던 흔들림·일시적 가림을 촬영 단계에서 막기 위함이다.
→ 로봇팔이 화면 밖 '관찰 자세'에 멈춘 뒤에 호출해야 한다.

인터페이스
  srv /couping/vision/detect                std_srvs/Trigger         message = 결과 JSON
  sub /couping/vision/cmd                   std_msgs/String          "start" → 인식 1회 (오케스트레이터 run_unit 호환)
  pub /couping/vision/status                std_msgs/String          "done" / "fail:<이유>"
  pub /couping/vision/detections            std_msgs/String          결과 JSON (마지막 결과 유지)
  pub /couping/vision/targets               geometry_msgs/PoseArray  깊이가 유효한 물체 위치 (카메라 좌표, m)
  pub /couping/vision/targets_base          geometry_msgs/PoseArray  같은 물체의 컵 중심, 로봇 base_link 기준 (m) — 캘리브레이션 후
  pub /couping/vision/image/compressed      sensor_msgs/CompressedImage  결과 그림
  tf  base_link → camera_color_optical_frame (정적, 캘리브레이션 결과)
결과 JSON
  {"frames": 5, "calibrated": true, "items": [{"cls": "tomato", "conf": 0.93, "votes": 5, "cls_agree": 1.0,
    "box": [x0, y0, x1, y1], "xyz": [x, y, z] 또는 null, "xyz_center": [...], "base_mm": [x, y, z] 또는 null,
    "depth_ok": true, "row": 1, "col": 2}, ...]}
  xyz 는 컵 앞면, 카메라 광학 좌표계(x 오른쪽, y 아래, z 앞), 단위 m.
  xyz_center 는 앞면에서 시선 방향으로 컵 반지름(박스 폭으로 추정)만큼 들어간 컵 중심.
  base_mm 은 컵 중심을 두산 베이스 좌표(mm)로 바꾼 값 = movel 에 바로 쓸 수 있는 단위.
  변환은 ~/couping_ws/calib/handeye.json (couping.handeye_calib 로 생성). 파일이 바뀌면 다음 인식 때 자동으로 다시 읽는다.
  row/col 은 화면 기준 위→아래, 왼→오른쪽 순서 (1부터). targets 의 poses 는 depth_ok 인 item 순서와 같다.

YOLO 가 설치된 venv(~/venv/yolo, Python 3.12)로 실행해야 해서 ros2 run 대신 launch 로 띄운다.
  ros2 launch couping vision.launch.py
  ros2 launch couping vision.launch.py image_dir:=~/couping_ws/dataset/yolo/images/val   # 카메라 없이 사진으로
"""
import glob
import json
import time
import math
import os
import threading

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from couping import handeye

COLORS = {'tomato': (0, 0, 255), 'carrot': (0, 140, 255), 'salad': (0, 200, 0),
          'bath_tissue': (255, 120, 0), 'tide': (0, 230, 230), 'paper_towel': (200, 0, 200)}


class UsbTooSlow(RuntimeError):
    """USB 2 로 잡혀 1280x720 컬러+깊이를 못 받는 상태."""


class RealSense:
    """컬러에 정렬된 깊이를 함께 주는 RealSense. grab() -> (bgr, depth_m) 또는 None."""

    def __init__(self, width, height, fps, usb2_ok=False):
        import pyrealsense2 as rs
        self.rs = rs
        self.usb2 = False
        devs = rs.context().query_devices()
        if len(devs) and not devs[0].get_info(rs.camera_info.usb_type_descriptor).startswith('3'):
            usb = devs[0].get_info(rs.camera_info.usb_type_descriptor)
            if not usb2_ok:
                raise UsbTooSlow(f"USB {usb} 로 연결됨")
            # USB 2 에서는 1280x720 컬러+깊이가 안 됨 → 640x480@30 (10/19 확인: 약 18fps, 깊이 유효 72%)
            width, height, fps, self.usb2 = 640, 480, 15, True  # 30fps 는 USB 2 직결 포트에서 프레임이 자주 끊김 (10/21)
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = self.pipe.start(cfg)
        self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        self.intr = None
        self.timeout = 8000 if self.usb2 else 2000  # USB 2 는 프레임이 띄엄띄엄 와서 길게 기다림 (10/21)
        self.name = profile.get_device().get_info(rs.camera_info.name) + (
            ' [USB 2: 640x480@15 저해상도 모드]' if self.usb2 else '')

    def flush(self, n=2):
        for _ in range(n):  # 큐에 쌓인 지난 프레임 버리기
            try:
                self.pipe.wait_for_frames(self.timeout)
            except RuntimeError:
                if not self.usb2:
                    raise

    def grab(self):
        try:
            frames = self.align.process(self.pipe.wait_for_frames(self.timeout))
        except RuntimeError:
            if self.usb2:  # 안 온 프레임은 건너뛰고 들어온 것만 씀
                return None
            raise
        cf, df = frames.get_color_frame(), frames.get_depth_frame()
        if not cf or not df:
            return None
        self.intr = cf.profile.as_video_stream_profile().intrinsics
        depth = np.asanyarray(df.get_data()).astype(np.float32) * self.scale
        return np.asanyarray(cf.get_data()).copy(), depth

    def deproject(self, u, v, z):
        return self.rs.rs2_deproject_pixel_to_point(self.intr, [float(u), float(v)], float(z))

    def close(self):
        self.pipe.stop()


class ImageFolder:
    """카메라 없이 시험할 때: 사진을 한 장씩 돌아가며 '정지한 카메라'처럼 같은 사진을 반복해서 준다."""

    def __init__(self, path):
        path = os.path.expanduser(path)
        pattern = os.path.join(path, '*.jpg') if os.path.isdir(path) else path
        self.files = sorted(glob.glob(pattern))
        if not self.files:
            raise RuntimeError(f'사진 없음: {pattern}')
        self.i = 0
        self.name = f'사진 폴더 ({len(self.files)}장)'

    def flush(self, n=2):
        pass

    def grab(self):
        return cv2.imread(self.files[self.i % len(self.files)]), None

    def next(self):
        self.i += 1

    def close(self):
        pass


def iou(a, b):
    iw = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    ih = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    i = iw * ih
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def sharpness(img):
    return cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()


def assign_slots(items):
    """박스 중심 높이가 크게 벌어지는 곳에서 줄을 나누고, 줄 안에서는 왼쪽부터 번호."""
    if not items:
        return
    cy = lambda it: (it['box'][1] + it['box'][3]) / 2
    cx = lambda it: (it['box'][0] + it['box'][2]) / 2
    gap = 0.6 * float(np.median([it['box'][3] - it['box'][1] for it in items]))
    rows, prev = [[]], None
    for it in sorted(items, key=cy):
        if prev is not None and cy(it) - cy(prev) > gap:
            rows.append([])
        rows[-1].append(it)
        prev = it
    for r, row in enumerate(rows, 1):
        for c, it in enumerate(sorted(row, key=cx), 1):
            it['row'], it['col'] = r, c


class ShelfVision(Node):

    def __init__(self):
        super().__init__('shelf_vision')
        self.declare_parameter('weights', '~/couping_ws/runs/shelf/yolo11s_1280/weights/best.pt')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('imgsz', 1280)
        self.declare_parameter('image_dir', '')          # 비어 있으면 RealSense 사용
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)
        self.declare_parameter('num_frames', 5)          # 한 번 인식할 때 찍는 장수
        self.declare_parameter('vote_ratio', 0.6)        # 이 비율 이상 프레임에서 보여야 인정
        self.declare_parameter('sharp_ratio', 0.6)       # 가장 선명한 프레임 대비 이 비율 미만이면 버림
        self.declare_parameter('min_depth_valid', 0.15)  # 박스 안 깊이 유효 비율 기준
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('preview_rate', 0.0)      # >0 이면 이 주기(Hz)로 미리보기 그림만 발행
        self.declare_parameter('save_path', '~/couping_ws/captures/vision_latest.jpg')
        self.declare_parameter('handeye_file', '~/couping_ws/calib/handeye.json')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('grasp_center', True)     # 앞면 대신 컵 중심(반지름만큼 안쪽)을 목표로
        self.declare_parameter('usb2_fallback', True)    # USB 2 로 잡히면 640x480 으로라도 동작 (False 면 리셋만 반복)
        # 10/19 측정: 같은 컵 12개를 640x480 으로 보면 1280x720 보다 x +52mm (y·z 는 거의 같음)
        self.declare_parameter('usb2_offset_mm', [-52.0, 0.0, 0.0])
        self.declare_parameter('max_depth', 1.8)         # 진열대 컵이 이보다 멀게 나오면 깊이 이상 → 카메라 다시 열기 (m)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/couping/vision/status', 10)
        self.det_pub = self.create_publisher(String, '/couping/vision/detections', latched)
        self.pose_pub = self.create_publisher(PoseArray, '/couping/vision/targets', latched)
        self.base_pub = self.create_publisher(PoseArray, '/couping/vision/targets_base', latched)
        self.img_pub = self.create_publisher(CompressedImage, '/couping/vision/image/compressed', 1)
        self.tf_pub = StaticTransformBroadcaster(self)
        self.T_base_cam, self._he_mtime = None, None
        self.load_handeye()
        self.create_subscription(String, '/couping/vision/cmd', self.on_cmd, 10)
        self.create_service(Trigger, '/couping/vision/detect', self.on_detect)

        from ultralytics import YOLO  # 무거워서 노드 생성 시 한 번만
        weights = os.path.expanduser(self.get_parameter('weights').value)
        self.model = YOLO(weights)
        self.model.predict(np.zeros((720, 1280, 3), np.uint8), imgsz=self.p('imgsz'), verbose=False)  # 워밍업
        self.get_logger().info(f'모델: {weights}  클래스: {list(self.model.names.values())}')

        self.lock = threading.Lock()
        self.cam, self._warned = None, False
        self.open_camera()
        self.create_timer(3.0, self.open_camera)  # 카메라가 빠졌다가 다시 꽂혀도 이어서 동작
        self.preview_timer = None
        self.set_preview_rate(self.p('preview_rate'))
        # 실행 중에도 실시간 화면 켜고 끄기: ros2 param set /shelf_vision preview_rate 10.0  (0 이면 끔)
        self.add_on_set_parameters_callback(self.on_params)

    def set_preview_rate(self, rate):
        if self.preview_timer is not None:
            self.destroy_timer(self.preview_timer)
            self.preview_timer = None
        if rate > 0:
            self.preview_timer = self.create_timer(1.0 / rate, self.preview)

    def on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == 'preview_rate':
                self.set_preview_rate(float(prm.value))
                self.get_logger().info(f'실시간 화면 {float(prm.value):.1f}Hz' if prm.value > 0 else '실시간 화면 끔')
        return SetParametersResult(successful=True)

    def p(self, name):
        return self.get_parameter(name).value

    # ---------- 카메라→로봇 좌표 ----------
    def load_handeye(self):
        """캘리브레이션 파일을 (바뀌었으면) 다시 읽고 정적 TF 를 발행."""
        path = os.path.expanduser(self.p('handeye_file'))
        mtime = os.path.getmtime(path) if os.path.exists(path) else None
        if mtime == self._he_mtime:
            return
        self._he_mtime = mtime
        T, info = handeye.load(path)
        self.T_base_cam = T
        if T is None:
            self.get_logger().warn(f'캘리브레이션 파일 없음({path}) → 로봇 좌표(base_mm) 없이 카메라 좌표만 냅니다')
            return
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id, tf.child_frame_id = self.p('base_frame'), self.p('camera_frame')
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = T[:3, 3].tolist()
        q = handeye.R_to_quat(T[:3, :3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.tf_pub.sendTransform(tf)
        self.get_logger().info(f"캘리브레이션 적용: {path} (표본 {info.get('samples')}개, "
                               f"빼고예측 RMS {info.get('loo_rms_mm')}mm, {info.get('note', '')})")

    def to_base_mm(self, xyz):
        if self.T_base_cam is None or xyz is None:
            return None
        p = self.T_base_cam[:3, :3] @ np.asarray(xyz) + self.T_base_cam[:3, 3]
        mm = p * 1000.0
        if getattr(self.cam, 'usb2', False):  # 보정값은 1280x720 기준이라 USB 2(640x480) 모드에서 생기는 어긋남을 되돌림
            mm = mm + np.asarray(self.p('usb2_offset_mm'), float)
        return [round(float(v), 1) for v in mm]

    # ---------- 카메라 ----------
    def open_camera(self):
        if self.cam is not None:
            return
        with self.lock:
            try:
                d = self.p('image_dir')
                self.cam = ImageFolder(d) if d else RealSense(self.p('width'), self.p('height'), self.p('fps'),
                                                                     usb2_ok=self.p('usb2_fallback'))
                self.get_logger().info(f'입력 연결: {self.cam.name}. /couping/vision/detect 대기 중')
            except UsbTooSlow as e:  # 리셋하면 대부분 USB 3 로 다시 잡힘 (30초에 한 번만)
                if time.monotonic() - getattr(self, '_last_reset', -1e9) > 30.0:
                    self._last_reset = time.monotonic()
                    self.get_logger().warn(f'{e} → 카메라를 리셋합니다 (계속되면 다시 꽂아 주세요)')
                    try:
                        import pyrealsense2 as rs
                        for dev in rs.context().query_devices():
                            dev.hardware_reset()
                    except Exception as e2:  # noqa: BLE001
                        self.get_logger().warn(f'리셋 실패: {e2}')
            except Exception as e:  # noqa: BLE001 (pyrealsense2 는 RuntimeError 외에도 던짐)
                if not getattr(self, '_warned', False):
                    self.get_logger().warn(f'카메라 없음({e}). 3초마다 다시 시도합니다')
                    self._warned = True

    def drop_camera(self, err):
        self.get_logger().error(f'카메라 끊김: {err}')
        try:
            self.cam.close()
        except Exception:  # noqa: BLE001
            pass
        self.cam, self._warned = None, False

    # ---------- 인식 ----------
    def frame_dets(self, img, depth):
        res = self.model.predict(img, imgsz=self.p('imgsz'), conf=self.p('conf'), verbose=False)[0]
        dets = []
        for b in res.boxes:
            box = [float(v) for v in b.xyxy[0].tolist()]
            xyz, ratio = self.box_xyz(depth, box)
            # 컵 반지름 ≈ 박스 폭의 절반을 실제 거리로 환산 (뚜껑 포함이라 약간 큼)
            radius = 0.5 * (box[2] - box[0]) * xyz[2] / self.cam.intr.fx if xyz else None
            dets.append({'cls': self.model.names[int(b.cls)], 'conf': float(b.conf), 'box': box,
                         'xyz': xyz, 'ratio': ratio, 'radius': radius})
        return dets

    def box_xyz(self, depth, box):
        """박스 안쪽 80% 영역에서 앞쪽(가까운) 20% 분위 깊이 → 3D 좌표(m). 깊이 없으면 (None, 0).
        투명 컵은 가운데가 뚫려 뒤쪽 벽 깊이가 섞이므로, 가운데 중앙값 대신 컵 앞면(가까운 쪽)을 쓴다.
        (position_check.py 측정: 가운데 중앙값은 유효 1~46% 인 컵이 있었고, 이 방식은 12개 모두 ±1mm 이내)"""
        if depth is None:
            return None, 0.0
        x0, y0, x1, y1 = box
        mx, my = (x1 - x0) * 0.1, (y1 - y0) * 0.1
        patch = depth[int(y0 + my):int(y1 - my) + 1, int(x0 + mx):int(x1 - mx) + 1]
        valid = patch[patch > 0]
        ratio = valid.size / max(patch.size, 1)
        if valid.size == 0:
            return None, 0.0
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        return [float(v) for v in self.cam.deproject(cx, cy, np.percentile(valid, 20))], ratio

    def vote(self, per_frame):
        """프레임마다의 검출을 IoU 로 묶고, 충분히 반복된 것만 남긴다."""
        tracks = []
        for dets in per_frame:
            used = set()
            for d in sorted(dets, key=lambda d: -d['conf']):
                best, bi = 0.0, -1
                for i, t in enumerate(tracks):
                    if i not in used and iou(d['box'], t[-1]['box']) > best:
                        best, bi = iou(d['box'], t[-1]['box']), i
                if best >= 0.5:
                    tracks[bi].append(d)
                    used.add(bi)
                else:
                    tracks.append([d])
                    used.add(len(tracks) - 1)
        need = max(1, math.ceil(self.p('vote_ratio') * len(per_frame)))
        items = []
        for t in tracks:
            if len(t) < need:
                continue
            score = {}
            for d in t:
                score[d['cls']] = score.get(d['cls'], 0.0) + d['conf']
            cls = max(score, key=score.get)
            same = [d for d in t if d['cls'] == cls]
            xyzs = [d['xyz'] for d in t if d['xyz'] is not None and d['ratio'] >= self.p('min_depth_valid')]
            depth_ok = len(xyzs) >= math.ceil(len(t) / 2)
            xyz = np.median(xyzs, axis=0) if xyzs else None
            center = None
            if xyz is not None:
                r = float(np.median([d['radius'] for d in t if d['radius'] is not None])) if self.p('grasp_center') else 0.0
                center = xyz + xyz / np.linalg.norm(xyz) * r  # 시선 방향으로 반지름만큼 안쪽
            items.append({
                'cls': cls,
                'conf': round(float(np.mean([d['conf'] for d in same])), 3),
                'votes': len(t),
                'cls_agree': round(len(same) / len(t), 2),
                'box': [round(float(v), 1) for v in np.median([d['box'] for d in t], axis=0)],
                'xyz': [round(float(v), 4) for v in xyz] if xyz is not None else None,
                'xyz_center': [round(float(v), 4) for v in center] if center is not None else None,
                'base_mm': self.to_base_mm(center) if depth_ok else None,
                'depth_ok': depth_ok,
            })
        assign_slots(items)
        return sorted(items, key=lambda it: (it['row'], it['col']))

    def detect(self):
        """여러 장 찍어 인식 → (성공 여부, 결과 dict 또는 실패 이유).
        깊이가 말이 안 되게 멀면(카메라가 다시 붙은 뒤 생기던 문제) 카메라를 다시 열고 한 번 더 인식."""
        ok, out = self.detect_once()
        if not ok and isinstance(out, str) and out.startswith('카메라'):
            # 카메라가 순간적으로 끊김(USB 허브 등) → 다시 열고 한 번 더
            self.get_logger().warn(f'{out} → 카메라를 다시 열고 한 번 더 인식')
            time.sleep(1.0)
            self.open_camera()
            ok, out = self.detect_once()
        if ok and self.depth_broken(out['items']):
            self.drop_camera(f"깊이 이상 (컵 거리 중앙값 > {self.p('max_depth')} m) → 카메라 다시 열기")
            self.open_camera()
            ok, out = self.detect_once()
            if ok and self.depth_broken(out['items']):
                return False, '카메라 깊이 이상: 카메라를 다시 꽂거나 비전 노드를 재시작하세요'
        return ok, out

    def depth_broken(self, items):
        z = sorted(it['xyz'][2] for it in items if it.get('xyz'))
        return len(z) >= 3 and z[len(z) // 2] > self.p('max_depth')

    def detect_once(self):
        with self.lock:
            if self.cam is None:
                return False, '카메라 없음'
            n = self.p('num_frames')
            try:
                self.cam.flush()
                frames = [f for f in (self.cam.grab() for _ in range(n)) if f is not None]
            except Exception as e:  # noqa: BLE001
                self.drop_camera(e)
                return False, f'카메라 오류: {e}'
            if isinstance(self.cam, ImageFolder):
                self.cam.next()
        if not frames:
            return False, '프레임 없음'
        self.load_handeye()
        sharp = [sharpness(img) for img, _ in frames]
        keep = [f for f, s in zip(frames, sharp) if s >= self.p('sharp_ratio') * max(sharp)]
        per_frame = [self.frame_dets(img, depth) for img, depth in keep]
        items = self.vote(per_frame)
        result = {'frames': len(keep), 'frames_dropped_blur': len(frames) - len(keep),
                  'calibrated': self.T_base_cam is not None, 'items': items}
        self.publish_result(result, keep[-1][0])
        if not items:
            return False, '인식된 물체 없음'
        return True, result

    # ---------- 발행 ----------
    def publish_result(self, result, img):
        stamp = self.get_clock().now().to_msg()
        self.det_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
        pa = PoseArray()
        pa.header.stamp, pa.header.frame_id = stamp, self.p('camera_frame')
        for it in result['items']:
            if it['depth_ok']:
                p = Pose()
                p.position.x, p.position.y, p.position.z = it['xyz']
                p.orientation.w = 1.0
                pa.poses.append(p)
        self.pose_pub.publish(pa)
        pb = PoseArray()
        pb.header.stamp, pb.header.frame_id = stamp, self.p('base_frame')
        for it in result['items']:
            if it['base_mm']:
                p = Pose()
                p.position.x, p.position.y, p.position.z = (v / 1000.0 for v in it['base_mm'])
                p.orientation.w = 1.0
                pb.poses.append(p)
        self.base_pub.publish(pb)
        self.publish_image(self.draw(img, result['items']), stamp)

        items = result['items']
        counts = {}
        for it in items:
            counts[it['cls']] = counts.get(it['cls'], 0) + 1
        summary = ', '.join(f'{k} {v}' for k, v in sorted(counts.items())) or '없음'
        self.get_logger().info(
            f"인식 {len(items)}개 ({summary}) | 깊이 유효 {sum(it['depth_ok'] for it in items)}/{len(items)}"
            f" | 프레임 {result['frames']}장 사용, 흐림으로 {result['frames_dropped_blur']}장 버림")

    def draw(self, img, items):
        img = img.copy()
        for it in items:
            x0, y0, x1, y1 = map(int, it['box'])
            col = COLORS.get(it['cls'], (255, 255, 255))
            z = f" Z{it['xyz'][2] * 1000:.0f}" if it['xyz'] else ''
            label = f"{it['cls']} {it['conf']:.2f}{z}{'' if it['depth_ok'] else ' ?'} r{it['row']}c{it['col']}"
            if it.get('base_mm'):
                bx, by, bz = it['base_mm']
                cv2.putText(img, f'B {bx:.0f},{by:.0f},{bz:.0f}', (x0, y1 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 4)
                cv2.putText(img, f'B {bx:.0f},{by:.0f},{bz:.0f}', (x0, y1 + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
            cv2.putText(img, label, (x0, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 4)
            cv2.putText(img, label, (x0, y0 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
        return img

    def publish_image(self, img, stamp, save=True):
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp, msg.header.frame_id = stamp, self.p('camera_frame')
        msg.format, msg.data = 'jpeg', buf.tobytes()
        self.img_pub.publish(msg)
        path = os.path.expanduser(self.p('save_path'))
        if path and save:  # 실시간 화면은 파일로 저장하지 않음
            os.makedirs(os.path.dirname(path), exist_ok=True)
            cv2.imwrite(path, img)

    def preview(self):
        """미리보기: 한 장만 인식해서 그림만 발행 (detections/targets 는 건드리지 않음)."""
        if self.cam is None or isinstance(self.cam, ImageFolder) or not self.lock.acquire(blocking=False):
            return
        try:
            f = self.cam.grab()
        except Exception as e:  # noqa: BLE001
            self.drop_camera(e)
            return
        finally:
            self.lock.release()
        if f is not None:
            dets = self.frame_dets(*f)
            for d in dets:
                d.update(depth_ok=d['xyz'] is not None and d['ratio'] >= self.p('min_depth_valid'))
            assign_slots(dets)
            self.publish_image(self.draw(f[0], dets), self.get_clock().now().to_msg(), save=False)

    # ---------- 요청 ----------
    def on_detect(self, req, res):
        ok, out = self.detect()
        res.success = ok
        res.message = json.dumps(out, ensure_ascii=False) if ok else out
        return res

    def on_cmd(self, msg):
        if msg.data not in ('start', 'detect'):
            return
        ok, out = self.detect()
        text = 'done' if ok else f'fail:{out}'
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f'status: {text}')


def main():
    rclpy.init()
    node = ShelfVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cam is not None:
            node.cam.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
