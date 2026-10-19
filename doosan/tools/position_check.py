"""RealSense + YOLO 위치 추정 점검.

  ~/venv/yolo/bin/python ~/couping_ws/tools/position_check.py [--frames 60]

카메라를 고정한 채 N 프레임 동안 각 컵의 3D 위치를 재서
- 프레임 사이 흔들림(표준편차) → 반복 정밀도
- 깊이 유효 비율 → 투명 컵 문제
- 깊이 추정 방법 3가지 비교: 가운데 40% 중앙값(현재) / 박스 전체의 앞쪽 20% 분위 / 뚜껑 띠(위 25%)
- 같은 층 컵들의 높이(Y)·거리(Z)가 비슷한지, 옆 칸 간격 → 기하 일관성
을 표로 낸다. 깊이 컬러맵 그림: ~/couping_ws/captures/position_depth.jpg
"""
import argparse
import os

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

W, H = 1280, 720


def iou(a, b):
    iw = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    ih = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    i = iw * ih
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def region(depth, box, fx0, fy0, fx1, fy1):
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    p = depth[int(y0 + h * fy0):int(y0 + h * fy1), int(x0 + w * fx0):int(x0 + w * fx1)]
    return p[p > 0], p.size


def estimates(depth, box):
    c, n = region(depth, box, 0.3, 0.3, 0.7, 0.7)            # A: 가운데 40%
    full, nf = region(depth, box, 0.1, 0.1, 0.9, 0.9)         # B: 박스 전체(가장자리 10% 제외)
    lid, nl = region(depth, box, 0.15, 0.0, 0.85, 0.25)       # C: 뚜껑 띠
    return {
        'A_center': (np.median(c) if c.size else np.nan, c.size / max(n, 1)),
        'B_front20': (np.percentile(full, 20) if full.size else np.nan, full.size / max(nf, 1)),
        'C_lid': (np.median(lid) if lid.size else np.nan, lid.size / max(nl, 1)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default=os.path.expanduser('~/couping_ws/runs/shelf/yolo11s_1280_v2/weights/best.pt'))
    ap.add_argument('--frames', type=int, default=60)
    args = ap.parse_args()

    model = YOLO(args.weights)
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, 30)
    prof = pipe.start(cfg)
    scale = prof.get_device().first_depth_sensor().get_depth_scale()
    align = rs.align(rs.stream.color)
    for _ in range(30):  # 자동 노출 안정화
        pipe.wait_for_frames()

    tracks = []  # {'cls', 'box', 'est': {method: [z...]}, 'ratio': {method: [...]}, 'xy': [(u,v)]}
    intr, last_img, last_depth = None, None, None
    for _ in range(args.frames):
        fr = align.process(pipe.wait_for_frames())
        cf, df = fr.get_color_frame(), fr.get_depth_frame()
        if not cf or not df:
            continue
        intr = cf.profile.as_video_stream_profile().intrinsics
        img = np.asanyarray(cf.get_data()).copy()
        depth = np.asanyarray(df.get_data()).astype(np.float32) * scale
        last_img, last_depth = img, depth
        res = model.predict(img, imgsz=1280, conf=0.5, verbose=False)[0]
        for b in res.boxes:
            box = b.xyxy[0].tolist()
            cls = model.names[int(b.cls)]
            t = max(tracks, key=lambda t: iou(t['box'], box), default=None)
            if t is None or iou(t['box'], box) < 0.5:
                t = {'cls': cls, 'box': box, 'z': {}, 'r': {}, 'boxes': []}
                tracks.append(t)
            t['boxes'].append(box)
            for k, (z, r) in estimates(depth, box).items():
                t['z'].setdefault(k, []).append(z)
                t['r'].setdefault(k, []).append(r)
    pipe.stop()

    tracks = [t for t in tracks if len(t['boxes']) >= args.frames * 0.5]
    for t in tracks:
        bx = np.median(t['boxes'], axis=0)
        t['box'] = bx
        t['u'], t['v'] = (bx[0] + bx[2]) / 2, (bx[1] + bx[3]) / 2
    # 줄·칸 번호
    hmed = np.median([t['box'][3] - t['box'][1] for t in tracks])
    tracks.sort(key=lambda t: t['v'])
    row, prev = 1, None
    for t in tracks:
        if prev is not None and t['v'] - prev['v'] > 0.6 * hmed:
            row += 1
        t['row'], prev = row, t
    for r in set(t['row'] for t in tracks):
        for c, t in enumerate(sorted([t for t in tracks if t['row'] == r], key=lambda t: t['u']), 1):
            t['col'] = c
    tracks.sort(key=lambda t: (t['row'], t['col']))

    methods = ['A_center', 'B_front20', 'C_lid']
    print(f"\n프레임 {args.frames}장, 검출된 컵 {len(tracks)}개\n")
    print(f"{'칸':6s}{'종류':13s}" + ''.join(f'| {m}: Z평균 ±표준편차(mm) 유효% ' for m in methods) + '| X, Y (mm, 방법A)')
    for t in tracks:
        line = f"r{t['row']}c{t['col']}  {t['cls']:12s}"
        for m in methods:
            z = np.array(t['z'][m], float)
            r = np.mean(t['r'][m]) * 100
            ok = z[~np.isnan(z)]
            line += f"| {np.mean(ok) * 1000:6.0f} ±{np.std(ok) * 1000:4.1f}  {r:4.0f}%            " if ok.size else '|   (없음)                      '
        zA = np.nanmedian(t['z']['A_center'])
        X, Y, Z = rs.rs2_deproject_pixel_to_point(intr, [float(t['u']), float(t['v'])], float(zA))
        t['xyz'] = (X * 1000, Y * 1000, Z * 1000)
        line += f'| {X * 1000:6.0f}, {Y * 1000:6.0f}'
        print(line)

    print('\n층별 일관성 (같은 층이면 Y 가 비슷해야 함, 옆 칸 간격은 실제 간격과 비교)')
    for r in sorted(set(t['row'] for t in tracks)):
        row = [t for t in tracks if t['row'] == r]
        ys = [t['xyz'][1] for t in row]
        zs = [t['xyz'][2] for t in row]
        gaps = [np.linalg.norm(np.subtract(row[i + 1]['xyz'], row[i]['xyz'])) for i in range(len(row) - 1)]
        print(f"  {r}층: Y {min(ys):.0f}~{max(ys):.0f} (폭 {max(ys) - min(ys):.0f}mm)  Z {min(zs):.0f}~{max(zs):.0f}"
              f"  옆 칸 간격(mm): {', '.join(f'{g:.0f}' for g in gaps)}")

    vis = cv2.applyColorMap(cv2.convertScaleAbs(np.clip(last_depth, 0, 1.0) * 255), cv2.COLORMAP_JET)
    vis[last_depth == 0] = 0
    for t in tracks:
        x0, y0, x1, y1 = map(int, t['box'])
        cv2.rectangle(vis, (x0, y0), (x1, y1), (255, 255, 255), 2)
        cv2.putText(vis, f"r{t['row']}c{t['col']} Z{t['xyz'][2]:.0f}", (x0, y0 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    both = np.hstack([cv2.resize(last_img, (640, 360)), cv2.resize(vis, (640, 360))])
    out = os.path.expanduser('~/couping_ws/captures/position_depth.jpg')
    cv2.imwrite(out, both)
    print('\n깊이 그림(검정 = 깊이 없음):', out)


if __name__ == '__main__':
    main()
