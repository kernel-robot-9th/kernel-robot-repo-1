"""학습된 YOLO 모델 스트레스 테스트 (로봇·카메라 없이 기존 사진으로).

  ~/venv/yolo/bin/python ~/couping_ws/tools/stress_test.py [--weights best.pt last.pt] [--conf 0.5]

라벨이 있는 원본 15장에 조명·거리·각도·가림 변형을 일부러 걸어 보고
상황별로 찾은 비율(재현율), 종류 오답, 헛검출 수를 표로 낸다.
- '검증'은 학습에 안 쓴 3장(img_0003/0008/0013), '전체'는 15장.
- 절반 넘게 가려진 컵은 정답에서 빼고(못 찾아도 정상), 그 컵을 찾은 예측도 헛검출로 세지 않는다.
결과 예시 그림: ~/couping_ws/captures/stress/<상황>.jpg
"""
import argparse
import glob
import os
import random

import cv2
import numpy as np
from ultralytics import YOLO

DS = os.path.expanduser('~/couping_ws/dataset/yolo')
OUT = os.path.expanduser('~/couping_ws/captures/stress')
VAL = {'img_0003', 'img_0008', 'img_0013'}


def load():
    items = []
    for split in ('train', 'val'):
        for f in sorted(glob.glob(f'{DS}/images/{split}/img_00??.jpg')):  # 증강본 제외, 원본만
            img = cv2.imread(f)
            h, w = img.shape[:2]
            boxes = []
            for line in open(f.replace('/images/', '/labels/').replace('.jpg', '.txt')):
                c, x, y, bw, bh = map(float, line.split())
                boxes.append([int(c), (x - bw / 2) * w, (y - bh / 2) * h, (x + bw / 2) * w, (y + bh / 2) * h])
            items.append((os.path.basename(f)[:-4], img, boxes))
    return items


# ---------- 변형: (img, boxes, rng) -> (img, boxes, ignore_boxes) ----------
def warp(img, boxes, M, persp=False):
    h, w = img.shape[:2]
    out = (cv2.warpPerspective if persp else cv2.warpAffine)(img, M, (w, h), borderMode=cv2.BORDER_REPLICATE)
    new = []
    for c, x0, y0, x1, y1 in boxes:
        pts = np.float32([[x0, y0], [x1, y0], [x0, y1], [x1, y1]])[None]
        p = (cv2.perspectiveTransform(pts, M) if persp else cv2.transform(pts, M))[0]
        a0, b0 = np.clip(p.min(0), 0, [w, h])
        a1, b1 = np.clip(p.max(0), 0, [w, h])
        full = np.ptp(p[:, 0]) * np.ptp(p[:, 1])
        if (a1 - a0) * (b1 - b0) > 0.5 * full:
            new.append([c, a0, b0, a1, b1])
    return out, new, []


def brightness(k, b=0):
    return lambda img, bx, r: (np.clip(img.astype(np.float32) * k + b, 0, 255).astype(np.uint8), bx, [])


def warm(img, bx, r):
    out = img.astype(np.float32) * np.array([0.7, 0.95, 1.15])  # BGR: 노란 조명
    return np.clip(out, 0, 255).astype(np.uint8), bx, []


def scale(s):
    def f(img, bx, r):
        h, w = img.shape[:2]
        M = cv2.getRotationMatrix2D((w / 2, h / 2), 0, s)
        return warp(img, bx, M)
    return f


def rotate(deg):
    def f(img, bx, r):
        h, w = img.shape[:2]
        return warp(img, bx, cv2.getRotationMatrix2D((w / 2, h / 2), deg, 1.0))
    return f


def side_view(k):
    """카메라가 옆에서 비스듬히 보는 것처럼 (오른쪽이 멀어짐)."""
    def f(img, bx, r):
        h, w = img.shape[:2]
        src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        dst = np.float32([[0, 0], [w, h * k], [w, h * (1 - k)], [0, h]])
        return warp(img, bx, cv2.getPerspectiveTransform(src, dst), persp=True)
    return f


def motion_blur(n):
    def f(img, bx, r):
        k = np.zeros((n, n), np.float32)
        k[n // 2] = 1 / n
        return cv2.filter2D(img, -1, k), bx, []
    return f


def cover(img, rect, color):
    x0, y0, x1, y1 = map(int, rect)
    cv2.rectangle(img, (x0, y0), (x1, y1), color, -1)


def covered_ratio(b, rects):
    m = 0.0
    for x0, y0, x1, y1 in rects:
        iw = max(0, min(b[3], x1) - max(b[1], x0))
        ih = max(0, min(b[4], y1) - max(b[2], y0))
        m += iw * ih
    return m / ((b[3] - b[1]) * (b[4] - b[2]))


def robot_arm(img, bx, r):
    """위에서 내려오는 로봇팔(회색 기둥 + 그리퍼 블록)이 진열대 일부를 가림."""
    img = img.copy()
    h, w = img.shape[:2]
    cx = r.uniform(0.2, 0.6) * w
    bot = r.uniform(0.45, 0.75) * h
    rects = [(cx - 0.035 * w, 0, cx + 0.035 * w, bot), (cx - 0.06 * w, bot - 0.06 * h, cx + 0.06 * w, bot + 0.04 * h)]
    for rc in rects:
        cover(img, rc, (95, 95, 100))
    keep = [b for b in bx if covered_ratio(b, rects) <= 0.5]
    ign = [b for b in bx if covered_ratio(b, rects) > 0.5]
    return img, keep, ign


def partial(frac):
    """컵마다 한쪽 면을 frac 만큼 가림 (손·그리퍼에 일부 가려진 상황)."""
    def f(img, bx, r):
        img = img.copy()
        for c, x0, y0, x1, y1 in bx:
            side = r.choice('lrtb')
            if side == 'l':
                rc = (x0, y0, x0 + (x1 - x0) * frac, y1)
            elif side == 'r':
                rc = (x1 - (x1 - x0) * frac, y0, x1, y1)
            elif side == 't':
                rc = (x0, y0, x1, y0 + (y1 - y0) * frac)
            else:
                rc = (x0, y1 - (y1 - y0) * frac, x1, y1)
            cover(img, rc, (60, 120, 170))  # 살구색 계열
        return img, bx, []
    return f


SCENARIOS = [
    ('기준(원본)', lambda i, b, r: (i, b, [])),
    ('어둡게 x0.5', brightness(0.5)),
    ('아주 어둡게 x0.3', brightness(0.3)),
    ('밝게(과노출)', brightness(1.5, 40)),
    ('노란 조명', warm),
    ('멀리 x0.7', scale(0.7)),
    ('아주 멀리 x0.5', scale(0.5)),
    ('가까이 x1.3', scale(1.3)),
    ('기울기 8도', rotate(8)),
    ('기울기 15도', rotate(15)),
    ('옆에서 보기(약)', side_view(0.08)),
    ('옆에서 보기(강)', side_view(0.16)),
    ('흔들림 블러', motion_blur(15)),
    ('로봇팔 가림', robot_arm),
    ('컵 30% 가림', partial(0.3)),
    ('컵 50% 가림', partial(0.5)),
]


def iou(a, b):
    iw = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    ih = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    i = iw * ih
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def score(pred, gt, ign):
    """pred: [(cls, conf, xyxy)], gt/ign: [[cls, x0, y0, x1, y1]] -> (맞음, 종류오답, 놓침, 헛검출)."""
    used, ok, wrong = set(), 0, 0
    fp = 0
    for c, s, p in sorted(pred, key=lambda t: -t[1]):
        best, j = 0, -1
        for k, g in enumerate(gt):
            if k not in used and iou(p, g[1:]) > best:
                best, j = iou(p, g[1:]), k
        if best >= 0.5:
            used.add(j)
            ok += c == gt[j][0]
            wrong += c != gt[j][0]
        elif not any(iou(p, g[1:]) >= 0.3 for g in ign):
            fp += 1
    return ok, wrong, len(gt) - len(used), fp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', nargs='+', default=[
        os.path.expanduser('~/couping_ws/runs/shelf/yolo11s_1280/weights/best.pt'),
        os.path.expanduser('~/couping_ws/runs/shelf/yolo11s_1280/weights/last.pt')])
    ap.add_argument('--conf', type=float, default=0.5)
    ap.add_argument('--imgsz', type=int, default=1280)
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    items = load()
    models = {os.path.basename(w)[:-3]: YOLO(w) for w in args.weights}

    names = list(models)
    head = f"{'상황':<16}" + ''.join(f"| {n}: 검증 / 전체 (오답,헛검출) " for n in names)
    print(head)
    print('-' * len(head))
    for si, (sname, fn) in enumerate(SCENARIOS):
        row = f'{sname:<16}'
        for mi, (mname, model) in enumerate(models.items()):
            tot = {'val': np.zeros(5), 'all': np.zeros(5)}
            for k, (stem, img, boxes) in enumerate(items):
                r = random.Random(si * 100 + k)  # 모델끼리 같은 변형
                t_img, gt, ign = fn(img, boxes, r)
                res = model.predict(t_img, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
                pred = [(int(c), float(s), b.tolist()) for c, s, b in
                        zip(res.boxes.cls, res.boxes.conf, res.boxes.xyxy)]
                ok, wrong, miss, fp = score(pred, gt, ign)
                v = np.array([ok, wrong, miss, fp, len(gt)])
                tot['all'] += v
                if stem in VAL:
                    tot['val'] += v
                if mi == 0 and stem == 'img_0008':
                    vis = res.plot(line_width=2, font_size=12)
                    cv2.imwrite(os.path.join(OUT, f'{si:02d}_{sname}.jpg'), cv2.resize(vis, None, fx=0.5, fy=0.5))
            v, a = tot['val'], tot['all']
            row += (f'| {100 * v[0] / max(v[4], 1):5.1f}% / {100 * a[0] / max(a[4], 1):5.1f}%'
                    f' ({int(a[1])},{int(a[3])})        ')
        print(row, flush=True)
    print('\n재현율 = 종류까지 맞게 찾은 컵 / 정답 컵.  (오답, 헛검출)은 전체 15장 기준 개수.')
    print('예시 그림:', OUT)


if __name__ == '__main__':
    main()
