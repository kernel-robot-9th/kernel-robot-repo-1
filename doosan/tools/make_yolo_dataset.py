"""Roboflow COCO 내보내기 → YOLO 데이터셋 변환 + 학습용 증강.

  python3 ~/couping_ws/tools/make_yolo_dataset.py [--per-image 20] [--val img_0003,img_0008,img_0013]

입력: ~/couping_ws/dataset/roboflow_coco/train/_annotations.coco.json (+ 이미지)
출력: ~/couping_ws/dataset/yolo/{images,labels}/{train,val}, data.yaml

- 검증(val) 이미지는 증강하지 않은 원본만 둔다 (성능을 정직하게 보기 위해).
- 학습 이미지마다 원본 1장 + 증강 N장을 만든다.
  1) 컵 바꿔치기: 일부 컵을 다른 사진에서 잘라 온 '다른 클래스의 실제 컵'으로 같은 자리에 교체
  2) 사진 변형: 밝기·대비·약한 색조·흐림·노이즈·약한 확대/이동/회전 (좌우 뒤집기 없음: 글자 반전 방지)
  3) --extra (v2, 스트레스 테스트 약점 보강): 흔들림 블러, 조명 색 변화, 컵 일부 가림, 로봇팔 가림
     python3 make_yolo_dataset.py --extra --out ~/couping_ws/dataset/yolo_v2
"""
import argparse
import json
import os
import random
import shutil

import cv2
import numpy as np

ROOT = os.path.expanduser('~/couping_ws/dataset')
SRC = os.path.join(ROOT, 'roboflow_coco/train')
OUT = os.path.join(ROOT, 'yolo')
CLASSES = ['tomato', 'carrot', 'salad', 'bath_tissue', 'tide', 'paper_towel']


def set_out(path):
    global OUT
    OUT = path


def load_coco():
    d = json.load(open(os.path.join(SRC, '_annotations.coco.json')))
    cats = {c['id']: c['name'] for c in d['categories']}
    items = []
    for im in d['images']:
        boxes = [(CLASSES.index(cats[a['category_id']]), *a['bbox'])
                 for a in d['annotations'] if a['image_id'] == im['id']]
        stem = im['file_name'].split('_jpg')[0]  # img_0003_jpg.rf.xxx.jpg → img_0003
        items.append({'stem': stem, 'path': os.path.join(SRC, im['file_name']), 'boxes': boxes})
    return sorted(items, key=lambda x: x['stem'])


def write_yolo(img, boxes, split, name):
    h, w = img.shape[:2]
    cv2.imwrite(os.path.join(OUT, 'images', split, name + '.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    with open(os.path.join(OUT, 'labels', split, name + '.txt'), 'w') as f:
        for c, x, y, bw, bh in boxes:
            f.write(f'{c} {(x + bw / 2) / w:.6f} {(y + bh / 2) / h:.6f} {bw / w:.6f} {bh / h:.6f}\n')


def build_crop_bank(items):
    """클래스별 실제 컵 이미지 조각 (조금 여유를 두고 자름)."""
    bank = {i: [] for i in range(len(CLASSES))}
    for it in items:
        img = cv2.imread(it['path'])
        for c, x, y, bw, bh in it['boxes']:
            x0, y0, x1, y1 = int(x), int(y), int(x + bw), int(y + bh)
            bank[c].append(img[y0:y1, x0:x1].copy())
    return bank


def swap_cups(img, boxes, bank, rng, ratio=0.4):
    """박스의 일부를 다른 클래스의 실제 컵 조각으로 교체 (가장자리는 부드럽게 섞음)."""
    img = img.copy()
    new_boxes = []
    for c, x, y, bw, bh in boxes:
        if rng.random() < ratio:
            nc = rng.choice([k for k in range(len(CLASSES)) if k != c])
            crop = rng.choice(bank[nc])
            x0, y0, w, h = int(x), int(y), int(bw), int(bh)
            patch = cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
            # 가장자리 페더링: 중앙 1, 테두리로 갈수록 0 → 배경과 자연스럽게
            mask = np.zeros((h, w), np.float32)
            m = max(2, min(w, h) // 12)
            cv2.rectangle(mask, (m, m), (w - m, h - m), 1.0, -1)
            mask = cv2.GaussianBlur(mask, (0, 0), m / 2)[..., None]
            roi = img[y0:y0 + h, x0:x0 + w].astype(np.float32)
            img[y0:y0 + h, x0:x0 + w] = (patch * mask + roi * (1 - mask)).astype(np.uint8)
            c = nc
        new_boxes.append((c, x, y, bw, bh))
    return img, new_boxes


def photometric(img, rng):
    out = img.astype(np.float32)
    out = out * rng.uniform(0.7, 1.3) + rng.uniform(-25, 25)        # 대비·밝기
    hsv = cv2.cvtColor(np.clip(out, 0, 255).astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.int16)
    hsv[..., 0] = (hsv[..., 0] + rng.randint(-4, 4)) % 180          # 색조는 약하게 (당근·토마토 색 보존)
    hsv[..., 1] = np.clip(hsv[..., 1] * rng.uniform(0.8, 1.2), 0, 255)
    out = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
    if rng.random() < 0.3:
        out = cv2.GaussianBlur(out, (0, 0), rng.uniform(0.5, 1.5))
    if rng.random() < 0.3:
        noise = np.random.default_rng(rng.randint(0, 1 << 30)).normal(0, rng.uniform(3, 8), out.shape)
        out = np.clip(out.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return out


def motion_blur(img, rng):
    """카메라·로봇팔이 움직이는 중에 찍힌 듯한 흔들림 (방향 무작위, 주로 가로)."""
    n = rng.choice([5, 7, 9, 13, 17, 21])
    k = np.zeros((n, n), np.float32)
    k[n // 2] = 1.0
    M = cv2.getRotationMatrix2D((n / 2 - 0.5, n / 2 - 0.5), rng.uniform(-30, 30), 1.0)
    k = cv2.warpAffine(k, M, (n, n))
    return cv2.filter2D(img, -1, k / max(k.sum(), 1e-6))


def color_cast(img, rng):
    """조명 색 변화 (노란 백열등·푸른 형광등 등): 채널별 이득."""
    g = np.array([rng.uniform(0.7, 1.2), rng.uniform(0.85, 1.1), rng.uniform(0.75, 1.25)], np.float32)
    return np.clip(img.astype(np.float32) * g, 0, 255).astype(np.uint8)


def _occluder_color(rng):
    return rng.choice([(95, 95, 100), (60, 60, 60), (200, 200, 200),          # 로봇팔·그리퍼 회색
                       (90, 140, 200), (60, 120, 170),                         # 살색 계열
                       tuple(rng.randint(0, 255) for _ in range(3))])


def _covered(b, rects):
    _, x, y, bw, bh = b
    a = 0.0
    for x0, y0, x1, y1 in rects:
        a += max(0, min(x + bw, x1) - max(x, x0)) * max(0, min(y + bh, y1) - max(y, y0))
    return a / (bw * bh)


def occlude(img, boxes, rng):
    """컵 일부 가림(라벨은 컵 전체 유지) + 가끔 위에서 내려오는 로봇팔.
    60% 넘게 가려진 컵은 라벨에서 뺀다 (보이지 않는 것을 찾으라고 가르치지 않게)."""
    img = img.copy()
    h, w = img.shape[:2]
    rects = []
    for c, x, y, bw, bh in boxes:
        if rng.random() < 0.25:
            f = rng.uniform(0.15, 0.45)
            side = rng.choice('lrtb')
            r = {'l': (x, y, x + bw * f, y + bh), 'r': (x + bw * (1 - f), y, x + bw, y + bh),
                 't': (x, y, x + bw, y + bh * f), 'b': (x, y + bh * (1 - f), x + bw, y + bh)}[side]
            rects.append(r)
    if rng.random() < 0.35:
        cx, bot, aw = rng.uniform(0.1, 0.7) * w, rng.uniform(0.3, 0.85) * h, rng.uniform(0.03, 0.05) * w
        rects += [(cx - aw, 0, cx + aw, bot), (cx - aw * 1.7, bot - 0.06 * h, cx + aw * 1.7, bot + 0.04 * h)]
    for x0, y0, x1, y1 in rects:
        cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), _occluder_color(rng), -1)
    return img, [b for b in boxes if _covered(b, rects) <= 0.6]


def geometric(img, boxes, rng):
    """약한 확대·이동·회전. 박스는 네 꼭짓점을 변환한 외접 사각형으로."""
    h, w = img.shape[:2]
    s = rng.uniform(0.9, 1.15)
    a = rng.uniform(-3, 3)
    tx, ty = rng.uniform(-0.05, 0.05) * w, rng.uniform(-0.05, 0.05) * h
    M = cv2.getRotationMatrix2D((w / 2, h / 2), a, s)
    M[:, 2] += (tx, ty)
    out = cv2.warpAffine(img, M, (w, h), borderMode=cv2.BORDER_REFLECT_101)
    new = []
    for c, x, y, bw, bh in boxes:
        pts = np.array([[x, y], [x + bw, y], [x, y + bh], [x + bw, y + bh]], np.float32)
        p = cv2.transform(pts[None], M)[0]
        x0, y0 = np.clip(p.min(0), 0, [w, h])
        x1, y1 = np.clip(p.max(0), 0, [w, h])
        if (x1 - x0) * (y1 - y0) < 0.5 * bw * bh * s * s:  # 절반 이상 화면 밖이면 버림
            continue
        new.append((c, float(x0), float(y0), float(x1 - x0), float(y1 - y0)))
    return out, new


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--per-image', type=int, default=20, help='학습 이미지 1장당 증강 장수')
    ap.add_argument('--val', default='img_0003,img_0008,img_0013', help='검증용 원본 (쉼표 구분)')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--extra', action='store_true', help='흔들림·조명색·가림 증강 추가 (v2)')
    ap.add_argument('--out', default=OUT, help='출력 폴더')
    args = ap.parse_args()
    rng = random.Random(args.seed)
    val = set(args.val.split(','))
    set_out(os.path.expanduser(args.out))

    shutil.rmtree(OUT, ignore_errors=True)
    for d in ('images', 'labels'):
        for s in ('train', 'val'):
            os.makedirs(os.path.join(OUT, d, s))

    items = load_coco()
    train = [it for it in items if it['stem'] not in val]
    bank = build_crop_bank(train)  # 검증 이미지의 컵은 증강에 쓰지 않음 (정보 누출 방지)
    counts = {'train': 0, 'val': 0}
    for it in items:
        img = cv2.imread(it['path'])
        split = 'val' if it['stem'] in val else 'train'
        write_yolo(img, it['boxes'], split, it['stem'])
        counts[split] += 1
        if split == 'val':
            continue
        for k in range(args.per_image):
            a_img, a_boxes = swap_cups(img, it['boxes'], bank, rng)
            if args.extra and rng.random() < 0.5:
                a_img, a_boxes = occlude(a_img, a_boxes, rng)
            a_img = photometric(a_img, rng)
            if args.extra and rng.random() < 0.4:
                a_img = color_cast(a_img, rng)
            if args.extra and rng.random() < 0.3:
                a_img = motion_blur(a_img, rng)
            a_img, a_boxes = geometric(a_img, a_boxes, rng)
            write_yolo(a_img, a_boxes, 'train', f"{it['stem']}_aug{k:02d}")
            counts['train'] += 1

    with open(os.path.join(OUT, 'data.yaml'), 'w') as f:
        f.write(f'path: {OUT}\ntrain: images/train\nval: images/val\n')
        f.write(f'nc: {len(CLASSES)}\nnames: {CLASSES}\n')
    print(f"완료: train {counts['train']}장 (원본 {len(train)} + 증강), val {counts['val']}장 (원본만)")
    print(f'출력: {OUT}/data.yaml')


if __name__ == '__main__':
    main()
