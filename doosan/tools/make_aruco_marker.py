"""핸드-아이 캘리브레이션용 ArUco 마커 PDF (A4, 실제 크기).

  ~/venv/yolo/bin/python ~/couping_ws/tools/make_aruco_marker.py [--size-mm 50] [--id 0]

인쇄할 때 반드시 '실제 크기(100%)'로. 아래 100mm 눈금자를 자로 재서 맞는지 확인.
마커는 위치(중심)만 쓰므로 크기가 조금 달라도 결과에는 영향이 거의 없다.
"""
import argparse
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw

DPI = 300
MM = DPI / 25.4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--size-mm', type=float, default=50.0)
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('--out', default='~/couping_ws/calib/aruco_4x4_id{id}_{size}mm.pdf')
    args = ap.parse_args()

    W, H = int(210 * MM), int(297 * MM)
    page = Image.new('L', (W, H), 255)
    d = ImageDraw.Draw(page)
    side = int(round(args.size_mm * MM))
    marker = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), args.id, side)
    x0, y0 = (W - side) // 2, int(40 * MM)
    page.paste(Image.fromarray(marker), (x0, y0))
    # 오려 붙일 선 (흰 여백 10mm 포함)
    m = int(10 * MM)
    d.rectangle([x0 - m, y0 - m, x0 + side + m, y0 + side + m], outline=160, width=3)
    d.text((x0 - m, y0 + side + m + 15), f'ArUco DICT_4X4_50 id={args.id}  marker {args.size_mm:g} mm  (cut on gray line, keep white border)', fill=0)
    # 100mm 눈금자
    ry, rx = int(200 * MM), int(55 * MM)
    d.line([rx, ry, rx + int(100 * MM), ry], fill=0, width=4)
    for k in range(11):
        x = rx + int(k * 10 * MM)
        d.line([x, ry - (30 if k % 5 == 0 else 15), x, ry], fill=0, width=3)
    d.text((rx, ry + 15), 'print at 100% (actual size): this line must be exactly 100 mm', fill=0)

    out = os.path.expanduser(args.out.format(id=args.id, size=int(args.size_mm)))
    os.makedirs(os.path.dirname(out), exist_ok=True)
    page.save(out, resolution=DPI)
    preview = out.replace('.pdf', '.png')
    page.resize((W // 4, H // 4)).save(preview)
    print('저장:', out)


if __name__ == '__main__':
    main()
