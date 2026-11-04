"""양팔 OMX: 어느 보드·손목 카메라가 왼팔인지 찾기 (팔에 명령은 보내지 않고 읽기만 함).

  ~/venv/il/bin/python omx_identify.py --ports /dev/ttyACM0 /dev/ttyACM1 --cams 4 14 --seconds 10
실행 중에 '왼팔' 만 손으로 천천히 움직이면, 관절 값이 가장 많이 바뀐 보드와
영상이 가장 많이 바뀐 손목 카메라를 왼팔로 알려준다.
"""
import argparse
import time

import cv2
import numpy as np
from dynamixel_sdk import PacketHandler, PortHandler

PRESENT_POSITION = 132  # XL430 / XL330 공통 주소 (4바이트)


def read_positions(port, packet, ids):
    out = []
    for i in ids:
        val, res, err = packet.read4ByteTxRx(port, i, PRESENT_POSITION)
        out.append(val if res == 0 else np.nan)
    return np.array(out, float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ports', nargs='+', required=True)
    ap.add_argument('--cams', nargs='+', type=int, default=[])
    ap.add_argument('--ids', nargs='+', type=int, default=[11, 12, 13, 14, 15, 16])
    ap.add_argument('--seconds', type=float, default=10.0)
    args = ap.parse_args()

    packet = PacketHandler(2.0)
    ports = []
    for name in args.ports:
        p = PortHandler(name)
        if not (p.openPort() and p.setBaudRate(1000000)):
            raise SystemExit(f'포트 열기 실패: {name}')
        ports.append(p)
    caps = []
    for n in args.cams:
        c = cv2.VideoCapture(f'/dev/video{n}', cv2.CAP_V4L2)
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        caps.append(c)

    start = [read_positions(p, packet, args.ids) for p in ports]
    move = [0.0] * len(ports)
    prev = [None] * len(caps)
    motion = [0.0] * len(caps)
    print(f'{args.seconds:.0f}초 동안 왼팔만 천천히 움직여 주세요...', flush=True)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        for k, p in enumerate(ports):
            d = np.nanmax(np.abs(read_positions(p, packet, args.ids) - start[k]))
            move[k] = max(move[k], float(d))
        for k, c in enumerate(caps):
            ok, img = c.read()
            if not ok:
                continue
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
            if prev[k] is not None:
                motion[k] += float(np.mean(np.abs(g - prev[k])))
            prev[k] = g
        time.sleep(0.05)
    for p in ports:
        p.closePort()
    for c in caps:
        c.release()

    for name, m in zip(args.ports, move):
        print(f'보드 {name}: 관절 최대 변화 {m:.0f} (단위: 4096=1바퀴)')
    for n, m in zip(args.cams, motion):
        print(f'카메라 /dev/video{n}: 영상 변화 {m:.0f}')
    print(f'→ 왼팔 보드: {args.ports[int(np.argmax(move))]}')
    if caps:
        print(f'→ 왼손목 카메라: /dev/video{args.cams[int(np.argmax(motion))]}')


if __name__ == '__main__':
    main()
