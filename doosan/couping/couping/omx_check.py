"""OMX 연결 확인: 보드에 연결된 모터를 찾아 응답만 확인하고 끝낸다 (위치 명령·토크·카메라·정책 사용 안 함).

리더·팔로워 어느 보드든 동작한다 (모터 ID 를 미리 정하지 않고 broadcast ping 으로 찾음).
ROS 가 아니라 LeRobot venv(Python 3.10)에서 실행된다. omx_packer 의 신호 확인 모드가 호출한다.
사용법: ~/venv/il/bin/python omx_check.py [/dev/omx_follower]
"""
import sys

from lerobot.motors.dynamixel import DynamixelMotorsBus


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/omx_follower'
    bus = DynamixelMotorsBus(port=port, motors={})
    bus.connect(handshake=False)
    try:
        found = bus.broadcast_ping()
    finally:
        bus.disconnect(disable_torque=False)  # 토크 상태를 건드리지 않음
    if not found:
        print(f'OMX 응답 없음: {port} 에 연결된 모터가 없음 (전원·케이블 확인)', flush=True)
        sys.exit(1)
    print(f'OMX 응답 OK: 모터 {len(found)}개 (id:model) ' + ' '.join(f'{i}:{m}' for i, m in sorted(found.items())), flush=True)


if __name__ == '__main__':
    main()
