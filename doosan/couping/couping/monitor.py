"""신호 송수신 현황 모니터 — 로봇을 제어하지 않고 신호 토픽만 구독해서 보여준다.

  ros2 run couping monitor

노트북(오케스트레이터) ↔ 두산·핑키·OMX 사이의 start/done 신호와 상태 변화를
시간순으로 출력하고, 한 사이클이 끝나면 구간별 소요 시간을 요약한다.
"""
import signal
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

UNITS = [('picker', '두산'), ('pinky', '핑키'), ('packer', 'OMX ')]
GREEN, RED, YELLOW, CYAN, DIM, RESET = '\033[32m', '\033[31m', '\033[33m', '\033[36m', '\033[2m', '\033[0m'


def fmt(sec):
    return f'{sec * 1000:.0f}ms' if sec < 1 else f'{sec:.2f}s'


def now():
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


class Monitor(Node):

    def __init__(self):
        super().__init__('couping_monitor')
        self.sent_at = {}      # unit → start 보낸 시각
        self.durations = []    # (이름, 초)
        self.cycle_start = None
        self.last_state = None
        for unit, name in UNITS:
            self.create_subscription(String, f'/couping/{unit}/cmd',
                                     lambda m, u=unit, n=name: self.on_cmd(u, n, m), 10)
            self.create_subscription(String, f'/couping/{unit}/status',
                                     lambda m, u=unit, n=name: self.on_status(u, n, m), 10)
        self.create_subscription(String, '/couping/state', self.on_state, 10)
        print(f'{CYAN}=== Couping 신호 모니터 (ROS_DOMAIN_ID 같은지 확인) — Ctrl+C 로 종료 ==={RESET}')
        print(f'{DIM}주문: ros2 run couping order   (신호만: ros2 service call /couping/start std_srvs/srv/Trigger){RESET}\n')

    def on_cmd(self, unit, name, msg):
        self.sent_at[unit] = time.monotonic()
        print(f'[{now()}] 노트북 ──▶ {name}   {YELLOW}{msg.data}{RESET}')

    def on_status(self, unit, name, msg):
        data = msg.data
        if data == 'running':
            print(f'[{now()}] {DIM}{name} ──▶ 노트북   running (수신 확인, 작업 중){RESET}')
            return
        if data.startswith('progress:'):
            print(f'[{now()}] {DIM}{name} ──▶ 노트북   {data[len("progress:"):]}{RESET}')
            return
        took = ''
        if unit in self.sent_at:
            sec = time.monotonic() - self.sent_at.pop(unit)
            took = f'  ({fmt(sec)})'
            self.durations.append((name.strip(), sec))
        color = GREEN if data == 'done' else RED
        print(f'[{now()}] {name} ──▶ 노트북   {color}{data}{RESET}{took}')

    def on_state(self, msg):
        state = msg.data
        if state == self.last_state:  # 오케스트레이터가 1초마다 같은 상태를 재발행하므로 변화만 출력
            return
        self.last_state = state
        if state == 'PICKING' or (self.cycle_start is None and state not in ('IDLE', 'DONE')
                                  and not state.startswith('ERROR')):
            self.cycle_start = time.monotonic()
            self.durations = []
            print(f'\n{CYAN}────────── 새 사이클 시작 ──────────{RESET}')
        color = GREEN if state == 'DONE' else RED if state.startswith('ERROR') else CYAN
        print(f'[{now()}] 상태: {color}{state}{RESET}')
        if state == 'DONE' or state.startswith('ERROR'):
            self.summary(state)

    def summary(self, state):
        total = time.monotonic() - self.cycle_start if self.cycle_start else 0.0
        parts = '  '.join(f'{n} {fmt(s)}' for n, s in self.durations) or '-'
        result = f'{GREEN}성공{RESET}' if state == 'DONE' else f'{RED}실패{RESET}'
        print(f'{CYAN}────────── 결과: {result}{CYAN} | 구간: {parts} | 전체 {fmt(total)} ──────────{RESET}\n')
        self.cycle_start = None


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
