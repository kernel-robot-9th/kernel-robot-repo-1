"""주문 입력 터미널: 번호로 품목을 골라 오케스트레이터에 주문을 보내고 진행 상황을 보여준다.

  ros2 run couping_integration order

  주문> 1, 3        토마토 + 샐러드 주문 (1 3 / 13 / tomato,salad 도 가능, 같은 번호 두 번 = 두 개)
  주문> s           대기열·상태 보기      주문> x   정지 (지금 품목까지만)
  주문> r           오류·정지 뒤 재개      주문> q   종료
오케스트레이터(/couping/order)가 떠 있어야 한다.
"""
import json
import threading
import time
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from couping import catalog

GREEN, RED, YELLOW, CYAN, DIM, BOLD, RESET = ('\033[32m', '\033[31m', '\033[33m', '\033[36m',
                                              '\033[2m', '\033[1m', '\033[0m')
PROMPT = '주문> '
STAGE = {'started': '처리 시작', 'MOVING_TO_PICK': '핑키 피킹 위치로 이동', 'PICKING': '두산 피킹', 'MOVING_TO_PACK': '핑키 포장대로 이동',
         'PACKING': 'OMX 포장', 'RETURNING': '핑키 복귀'}


def menu():
    cells = [f'{BOLD}{i}{RESET} {catalog.label(it)}' for i, it in enumerate(catalog.ITEMS, 1)]
    print(f'\n{CYAN}========== Couping 주문 =========={RESET}')
    for k in range(0, len(cells), 3):
        print('  ' + '   '.join(cells[k:k + 3]))
    print(f'{DIM}  번호를 쉼표·띄어쓰기로 입력 (예: 1, 3)   s=상태  x=정지  r=재개  h=메뉴  q=종료{RESET}\n')


class OrderTerminal(Node):

    def __init__(self):
        super().__init__('order_terminal')
        self.pub = self.create_publisher(String, '/couping/order', 10)
        self.create_subscription(String, '/couping/order/status', self.on_order_status, 10)
        self.create_subscription(String, '/couping/picker/status', self.on_picker, 10)
        self.create_subscription(String, '/couping/state', self.on_state, 10)
        self.clients_ = {name: self.create_client(Trigger, f'/couping/{name}') for name in ('stop', 'resume')}
        self.state = None
        self.last = {}  # 주문 id → 마지막 상태 dict
        self.count = 0

    # ---------- 출력 ----------
    def say(self, text):
        # 입력 중인 줄을 지우고 출력한 뒤 프롬프트를 다시 그림
        print(f"\r\033[K[{datetime.now().strftime('%H:%M:%S')}] {text}\n{PROMPT}", end='', flush=True)

    def on_order_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.last[d['id']] = d
        names = ', '.join(catalog.KOREAN.get(i, i) for i in d['items']) or '(신호 확인)'
        head = f"주문 {d['id']} [{names}]"
        st = d['state']
        if st == 'queued':
            self.say(f"{YELLOW}{head} 접수{RESET} - {d['detail']}")
        elif st == 'rejected':
            self.say(f"{RED}{head} 거부: {d['detail']}{RESET}")
        elif st == 'DONE':
            self.say(f"{GREEN}{BOLD}{head} 완료{RESET} ({d['detail']}, 남은 주문 {d['queue']}건)")
        elif st == 'ERROR':
            self.say(f"{RED}{BOLD}{head} 실패{RESET}{RED} - {d['detail']}{RESET}")
            self.say(f"{RED}대기열 일시정지 (남은 주문 {d['queue']}건). 로봇 상태 확인 후 r 입력하면 재개{RESET}")
        else:
            self.say(f'{CYAN}{head} → {STAGE.get(st, st)}{RESET}')

    def on_picker(self, msg):
        data = msg.data
        if data.startswith('progress:'):
            self.say(f'{DIM}   두산: {data[len("progress:"):]}{RESET}')
        elif data.startswith('fail:'):
            self.say(f'{RED}   두산 실패: {data[len("fail:"):]}{RESET}')

    def on_state(self, msg):
        self.state = msg.data

    # ---------- 명령 ----------
    def send_order(self, items):
        t0 = time.monotonic()
        while self.pub.get_subscription_count() == 0 and time.monotonic() - t0 < 3.0:
            time.sleep(0.1)  # 처음 주문은 발견(discovery)될 때까지 잠깐 기다림
        if self.pub.get_subscription_count() == 0:
            print(f'{RED}오케스트레이터가 없습니다. 먼저 launch 를 실행하세요 (README "주문 처리" 참고){RESET}')
            return
        self.count += 1
        oid = f'{self.count:03d}'
        self.pub.publish(String(data=json.dumps({'id': oid, 'items': items}, ensure_ascii=False)))
        print(f"주문 {oid} 전송: {', '.join(catalog.label(i) for i in items)}")

    def call(self, name):
        cli = self.clients_[name]
        if not cli.wait_for_service(timeout_sec=2.0):
            print(f'{RED}오케스트레이터가 없습니다{RESET}')
            return
        fut = cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 5.0:
            time.sleep(0.05)
        print(fut.result().message if fut.done() else f'{RED}응답 없음{RESET}')

    def show(self):
        print(f'오케스트레이터 상태: {self.state or "(수신 안 됨)"}')
        for oid, d in list(self.last.items())[-5:]:
            print(f"  주문 {oid} {', '.join(d['items']) or '-'}: {d['state']} {d['detail']}")


def main():
    rclpy.init()
    node = OrderTerminal()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:  # q 로 종료할 때
            pass

    spinner = threading.Thread(target=spin, daemon=True)
    spinner.start()
    menu()
    try:
        while True:
            line = input(PROMPT).strip()
            if not line:
                continue
            cmd = line.lower()
            if cmd in ('q', 'quit', 'exit'):
                break
            if cmd in ('h', '?', 'help', 'm'):
                menu()
            elif cmd == 's':
                node.show()
            elif cmd == 'x':
                node.call('stop')
            elif cmd == 'r':
                node.call('resume')
            else:
                try:
                    node.send_order(catalog.parse(line))
                except ValueError as e:
                    print(f'{RED}{e}{RESET}')
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        rclpy.try_shutdown()  # spin 스레드가 먼저 끝나야 종료 시 abort 가 나지 않음
        spinner.join(timeout=2.0)


if __name__ == '__main__':
    main()
