"""웹 UI 주문 → order_run.sh 실행. 고객 화면에서 고른 품목을 order_run.sh 에 그대로 넘긴다.

고객 화면(web_ui)은 /couping/order 에 {"id": "001", "items": ["tide", "carrot"]} 를 보낸다.
이 노드가 받아서 한 건씩 차례로 `bash integration/scripts/order_run.sh tide,carrot web001_<시각>` 을 실행하고,
출력 문구로 단계를 읽어 /couping/order/status 로 보내 고객·관리자 화면에 진행 상황이 뜨게 한다.
두산 품목별 진행(담는 중·상자에 넣음)은 web_ui 가 /couping/picker/status 를 직접 읽어 표시한다.

  PINKY=2 python3 integration/tools/web_order_run.py        # order_run.sh 와 같은 PINKY (핑키 번호 = 도메인)
  RETURN_HOME=0, SHELF1=, SHELF2=, HOME_FIT= 같은 환경변수는 order_run.sh 로 그대로 넘어간다.
웹 UI 는 같은 도메인에서 orchestrator 없이 띄운다 (둘 다 /couping/order 를 받으면 같은 주문이 두 번 돈다).
"""
import argparse
import json
import os
import queue
import subprocess
import threading
import time

import rclpy
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
ORDER_RUN = os.path.join(HERE, '..', 'scripts', 'order_run.sh')
ITEMS = ('tomato', 'carrot', 'salad', 'bath_tissue', 'tide', 'paper_towel')   # order_run.sh 가 아는 품목

# order_run.sh 출력 문구 → 웹 UI 단계 (web_ui STAGE_OF: 주문 확인 · 상품 담는 중 · 포장 · 배송)
STAGES = [
    ('STARTING PINKY', 'MOVING_TO_PICK'),       # run_arena_mission 선반 구역 출발
    ('[STATION TASK]', 'PICKING'),              # 선반 도킹 끝, 두산 작업 시작
    ('선반에서 이어서 출발', 'MOVING_TO_PACK'),   # pack_run.py --after-pick 시작
    ('OMX 포장 시작', 'PACKING'),
    ('home(P)(으)로 출발', 'RETURNING'),
]


class WebOrderRun:

    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node('web_order_run')
        self.log = self.node.get_logger()
        self.status_pub = self.node.create_publisher(String, '/couping/order/status', 10)
        self.state_pub = self.node.create_publisher(String, '/couping/state', 10)
        self.node.create_subscription(String, '/couping/order', self.on_order, 10)
        self.orders = queue.Queue()
        self.proc = None

    def publish(self, order, state, detail=''):
        self.status_pub.publish(String(data=json.dumps(
            {'id': order['id'], 'items': order['items'], 'state': state, 'detail': detail,
             'queue': self.orders.qsize(), 'paused': False}, ensure_ascii=False)))
        self.state_pub.publish(String(data=state if state not in ('DONE', 'ERROR') else 'IDLE'))
        self.log.info(f"주문 {order['id']} {state} {detail}".strip())

    def on_order(self, msg):
        try:
            order = json.loads(msg.data)
            order['items'] = [str(k) for k in order['items']]
            order['id'] = str(order.get('id') or time.strftime('%H%M%S'))
        except (ValueError, KeyError, TypeError):
            self.log.warn(f'주문 형식이 아님: {msg.data}')
            return
        bad = [k for k in order['items'] if k not in ITEMS]
        if bad or not order['items']:
            self.publish(order, 'rejected', f"모르는 품목: {', '.join(bad)}" if bad else '품목 없음')
            return
        self.publish(order, 'queued', f'앞 주문 {self.orders.qsize() + (self.proc is not None)}건')  # 꺼내기 전에 먼저 알림
        self.orders.put(order)

    def worker(self):
        while rclpy.ok():
            try:
                order = self.orders.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.run(order)
            except Exception as e:  # noqa: BLE001  다음 주문은 계속 받음
                self.publish(order, 'ERROR', str(e))
            self.proc = None

    def run(self, order):
        name = f"web{order['id']}_{time.strftime('%m%d_%H%M%S')}"   # order_run.sh 로그 이름 order_<name>_*.log
        cmd = ['bash', self.args.order_run, ','.join(order['items']), name]
        self.log.info('실행: ' + ' '.join(cmd))
        # stdin 은 order_run.sh 가 미션에 따로 막아 줌. 여기서는 터미널 입력을 넘기지 않게만
        self.proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, env=dict(os.environ, PYTHONUNBUFFERED='1'))
        last_err = ''
        for line in self.proc.stdout:
            line = line.rstrip()
            if not line:
                continue
            print('  [order_run] ' + line, flush=True)
            for key, state in STAGES:
                if key in line:
                    self.publish(order, state)
            if '중단' in line or 'MISSION ABORTED' in line:
                last_err = line.strip()
        rc = self.proc.wait()
        if rc == 0:
            self.publish(order, 'DONE')
        else:
            self.publish(order, 'ERROR', last_err or f'order_run.sh 종료 코드 {rc}')

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()   # order_run.sh 의 trap 이 mission_bridge 를 끔


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--order-run', default=ORDER_RUN, help='주문 실행 스크립트 (시험할 땐 가짜 스크립트)')
    args = ap.parse_args()
    rclpy.init()
    w = WebOrderRun(args)
    w.log.info(f'웹 주문 대기 중 → {os.path.normpath(args.order_run)}')
    threading.Thread(target=w.worker, daemon=True).start()
    try:
        rclpy.spin(w.node)
    except KeyboardInterrupt:
        pass
    w.stop()
    w.node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
