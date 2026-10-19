"""1차 연동 오케스트레이터 (SR_10~19, 33~34, 39~41).

주문은 /couping/order (std_msgs/String, JSON {"id": "001", "items": ["tomato", "salad"]}) 로 받아
대기열에 넣고 하나씩
  PICKING(두산이 상자에 담음) → PACKING(OMX 포장) → DELIVERING(핑키가 배송 구역으로 이송) → [RETURNING] → DONE
순서로 진행한다. 현재 상태는 /couping/state, 주문별 진행은 /couping/order/status (JSON) 에 발행한다.
주문 입력 터미널: ros2 run couping_integration order
/couping/start (std_srvs/Trigger) 는 품목 없는 주문(두산에 start)을 넣는다 (신호 확인용).
/couping/stop 을 받거나 단계가 실패·시간 초과되면 핑키 주행을 취소하고 ERROR 로 멈추며,
남은 주문은 /couping/resume 을 받을 때까지 대기열에서 기다린다.

signal_only: true 이면 로봇 작업 없이 신호만 주고받는다. 핑키는 Nav2 대신
핑키에서 도는 pinky_signal 노드에 /couping/pinky/cmd 를 보내고 답을 받으며, 복귀는 생략한다.

enable_picker / enable_pinky / enable_packer 를 false 로 두면 해당 단계를 건너뛰므로
로봇이 일부만 준비된 상태에서도 연동 흐름을 시험할 수 있다.
"""
import collections
import json
import math
import signal
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from couping import catalog


class StageError(Exception):
    pass


class Orchestrator(Node):

    def __init__(self):
        super().__init__('orchestrator')
        for name, default in [('signal_only', False), ('enable_picker', True), ('enable_pinky', True),
                              ('enable_packer', True), ('return_home', True),
                              ('manual_transfer', False), ('transfer_timeout', 600.0),
                              ('pause_on_error', False),  # false: 실패한 주문은 넘기고 다음 주문 진행
                              ('picker_timeout', 120.0),  # 품목 하나당
                              ('nav_timeout', 120.0),
                              ('packer_timeout', 90.0),
                              ('nav_action', '/navigate_to_pose'), ('map_frame', 'map'),
                              ('deliver_pose', [0.0, 0.0, 0.0]), ('home_pose', [0.0, 0.0, 0.0])]:
            self.declare_parameter(name, default)

        self.state = 'IDLE'
        self.state_pub = self.create_publisher(String, '/couping/state', 10)
        self.create_timer(1.0, self.publish_state)  # 관제 화면이 늦게 붙어도 보이도록 주기 발행

        units = ('picker', 'pinky', 'packer')
        self.status = {u: None for u in units}
        self.busy = {u: False for u in units}  # running ~ done/fail 사이
        self.status_events = {u: threading.Event() for u in units}
        self.cmd_pubs = {}
        for unit in units:
            self.cmd_pubs[unit] = self.create_publisher(String, f'/couping/{unit}/cmd', 10)
            self.create_subscription(String, f'/couping/{unit}/status',
                                     lambda msg, u=unit: self.on_status(u, msg), 10)

        self.nav = ActionClient(self, NavigateToPose, self.get_parameter('nav_action').value)
        self.nav_goal = None
        self.stop_event = threading.Event()

        self.orders = collections.deque()
        self.order_cv = threading.Condition()
        self.paused = False      # 오류·정지 뒤에는 /couping/resume 전까지 다음 주문을 시작하지 않음
        self.current = None
        self.order_pub = self.create_publisher(String, '/couping/order/status', 10)
        self.create_subscription(String, '/couping/order', self.on_order, 10)

        self.create_service(Trigger, '/couping/start', self.on_start)
        self.create_service(Trigger, '/couping/stop', self.on_stop)
        self.create_service(Trigger, '/couping/resume', self.on_resume)
        self.confirm_event = threading.Event()   # 핑키 없이 사람이 박스를 옮긴 뒤 /couping/confirm
        self.create_service(Trigger, '/couping/confirm', self.on_confirm)
        self.worker = threading.Thread(target=self.order_loop, daemon=True)
        self.worker.start()
        self.get_logger().info('orchestrator 준비 완료. 주문: ros2 run couping_integration order')

    # ---------- 상태 ----------
    def set_state(self, state):
        self.state = state
        self.publish_state()
        self.get_logger().info(f'STATE → {state}')

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def publish_order(self, order, state, detail=''):
        self.order_pub.publish(String(data=json.dumps(
            {'id': order['id'], 'items': order['items'], 'state': state, 'detail': detail,
             'queue': len(self.orders), 'paused': self.paused}, ensure_ascii=False)))

    def on_status(self, unit, msg):
        if msg.data == 'running':
            self.busy[unit] = True
            return
        if msg.data.startswith('progress:'):  # 진행 상황은 주문 터미널이 직접 표시
            return
        self.busy[unit] = False
        self.status[unit] = msg.data
        self.status_events[unit].set()

    # ---------- 주문·서비스 ----------
    def enqueue(self, order):
        with self.order_cv:
            self.orders.append(order)
            self.order_cv.notify()
        ahead = len(self.orders) - 1 + (self.current is not None)
        self.publish_order(order, 'queued', f'앞에 {ahead}건' + (' (일시정지 중: resume 필요)' if self.paused else ''))
        self.get_logger().info(f"주문 {order['id']} 접수: {order['items']} (앞에 {ahead}건)")
        return ahead

    def on_order(self, msg):
        try:
            data = json.loads(msg.data)
            order = {'id': str(data.get('id', '?')), 'items': list(data['items'])}
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn(f'주문 형식 오류: {msg.data}')
            return
        bad = [it for it in order['items'] if it not in catalog.ITEMS]
        if bad or not order['items']:
            self.publish_order(order, 'rejected', f'알 수 없는 품목 {bad or "(없음)"}')
            return
        self.enqueue(order)

    def on_start(self, request, response):
        ahead = self.enqueue({'id': 'start', 'items': []})
        response.success, response.message = True, f'접수 (앞에 {ahead}건)'
        return response

    def on_stop(self, request, response):
        self.paused = True
        self.stop_event.set()
        self.cancel_nav()
        self.cmd_pubs['picker'].publish(String(data='stop'))  # 지금 품목까지만 집음
        self.cmd_pubs['packer'].publish(String(data='stop'))
        # 두산은 movel 도중 끊을 수 없으므로 비상정지 버튼을 사용
        response.success, response.message = True, (f'정지 요청 전송, 대기 주문 {len(self.orders)}건은 resume 때까지 대기 '
                                                     '(두산 즉시 정지는 비상정지 버튼)')
        return response

    def on_resume(self, request, response):
        with self.order_cv:
            self.paused = False
            self.order_cv.notify()
        response.success, response.message = True, f'재개: 대기 주문 {len(self.orders)}건'
        return response

    def on_confirm(self, request, response):
        if self.state != 'WAIT_TRANSFER':
            response.success, response.message = False, f'지금은 박스 옮김 단계가 아닙니다 ({self.state})'
            return response
        self.confirm_event.set()
        response.success, response.message = True, '박스 옮김 확인 → 포장 시작'
        return response

    def order_loop(self):
        while rclpy.ok():
            with self.order_cv:
                if not self.order_cv.wait_for(lambda: self.orders and not self.paused, timeout=0.5):
                    continue
                self.current = self.orders.popleft()
            self.stop_event.clear()
            try:
                self.run_order(self.current)
            finally:
                self.current = None

    # ---------- 시나리오 ----------
    def run_order(self, order):
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        t0 = time.monotonic()
        items = order['items']
        self.publish_order(order, 'started')
        try:
            if p('enable_picker'):
                self.set_state('PICKING')
                self.publish_order(order, 'PICKING')
                self.run_unit('picker', p('picker_timeout') * max(1, len(items)),
                              'pick:' + ','.join(items) if items else 'start')
            # 상자는 두산이 담은 자리에서 OMX 가 바로 포장한다 (핑키에 싣고 옮기지 않음, 10/19 변경)
            if p('manual_transfer') and p('enable_packer'):
                # 핑키 대신 사람이 박스를 OMX 앞으로 옮기고 관리자 화면에서 확인
                self.confirm_event.clear()
                self.set_state('WAIT_TRANSFER')
                self.publish_order(order, 'WAIT_TRANSFER', '박스를 OMX 앞으로 옮긴 뒤 관리자 화면에서 확인')
                self.wait_or_stop(self.confirm_event.is_set, p('transfer_timeout'))
            if p('enable_packer'):
                self.set_state('PACKING')
                self.publish_order(order, 'PACKING')
                self.run_unit('packer', p('packer_timeout'))
            if p('enable_pinky'):
                # 포장 완료 신호를 받은 핑키가 배송 구역으로 이송
                self.set_state('DELIVERING')
                self.publish_order(order, 'DELIVERING')
                if p('signal_only'):
                    self.run_unit('pinky', p('nav_timeout'))
                else:
                    self.navigate(p('deliver_pose'), p('nav_timeout'))
            if p('enable_pinky') and p('return_home') and not p('signal_only'):
                self.set_state('RETURNING')
                self.publish_order(order, 'RETURNING')
                self.navigate(p('home_pose'), p('nav_timeout'))
            self.set_state('DONE')
            self.publish_order(order, 'DONE', f'{time.monotonic() - t0:.0f}s')
            self.get_logger().info(f"주문 {order['id']} 완료: {time.monotonic() - t0:.1f}s")
        except StageError as e:
            self.cancel_nav()
            # 정지 버튼(stopped)이면 항상 멈추고, 그 외 실패는 설정에 따라 넘기고 다음 주문 진행
            if p('pause_on_error') or str(e) == 'stopped':
                self.paused = True
            stage = self.state
            self.set_state(f'ERROR:{stage}:{e}')
            self.publish_order(order, 'ERROR', f'{stage}: {e}')

    def wait_or_stop(self, done, timeout):
        """done() 이 참이 될 때까지 기다린다. 정지 요청 또는 시간 초과면 StageError."""
        deadline = time.monotonic() + timeout
        while not done():
            if self.stop_event.is_set():
                raise StageError('stopped')
            if time.monotonic() > deadline:
                raise StageError(f'timeout {timeout:.0f}s')
            time.sleep(0.1)

    def run_unit(self, unit, timeout, cmd='start'):
        self.status_events[unit].clear()
        self.status[unit] = None
        if self.count_subscribers(f'/couping/{unit}/cmd') == 0:
            raise StageError(f'{unit} 노드가 없음')
        if self.busy[unit]:
            # 앞 주문을 정지·시간 초과로 끊었으면 그 작업이 끝날 때까지 기다림
            # (늦게 온 done/fail 을 이번 주문 결과로 착각하지 않도록)
            self.get_logger().info(f'{unit} 이전 작업이 끝나길 기다림')
            self.wait_or_stop(lambda: not self.busy[unit], timeout)
            self.status_events[unit].clear()
            self.status[unit] = None
        self.cmd_pubs[unit].publish(String(data=cmd))
        self.wait_or_stop(self.status_events[unit].is_set, timeout)
        if self.status[unit] != 'done':
            raise StageError(self.status[unit])

    def navigate(self, xyyaw, timeout):
        if not self.nav.wait_for_server(timeout_sec=5.0):
            raise StageError('Nav2 액션 서버 없음 (핑키 navigation 실행·ROS_DOMAIN_ID 확인)')
        x, y, yaw = xyyaw
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.get_parameter('map_frame').value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self.nav.send_goal_async(goal)
        self.wait_or_stop(send_future.done, 10.0)
        handle = send_future.result()
        if not handle.accepted:
            raise StageError('Nav2 가 목표를 거부함')
        self.nav_goal = handle
        result_future = handle.get_result_async()
        # 정지·시간 초과로 예외가 나면 nav_goal 을 남겨 두어 run_order 에서 취소되게 함
        self.wait_or_stop(result_future.done, timeout)
        self.nav_goal = None
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise StageError(f'Nav2 실패(status={status})')

    def cancel_nav(self):
        if self.nav_goal is not None:
            self.nav_goal.cancel_goal_async()
            self.nav_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl+C 가 두 번 들어와 정리 중 traceback 나는 것 방지
        node.cancel_nav()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
