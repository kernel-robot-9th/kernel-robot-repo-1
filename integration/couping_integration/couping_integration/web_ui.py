"""고객·관리자 웹 화면 서버 (ROS 노드 + 웹서버, 추가 설치 없음).

  ros2 run couping_integration web_ui                 # http://<노트북 IP>:8080/  (고객)  /admin (관리자)
  ros2 run couping_integration web_ui --ros-args -p port:=8090

고객 화면: 상품 고르기 → 주문(/couping/order 발행) → 배송 현황(주문 확인·상품 담는 중·포장 시작·배송 시작·배송 완료)
관리자 화면: 들어온 주문, 품목별 상태, 두산 진행, 실시간 카메라, 로봇 상태, 신호 로그, 정지·재개
브라우저와는 HTTP 로만 주고받는다: 상태는 /events (Server-Sent Events) 로 밀어주고,
카메라는 /camera.mjpg (비전 노드의 /couping/vision/image/compressed) 로 보낸다.
같은 와이파이의 휴대폰에서도 고객 화면을 열 수 있다 (실습실 내부망 전용, 인증 없음).
"""
import json
import os
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from couping import catalog

# 오케스트레이터 상태 → 고객에게 보이는 단계 (0 주문 확인 … 4 배송 완료)
STAGE_OF = {'queued': 0, 'started': 0, 'PICKING': 1, 'WAIT_TRANSFER': 1, 'PACKING': 2,
            'DELIVERING': 3, 'RETURNING': 3, 'DONE': 4}
PROGRESS_RE = re.compile(r'(\d+)/(\d+) .*?\((\w+)\) (.*)')


class WebUI(Node):

    def __init__(self):
        super().__init__('web_ui')
        self.declare_parameter('port', 8080)
        self.declare_parameter('camera_rate', 5.0)   # 화면을 보는 동안 비전 노드 실시간 화면 Hz
        self.declare_parameter('robot_id', 'dsr01')
        self.lock = threading.Lock()
        self.changed = threading.Condition(self.lock)
        self.version = 0
        self.orders = {}          # id → {id, items, state, stage, detail, time, lines:[{item, state}], paused}
        self.order_seq = 0
        self.current = None       # 피킹 중인 주문 id
        self.state = 'IDLE'
        self.paused = False
        self.picker = {'text': '', 'busy': False}
        self.logs = []
        self.stock = {}
        self.detected = 0
        self.frame = None
        self.frame_seq = 0
        self.frame_cv = threading.Condition()
        self.camera_clients = 0
        self.camera_on = False
        self.camera_last = 0.0
        self.robots = {'doosan': {'ok': False, 'text': '응답 없음'},
                       'pinky': {'ok': False, 'text': '확인 중'},
                       'omx': {'ok': False, 'text': '확인 중'},
                       'vision': {'ok': False, 'text': '확인 중'}}

        self.order_pub = self.create_publisher(String, '/couping/order', 10)
        self.create_subscription(String, '/couping/order/status', self.on_order_status, 10)
        self.create_subscription(String, '/couping/picker/status', self.on_picker, 10)
        self.create_subscription(String, '/couping/state', self.on_state, 10)
        self.create_subscription(String, '/couping/packer/status', self.on_packer, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/couping/vision/detections', self.on_detections, latched)
        self.create_subscription(CompressedImage, '/couping/vision/image/compressed', self.on_image, 1)
        self.triggers = {n: self.create_client(Trigger, f'/couping/{n}') for n in ('stop', 'resume', 'confirm')}
        self.vision_param = self.create_client(SetParameters, '/shelf_vision/set_parameters')

        rid = self.get_parameter('robot_id').value
        from dsr_msgs2.srv import GetRobotMode, GetRobotState
        self.dsr_state = self.create_client(GetRobotState, f'/{rid}/dsr_controller2/system/get_robot_state')
        self.dsr_mode = self.create_client(GetRobotMode, f'/{rid}/dsr_controller2/system/get_robot_mode')
        self.GetRobotState, self.GetRobotMode = GetRobotState, GetRobotMode
        self.create_timer(2.0, self.poll_robots)
        self.create_timer(1.0, self.camera_watchdog)

    # ---------- 공통 ----------
    def bump(self):
        """상태가 바뀌었음을 /events 구독자에게 알림 (lock 잡은 상태에서 호출)."""
        self.version += 1
        self.changed.notify_all()

    def log(self, text, tone='info'):
        self.logs.insert(0, {'t': time.strftime('%H:%M:%S'), 'msg': text, 'tone': tone})
        del self.logs[40:]

    def snapshot(self):
        orders = sorted(self.orders.values(), key=lambda o: o['id'], reverse=True)
        return {'v': self.version, 'state': self.state, 'paused': self.paused, 'current': self.current,
                'picker': self.picker, 'orders': orders, 'logs': self.logs[:15], 'robots': self.robots,
                'stock': self.stock, 'detected': self.detected,
                'catalog': [{'no': i + 1, 'key': k, 'name': catalog.KOREAN[k]} for i, k in enumerate(catalog.ITEMS)]}

    # ---------- ROS 콜백 ----------
    def on_order_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            oid = d['id']
            o = self.orders.get(oid)
            if o is None:  # 터미널 등 다른 곳에서 들어온 주문도 표시
                o = self.orders[oid] = self.new_order(oid, d.get('items', []), '터미널')
            st = d['state']
            o['state'], o['detail'] = st, d.get('detail', '')
            self.paused = bool(d.get('paused'))
            if st in STAGE_OF:
                o['stage'] = max(o['stage'], STAGE_OF[st])
            if st == 'PICKING':
                self.current = oid
            if st in ('DONE', 'ERROR', 'rejected'):
                if self.current == oid:
                    self.current = None
                if st == 'DONE':
                    for ln in o['lines']:
                        if ln['state'] in ('대기', '담는 중', '다시 담는 중'):
                            ln['state'] = '상자에 넣음'
            if st == 'ERROR':
                o['stage'] = -1
            tone = {'DONE': 'ok', 'ERROR': 'warn', 'rejected': 'warn'}.get(st, 'info')
            self.log(f"주문 {oid} {st} {d.get('detail', '')}".strip(), tone)
            self.bump()

    def on_picker(self, msg):
        data = msg.data
        with self.lock:
            if data == 'running':
                self.picker = {'text': '작업 시작', 'busy': True}
            elif data.startswith('progress:'):
                text = data[len('progress:'):]
                self.picker = {'text': text, 'busy': True}
                m = PROGRESS_RE.match(text)
                o = self.orders.get(self.current)
                if m and o:
                    i = int(m.group(1)) - 1
                    if i < len(o['lines']):
                        word = m.group(4)
                        o['lines'][i]['state'] = ('다시 담는 중' if '다시 시도' in word else '담는 중' if '집는' in word
                                                  else '담지 못함' if ('실패' in word or '건너뜀' in word)
                                                  else '상자에 넣음' if ('놓음' in word or '완료' in word) else word)
                self.log('두산 ' + text, 'info')
            else:
                self.picker = {'text': '완료' if data == 'done' else data, 'busy': False}
                self.log('두산 ' + data, 'ok' if data == 'done' else 'warn')
            self.bump()

    def on_packer(self, msg):
        with self.lock:
            text = {'running': '포장 중', 'done': '포장 완료'}.get(msg.data, msg.data)
            self.robots['omx'] = {'ok': not msg.data.startswith('fail'), 'text': text}
            self.log('OMX ' + msg.data, 'warn' if msg.data.startswith('fail') else 'ok' if msg.data == 'done' else 'info')
            self.bump()

    def on_state(self, msg):
        with self.lock:
            if msg.data != self.state:
                self.state = msg.data
                self.bump()

    def on_detections(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        items = [it for it in d.get('items', []) if not it.get('base_mm') or it['base_mm'][1] >= 380]
        with self.lock:
            self.stock = {k: sum(1 for it in items if it['cls'] == k) for k in catalog.ITEMS}
            self.detected = len(items)
            self.robots['vision'] = {'ok': True, 'text': f'인식 {len(items)}개'}
            self.bump()

    def on_image(self, msg):
        with self.frame_cv:
            self.frame = bytes(msg.data)
            self.frame_seq += 1
            self.frame_cv.notify_all()

    # ---------- 주기 확인 ----------
    def poll_robots(self):
        def nodes_with(topic):
            return self.count_publishers(topic) + self.count_subscribers(topic)
        with self.lock:
            self.robots['pinky'] = ({'ok': True, 'text': 'Nav2 연결됨'} if self.count_publishers('/odom')
                                    else {'ok': False, 'text': '연결 안 됨'})
            if nodes_with('/couping/packer/cmd') <= 1:
                self.robots['omx'] = {'ok': False, 'text': '노드 없음'}
            elif self.robots['omx']['text'] in ('노드 없음', '확인 중'):
                self.robots['omx'] = {'ok': True, 'text': '대기'}
            if not self.count_publishers('/couping/vision/detections'):
                self.robots['vision'] = {'ok': False, 'text': '비전 노드 없음'}
        if self.dsr_state.service_is_ready():
            self.dsr_state.call_async(self.GetRobotState.Request()).add_done_callback(self.on_dsr_state)
        else:
            with self.lock:
                self.robots['doosan'] = {'ok': False, 'text': 'bringup 없음'}
        if self.dsr_mode.service_is_ready():
            self.dsr_mode.call_async(self.GetRobotMode.Request()).add_done_callback(self.on_dsr_mode)
        with self.lock:
            self.bump()

    def on_dsr_state(self, fut):
        r = fut.result()
        if r is None:
            return
        names = {0: '초기화', 1: '대기', 2: '동작 중', 3: '안전정지(SAFE_OFF)', 5: '안전정지', 6: '비상정지',
                 7: '복구 모드'}
        with self.lock:
            d = self.robots['doosan']
            d['ok'] = r.robot_state in (1, 2)
            d['state'] = names.get(r.robot_state, f'상태 {r.robot_state}')
            d['text'] = d['state'] + (' · ' + d['mode'] if d.get('mode') else '')

    def on_dsr_mode(self, fut):
        r = fut.result()
        if r is None:
            return
        with self.lock:
            d = self.robots['doosan']
            d['mode'] = {0: '수동 모드', 1: '자동 모드'}.get(r.robot_mode, '')
            d['text'] = d.get('state', '') + (' · ' + d['mode'] if d['mode'] else '')

    def camera_watchdog(self):
        """카메라 화면을 보는 사람이 있을 때만 비전 노드의 실시간 화면을 켠다."""
        want = self.camera_clients > 0 or time.monotonic() - self.camera_last < 5.0
        if want != self.camera_on and self.vision_param.service_is_ready():
            rate = float(self.get_parameter('camera_rate').value) if want else 0.0
            req = SetParameters.Request()
            req.parameters = [Parameter(name='preview_rate', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=rate))]
            self.vision_param.call_async(req)
            self.camera_on = want

    # ---------- 웹에서 부르는 동작 ----------
    def new_order(self, oid, items, channel):
        return {'id': oid, 'items': list(items), 'state': 'sent', 'stage': 0, 'detail': '',
                'time': time.strftime('%H:%M:%S'), 'channel': channel,
                'lines': [{'item': k, 'name': catalog.KOREAN.get(k, k), 'state': '대기'} for k in items]}

    def place_order(self, items):
        if self.order_pub.get_subscription_count() == 0:
            raise RuntimeError('오케스트레이터가 실행 중이 아닙니다')
        with self.lock:
            nums = [int(k) for k in self.orders if k.isdigit()]
            self.order_seq = max([self.order_seq] + nums) + 1
            oid = f'{self.order_seq:03d}'
            self.orders[oid] = self.new_order(oid, items, '고객 화면')
            self.log(f"고객 화면 ▶ 주문 {oid} ({', '.join(catalog.KOREAN[k] for k in items)})", 'cmd')
            self.bump()
        self.order_pub.publish(String(data=json.dumps({'id': oid, 'items': items}, ensure_ascii=False)))
        return oid

    def call_trigger(self, name):
        cli = self.triggers[name]
        if not cli.wait_for_service(timeout_sec=1.0):
            return False, '오케스트레이터가 실행 중이 아닙니다'
        ev = threading.Event()
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: ev.set())
        if not ev.wait(5.0) or fut.result() is None:
            return False, '응답 없음'
        with self.lock:
            self.log(f'관리자 ▶ {name}: {fut.result().message}', 'warn' if name == 'stop' else 'info')
            self.bump()
        return fut.result().success, fut.result().message


def make_handler(node, web_dir):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *args):  # 요청마다 찍히는 로그 끄기
            pass

        def send_json(self, code, obj):
            body = json.dumps(obj, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def send_file(self, name):
            path = os.path.join(web_dir, name)
            with open(path, 'rb') as f:
                body = f.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split('?')[0]
            if path in ('/', '/customer'):
                return self.send_file('customer.html')
            if path == '/admin':
                return self.send_file('admin.html')
            if path == '/api/state':
                with node.lock:
                    return self.send_json(200, node.snapshot())
            if path == '/events':
                return self.events()
            if path == '/camera.mjpg':
                return self.camera()
            self.send_json(404, {'error': 'not found'})

        def do_POST(self):
            path = self.path.split('?')[0]
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:
                return self.send_json(400, {'ok': False, 'error': 'JSON 형식 오류'})
            if path == '/api/order':
                items = body.get('items') or []
                if not items or len(items) > 6 or any(k not in catalog.ITEMS for k in items):
                    return self.send_json(400, {'ok': False, 'error': '품목은 1~6개, 목록에 있는 것만'})
                try:
                    oid = node.place_order(items)
                except RuntimeError as e:
                    return self.send_json(503, {'ok': False, 'error': str(e)})
                return self.send_json(200, {'ok': True, 'id': oid})
            if path in ('/api/stop', '/api/resume', '/api/confirm'):
                ok, message = node.call_trigger(path.rsplit('/', 1)[1])
                return self.send_json(200 if ok else 503, {'ok': ok, 'message': message})
            self.send_json(404, {'error': 'not found'})

        def events(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            seen = -1
            try:
                while rclpy.ok():
                    with node.lock:
                        if node.version == seen:
                            node.changed.wait(timeout=10.0)
                        if node.version == seen:
                            data = None  # 10초간 변화 없음: 연결 유지용 신호
                        else:
                            seen = node.version
                            data = json.dumps(node.snapshot(), ensure_ascii=False)
                    chunk = f'data: {data}\n\n' if data else ': keepalive\n\n'
                    self.wfile.write(chunk.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

        def camera(self):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            node.camera_clients += 1
            seen = -1
            try:
                while rclpy.ok():
                    node.camera_last = time.monotonic()
                    with node.frame_cv:
                        if node.frame_seq == seen:
                            node.frame_cv.wait(timeout=2.0)
                        if node.frame_seq == seen or node.frame is None:
                            continue
                        seen, frame = node.frame_seq, node.frame
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                node.camera_clients -= 1

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebUI()
    web_dir = os.path.join(get_package_share_directory('couping_integration'), 'web')
    port = node.get_parameter('port').value
    server = ThreadingHTTPServer(('0.0.0.0', port), make_handler(node, web_dir))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'웹 화면: 고객 http://localhost:{port}/   관리자 http://localhost:{port}/admin')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
