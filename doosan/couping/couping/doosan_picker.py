"""두산 로봇팔 피킹 노드 (SR_07~10).

/couping/picker/cmd 로 받는 명령
  pick:tomato,salad   주문 품목을 차례로 비전으로 찾아 수평으로 집고 핑키에 놓는다
                      (k번째 품목 → place_k 자세, 없으면 place)
  stop                지금 품목까지만 하고 나머지는 하지 않음 (동작 중 즉시 정지는 비상정지 버튼)
  start               signal_only 일 때: 현재 관절 각도만 읽고 done
/couping/picker/status 로 보내는 상태
  running → progress:<진행 상황> … → done 또는 fail:<이유>

핑키 자세(place_above, place)가 아직 없으면 no_place 에 따라
  return: 집은 물건을 진열대 제자리에 도로 놓음 (반복 시험용, 기본)
  hold:   front 에서 들고 대기 (한 품목 주문만 가능)
진열대에 없거나 도달할 수 없는 품목은 건너뛰고 나머지를 집은 뒤 fail 로 알린다.

DSR_ROBOT2 함수는 내부에서 이 노드를 spin 하므로 콜백 안에서 부르면 멈춘다.
그래서 콜백은 명령을 큐에 넣기만 하고, 실제 동작은 메인 루프에서 실행한다.
launch 에서 namespace 를 로봇 이름(dsr01)으로 줘야 DSR 서비스가 연결된다.

시험용 파라미터
  signal_only: true  동작 대신 현재 관절 각도만 읽고 done (두산 신호 확인)
  dry_run: true      인식·계획만 하고 로봇은 안 움직임 (두산 연결 불필요, 비전 노드 필요)
  fake: true         로봇·카메라 없이 품목당 2초 뒤 완료 (주문 신호 흐름 확인)
"""
import collections
import signal
import time

import rclpy
from std_msgs.msg import String

from couping import catalog, pick_core
from couping.dsr_util import connect

PARAMS = ('open', 'release', 'force', 'grip_wait', 'vel', 'acc', 'vel_slow', 'velj', 'accj', 'approach', 'depth', 'dz',
          'lift', 'retries')
RESULT = {'placed': '상자에 놓음', 'returned': '진열대에 도로 놓음', 'holding': '들고 대기'}


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('doosan_picker')
    log = node.get_logger()

    node.declare_parameter('robot_model', 'e0509')
    for name in ('signal_only', 'dry_run', 'fake'):
        node.declare_parameter(name, False)
    node.declare_parameter('no_place', 'return')
    for k in PARAMS:  # 속도·그리퍼 값 (기본값은 pick_core.DEFAULTS)
        node.declare_parameter(k, pick_core.DEFAULTS[k])

    def get(name):
        return node.get_parameter(name).value

    status_pub = node.create_publisher(String, '/couping/picker/status', 10)
    queue = collections.deque()
    stop = {'requested': False}

    def on_cmd(msg):
        if msg.data == 'stop':
            stop['requested'] = True
        else:
            queue.append(msg.data)

    node.create_subscription(String, '/couping/picker/cmd', on_cmd, 10)

    def publish(text):
        status_pub.publish(String(data=text))
        log.info(f'status: {text}')

    signal_only, dry_run, fake = get('signal_only'), get('dry_run'), get('fake')
    dsr = None
    if not (dry_run or fake):
        dsr = connect(node, robot_id=node.get_namespace().strip('/') or 'dsr01', model=get('robot_model'))

    def read_joints():
        ret = dsr.get_current_posj()
        if isinstance(ret, int):  # 실패 시 -1
            raise RuntimeError('get_current_posj 실패 (컨트롤러 응답 없음)')
        log.info('두산 응답 OK, 현재 관절(deg): ' + ', '.join(f'{v:.1f}' for v in ret))

    def pump(sec=0.0):
        """stop 같은 명령 콜백 처리. DSR 함수도 내부에서 spin 하지만 품목 사이에서도 확인한다."""
        end = time.monotonic() + sec
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= end:
                return

    def run_order(items):
        p = {k: get(k) for k in PARAMS}
        if not fake:
            pick_core.check_slots(len(items))
        start_mode = dsr.get_robot_mode() if dsr else None
        skipped = []
        try:
            for i, item in enumerate(items, 1):
                pump()
                if stop['requested']:
                    raise pick_core.PickError(f'정지 요청 ({i - 1}/{len(items)} 품목 완료)')
                head = f'{i}/{len(items)} {catalog.label(item)}'
                publish(f'progress:{head} 집는 중')
                if fake:
                    pump(2.0)
                    publish(f'progress:{head} 완료 (fake)')
                    continue
                try:
                    res = pick_core.pick_verified(
                        node, dsr, item, p, log=log.info, dry_run=dry_run,
                        on_retry=lambda k, n, why, h=head: publish(f'progress:{h} 다시 시도 {k}/{n} - {why}'),
                        slot=i, no_place=get('no_place'))
                except pick_core.PickFailed as e:  # 다시 시도해도 실패: 로봇은 관찰 자세, 다음 품목 계속
                    skipped.append(item)
                    publish(f'progress:{head} 실패 - {e}')
                    continue
                except pick_core.NotPickable as e:  # 로봇은 관찰 자세: 다음 품목 계속
                    skipped.append(item)
                    publish(f'progress:{head} 건너뜀 - {e}')
                    continue
                done = RESULT[res['result']]
                if res.get('attempts'):  # 카메라로 진열대에서 빠진 것까지 확인함
                    done += ' (확인 완료' + (f", {res['attempts']}번째 시도" if res['attempts'] > 1 else '') + ')'
                publish(f"progress:{head} {'계획 OK (dry_run)' if dry_run else done}")
                if res['result'] == 'holding' and i < len(items) and not dry_run:
                    raise pick_core.PickError('물건을 들고 있어 다음 품목을 집을 수 없습니다 '
                                              '(no_place: return 으로 바꾸거나 핑키 자세 저장)')
        finally:
            if dsr and start_mode == dsr.ROBOT_MODE_MANUAL:
                dsr.set_robot_mode(dsr.ROBOT_MODE_MANUAL)  # 시작 전처럼 직접교시 가능하게
        if skipped:
            raise pick_core.PickError('못 집은 품목: ' + ', '.join(catalog.label(s) for s in skipped))

    tags = [n for n, on in (('신호 확인', signal_only), ('dry_run', dry_run), ('fake', fake)) if on]
    log.info(f"doosan_picker 준비 완료{' [' + ', '.join(tags) + ']' if tags else ''}. "
             '/couping/picker/cmd 대기 중 (pick:<품목,...>)')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not queue:
                continue
            cmd = queue.popleft()
            stop['requested'] = False
            publish('running')
            try:
                if signal_only:
                    if not fake:
                        read_joints()
                elif cmd.startswith('pick:'):
                    items = [s.strip() for s in cmd[len('pick:'):].split(',') if s.strip()]
                    bad = [s for s in items if s not in catalog.ITEMS]
                    if bad or not items:
                        raise pick_core.PickError(f'알 수 없는 품목 {bad or "(없음)"}. 가능: {", ".join(catalog.ITEMS)}')
                    run_order(items)
                else:
                    raise pick_core.PickError(f"알 수 없는 명령 '{cmd}' (pick:<품목,...>)")
                publish('done')
            except Exception as e:  # DSR 오류도 상태로 알려야 오케스트레이터가 멈춤
                publish(f'fail:{e}')
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl+C 가 두 번 들어와 정리 중 traceback 나는 것 방지
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
