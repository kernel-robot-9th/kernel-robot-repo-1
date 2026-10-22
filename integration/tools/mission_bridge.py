"""팀원 핑키 미션(run_arena_mission) 작업 신호 ↔ 두산 피킹 연결.

run_arena_mission 은 도킹 뒤 /pinky/station_task/trigger 에 {"action": "START", "station_id": ...} 를 보내고
/pinky/station_task/complete 를 기다린다. 이 노드는 두산 구역(shelf*) START 를 받으면 두산에
pick:<품목> 을 보내고, done/fail 이 오면 complete 를 보낸다. 포장 구역(packaging)은 OMX 가 없으면 바로 complete.

  python3 mission_bridge.py --items tide            # 실행해 둔 뒤
  ros2 run pinky_precision_docking run_arena_mission --step 7 --no-home   # 다른 터미널
두산 picker·비전은 integration.launch.py pinky:=false docking:=false omx:=false vision:=true 로 띄워 둔다.
"""
import argparse
import json

import rclpy
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--items', required=True, help='두산이 집을 품목, 쉼표로 구분 (예: tide,bath_tissue)')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('mission_bridge')
    log = node.get_logger()
    picker_pub = node.create_publisher(String, '/couping/picker/cmd', 10)
    complete_pub = node.create_publisher(String, '/pinky/station_task/complete', 10)
    state = {'station': None}

    def complete(text):
        complete_pub.publish(String(data=text))
        log.info(f'→ /pinky/station_task/complete: {text}')
        state['station'] = None

    def on_trigger(msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if data.get('action') != 'START' or state['station']:
            return  # COMPLETED 알림이나 같은 START 중복(3번 보냄)은 무시
        station = data.get('station_id', '')
        state['station'] = station
        log.info(f"작업 START: {station} ({data.get('station_name', '')})")
        if station.startswith('shelf'):
            picker_pub.publish(String(data='pick:' + args.items))
            log.info(f'→ 두산 pick:{args.items}')
        else:
            complete('done (OMX 없음: 포장 건너뜀)')

    def on_picker(msg):
        if state['station'] and (msg.data == 'done' or msg.data.startswith('fail')):
            complete(msg.data)
        elif msg.data.startswith('progress:'):
            log.info(msg.data)

    node.create_subscription(String, '/pinky/station_task/trigger', on_trigger, 10)
    node.create_subscription(String, '/couping/picker/status', on_picker, 10)
    log.info(f'대기 중: 두산 구역 도착하면 pick:{args.items}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
