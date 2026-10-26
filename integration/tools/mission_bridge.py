"""팀원 핑키 미션(run_arena_mission) 작업 신호 ↔ 두산 피킹 연결.

run_arena_mission 은 도킹 뒤 /pinky/station_task/trigger 에 {"action": "START", "station_id": ...} 를 보내고
/pinky/station_task/complete 를 기다린다. 이 노드는 두산 구역(shelf*) START 를 받으면 두산에
pick:<품목> 을 보내고, done/fail 이 오면 complete 를 보낸다. 포장 구역(packaging)은 OMX 가 없으면 바로 complete.

  python3 mission_bridge.py --items tide            # 어느 선반 구역이든 같은 품목
  python3 mission_bridge.py --order tide,carrot     # 구역별로 나눔: 선반1(8·9·7)=생필품, 선반2(4·2)=식자재
  ros2 run pinky_precision_docking run_arena_mission --step 7 --no-home   # 다른 터미널
두산 picker·비전은 integration.launch.py pinky:=false docking:=false omx:=false vision:=true 로 띄워 둔다.
"""
import argparse
import json

import rclpy
from std_msgs.msg import String

# 구역별 품목 (10/26): 선반1(마커 8·9·7) = 생필품(진열대 왼쪽), 선반2(마커 4·2) = 식자재(진열대 오른쪽)
ZONE_ITEMS = {
    'shelf1': ('tide', 'paper_towel', 'bath_tissue'),
    'shelf2': ('carrot', 'tomato', 'salad'),
}


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--items', help='두산이 집을 품목, 쉼표로 구분 (예: tide,bath_tissue). 어느 선반 구역이든 같은 품목')
    g.add_argument('--order', help='주문 품목, 쉼표로 구분. 도킹한 구역의 품목만 집는다 (ZONE_ITEMS)')
    args = ap.parse_args()
    order = [s.strip() for s in (args.order or args.items).split(',') if s.strip()]

    def items_for(station):
        if args.items:
            return order
        zone = station.split('_')[0]          # shelf1_bay3 → shelf1
        return [it for it in order if it in ZONE_ITEMS.get(zone, ())]

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
        items = items_for(station) if station.startswith('shelf') else []
        if items:
            picker_pub.publish(String(data='pick:' + ','.join(items)))
            log.info(f"→ 두산 pick:{','.join(items)}")
        elif station.startswith('shelf'):
            complete('done (이 구역 주문 품목 없음)')
        else:
            complete('done (OMX 없음: 포장 건너뜀)')

    def on_picker(msg):
        if state['station'] and (msg.data == 'done' or msg.data.startswith('fail')):
            complete(msg.data)
        elif msg.data.startswith('progress:'):
            log.info(msg.data)

    node.create_subscription(String, '/pinky/station_task/trigger', on_trigger, 10)
    node.create_subscription(String, '/couping/picker/status', on_picker, 10)
    log.info('대기 중: ' + (f'두산 구역 도착하면 pick:{args.items}' if args.items else
                           ', '.join(f'{z}→{[it for it in order if it in ZONE_ITEMS[z]]}' for z in ZONE_ITEMS)))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
