"""두산을 준비자세(관절 0,0,90,0,90,0)로 저속 이동. 진열대 쪽에 있으면 먼저 뒤로 뺀다.

  python3 ~/couping_ws/tools/go_ready.py      (ROS 환경에서, 로봇에서 손을 뗀 상태로)
원래 수동 모드였으면 끝나고 수동 모드로 되돌린다.
"""
import time

import rclpy

from couping.dsr_util import connect

rclpy.init()
node = rclpy.create_node('go_ready', namespace='dsr01')
dsr = connect(node)
start = dsr.get_robot_mode()
print('현재 관절', [round(v, 1) for v in dsr.get_current_posj()])
dsr.set_robot_mode(dsr.ROBOT_MODE_AUTONOMOUS)
time.sleep(0.5)
p, _ = dsr.get_current_posx()
p = [float(v) for v in p]
if p[1] > 100:  # 진열대(+Y) 쪽에 있으면 같은 자세로 뒤로 빼기
    back = list(p)
    back[1] = 90.0
    r = dsr.movel(dsr.posx(*back), vel=30, acc=30)
    print('뒤로 빼기', 'OK' if r == 0 else f'실패 {r}')
r = dsr.movej(dsr.posj(0, 0, 90, 0, 90, 0), vel=20, acc=20)
print('준비자세 이동', 'OK' if r == 0 else f'실패 {r}')
if start == dsr.ROBOT_MODE_MANUAL:
    dsr.set_robot_mode(dsr.ROBOT_MODE_MANUAL)
    print('수동 모드로 복귀')
node.destroy_node()
rclpy.shutdown()
