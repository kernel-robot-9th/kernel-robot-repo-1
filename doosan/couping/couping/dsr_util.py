"""DSR_ROBOT2 연결 도우미.

DSR_ROBOT2 는 import 직후 서비스가 아직 발견되기 전에 호출하면 요청이 사라져
spin_until_future_complete 가 영원히 기다린다(간헐적 멈춤). 그래서 쓰는 서비스들이
준비될 때까지 기다린 뒤 모듈을 돌려준다.
"""
import time

CLIENTS = ('_ros2_get_robot_mode', '_ros2_set_robot_mode', '_ros2_get_current_posj',
           '_ros2_get_current_posx', '_ros2_movej', '_ros2_movel')


def connect(node, robot_id='dsr01', model='e0509', timeout=15.0):
    import DR_init
    DR_init.__dsr__id, DR_init.__dsr__model, DR_init.__dsr__node = robot_id, model, node
    import DSR_ROBOT2 as dsr
    t0 = time.time()
    for name in CLIENTS:
        cli = getattr(dsr, name, None)
        if cli is not None and not cli.wait_for_service(timeout_sec=max(0.1, timeout - (time.time() - t0))):
            raise RuntimeError(f'두산 서비스 연결 안 됨: {name} (bringup·ROS_DOMAIN_ID 확인)')
    time.sleep(0.3)
    return dsr
