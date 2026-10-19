"""이름 붙인 로봇 자세 저장소 (~/couping_ws/calib/poses.json).

  observe      관찰 자세: 팔이 카메라 화면 밖 (기본값 = 준비자세 0,0,90,0,90,0)
  front        진열대 앞 수평 자세: 그리퍼가 진열대를 향해 수평, 진열대에서 충분히 떨어진 곳
  place_above  핑키 적재 위치 바로 위 (내려놓기 전·후 대기)
  place        핑키에 물건을 내려놓는 위치 (주문의 1번째 품목)
  place_2 ~ 6  주문의 2~6번째 품목 자리 (핑키에 놓을 때 주문 품목 수만큼 필요)
각 자세는 관절각(posj, deg)과 그때의 TCP(posx, mm·deg ZYZ)를 함께 저장한다.
"""
import json
import os
import time

POSE_FILE = os.path.expanduser('~/couping_ws/calib/poses.json')
READY_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
NAMES = ('observe', 'front', 'place_above', 'place') + tuple(f'place_{k}' for k in range(2, 7))


def load():
    if os.path.exists(POSE_FILE):
        with open(POSE_FILE) as f:
            return json.load(f)
    return {}


def get(name):
    """저장된 자세 dict 또는 None. observe 는 없으면 준비자세."""
    p = load().get(name)
    if p is None and name == 'observe':
        return {'posj': READY_J, 'posx': None, 'note': '기본 준비자세'}
    return p


def save(name, posj, posx, note=''):
    data = load()
    data[name] = {'posj': [round(float(v), 3) for v in posj],
                  'posx': [round(float(v), 3) for v in posx],
                  'time': time.strftime('%Y-%m-%d %H:%M:%S'), 'note': note}
    os.makedirs(os.path.dirname(POSE_FILE), exist_ok=True)
    with open(POSE_FILE, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    return data[name]


def place_for(slot):
    """주문의 slot 번째(1부터) 품목을 놓을 자세: 1번은 place, k번은 place_k (없으면 None)."""
    return get('place') if slot == 1 else get(f'place_{slot}')
