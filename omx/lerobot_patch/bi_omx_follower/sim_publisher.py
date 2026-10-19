"""bi_omx_follower 의 관절 값을 가상 OMX(Gazebo) 쪽으로 UDP 로 내보낸다.

- 환경변수 OMX_SIM_UDP 가 있을 때만 동작한다. 예) export OMX_SIM_UDP=127.0.0.1:5005
  (없으면 아무 일도 하지 않으므로 평소 실행에는 영향 없음)
- 표준 라이브러리만 사용하고, 어떤 오류가 나도 로봇 제어 루프에는 영향을 주지 않는다.
- 보내는 값은 get_observation() 이 돌려주는 값 그대로(정규화 -100~100, 그리퍼 0~100)이다.
  각도(rad) 변환은 ROS 쪽(pinky_gz_sim/scripts/omx_udp_bridge.py)에서 한다.
"""
import json
import os
import socket
import time

_MOTORS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


class SimPublisher:
    def __init__(self, host=None, port=None, max_hz=30.0):
        self.enabled = host is not None
        self._addr = (host, port)
        self._min_dt = 1.0 / max_hz
        self._last = 0.0
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if self.enabled else None

    @classmethod
    def from_env(cls):
        spec = os.environ.get("OMX_SIM_UDP", "").strip()
        if not spec:
            return cls()
        try:
            host, port = spec.rsplit(":", 1)
            return cls(host, int(port))
        except Exception:
            return cls()

    def publish(self, obs):
        if not self.enabled:
            return
        now = time.monotonic()
        if now - self._last < self._min_dt:
            return
        self._last = now
        try:
            msg = {"t": time.time()}
            for side in ("left", "right"):
                msg[side] = {m: float(obs[f"{side}_{m}.pos"]) for m in _MOTORS if f"{side}_{m}.pos" in obs}
            self._sock.sendto(json.dumps(msg).encode(), self._addr)
        except Exception:
            pass  # 시뮬 전송 실패가 로봇 동작을 막으면 안 된다
