"""OMX 포장 노드 (SR_34, 36).

/couping/packer/cmd 에 "start" 를 받으면 OMX 팀의 omx/scripts/run_act.sh (양팔 OMX, LeRobot ACT) 를
한 에피소드 실행하고, 종료 코드에 따라 /couping/packer/status 에 "done" / "fail:<이유>" 를 발행한다.
"stop" 을 받으면 실행 중인 정책을 종료한다.
run_script 를 비우면 예전 단일 팔 ~/il_ws/scripts/eval.sh 를 쓴다.
signal_only: true 이면 정책 대신 omx_check.py 로 ports 의 각 팔 모터 위치만 읽고 done 을 보낸다
(OMX 가 연결돼 신호를 받는지 확인용. 토크·카메라·정책 사용 안 함).
fake: true 이면 OMX 없이 fake_seconds 초 뒤 done (연동 흐름 시험).

LeRobot 은 Python 3.10 venv, ROS 2 Jazzy 는 Python 3.12 라서 같은 프로세스에서 쓸 수 없다.
그래서 별도 프로세스로 실행하고, ROS 환경 변수를 지워서 venv 가 ROS 패키지를 불러오지 않게 한다.
"""
import os
import signal
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 하위 프로세스에 넘기면 venv(Python 3.10)가 ROS(Python 3.12) 패키지를 잘못 불러오는 변수들
ROS_ENV_PREFIXES = ('ROS_', 'AMENT_', 'COLCON_', 'RMW_')
ROS_ENV_NAMES = ('PYTHONPATH', 'CMAKE_PREFIX_PATH', 'LD_LIBRARY_PATH')


class OmxPacker(Node):

    def __init__(self):
        super().__init__('omx_packer')
        self.declare_parameter('run_script', '')   # 양팔: ~/team/kernel-robot-repo-1/omx/scripts/run_act.sh
        self.declare_parameter('left_cam', '')     # 비우면 run_act.sh 기본값
        self.declare_parameter('right_cam', '')
        self.declare_parameter('head_cam', '')
        self.declare_parameter('cam_fourcc', '')   # 예: MJPG (비우면 run_act.sh 기본 형식)
        self.declare_parameter('ports', [''])      # signal_only 때 확인할 팔 포트들
        self.declare_parameter('fake', False)
        self.declare_parameter('fake_seconds', 5.0)
        self.declare_parameter('eval_script', os.path.expanduser('~/il_ws/scripts/eval.sh'))
        self.declare_parameter('policy_path', '')
        self.declare_parameter('dataset_name', 'close_box')
        self.declare_parameter('task', 'Close the box lid')
        self.declare_parameter('episode_time', 30)
        self.declare_parameter('hf_user', 'euchxn')  # 오프라인에서도 동작하도록 고정
        self.declare_parameter('log_dir', os.path.expanduser('~/couping_ws/log_omx'))
        self.declare_parameter('signal_only', False)
        self.declare_parameter('venv_python', os.path.expanduser('~/venv/il/bin/python'))
        self.declare_parameter('port', '/dev/omx_follower')

        self.status_pub = self.create_publisher(String, '/couping/packer/status', 10)
        self.create_subscription(String, '/couping/packer/cmd', self.on_cmd, 10)
        self.proc = None
        self.stop_requested = False
        self.lock = threading.Lock()
        mode = (' [fake: OMX 없이 흉내]' if self.get_parameter('fake').value
                else ' [신호 확인 모드: 모터 응답만 확인]' if self.get_parameter('signal_only').value else '')
        self.get_logger().info(f'omx_packer 준비 완료{mode}. /couping/packer/cmd 대기 중')

    def publish(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f'status: {text}')

    def on_cmd(self, msg):
        if msg.data == 'start':
            with self.lock:
                if self.proc is not None:
                    self.get_logger().warn('이미 실행 중이라 start 를 무시합니다')
                    return
                self.stop_requested = False
                self.proc = self.launch()
            self.publish('running')
            threading.Thread(target=self.wait_proc, daemon=True).start()
        elif msg.data == 'stop':
            self.stop()

    def launch(self):
        get = lambda name: self.get_parameter(name).value  # noqa: E731
        env = {k: v for k, v in os.environ.items()
               if not k.startswith(ROS_ENV_PREFIXES) and k not in ROS_ENV_NAMES}
        os.makedirs(get('log_dir'), exist_ok=True)
        if get('fake'):
            self.log_path = os.path.join(get('log_dir'), 'fake_latest.log')
            sec = float(get('fake_seconds'))
            self.get_logger().info(f'[fake] start 수신 → {sec:.0f}초 뒤 done')
            return subprocess.Popen(['bash', '-c', f'sleep {sec}; echo fake OMX done'],
                                    stdin=subprocess.DEVNULL, stdout=open(self.log_path, 'w'),
                                    stderr=subprocess.STDOUT, start_new_session=True)
        if get('signal_only'):
            self.log_path = os.path.join(get('log_dir'), 'omx_check_latest.log')
            ports = [p for p in get('ports') if p] or [get('port')]
            self.get_logger().info(f'[신호 확인 모드] start 수신 → OMX 모터 응답 확인 {ports}')
            check = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'omx_check.py')
            cmd = ' && '.join(f'"{get("venv_python")}" "{check}" "{p}"' for p in ports)
            return subprocess.Popen(
                ['bash', '-c', cmd], env=env,
                stdin=subprocess.DEVNULL, stdout=open(self.log_path, 'w'), stderr=subprocess.STDOUT,
                start_new_session=True)
        if get('run_script'):
            for k, v in (('POLICY_PATH', get('policy_path')), ('LEFT_CAM', get('left_cam')),
                         ('RIGHT_CAM', get('right_cam')), ('HEAD_CAM', get('head_cam')),
                         ('TASK', get('task')), ('EPISODE_TIME', str(get('episode_time'))),
                         ('CAM_FOURCC', get('cam_fourcc'))):
                if v:
                    env[k] = os.path.expanduser(v)
            script = os.path.expanduser(get('run_script'))
            self.log_path = os.path.join(get('log_dir'), 'run_act_latest.log')
            self.get_logger().info(f'{script} 실행 (policy={env.get("POLICY_PATH", "스크립트 기본값")}), '
                                   f'로그: {self.log_path}')
            return subprocess.Popen(
                ['bash', script], env=env,
                stdin=subprocess.DEVNULL, stdout=open(self.log_path, 'w'), stderr=subprocess.STDOUT,
                start_new_session=True)
        env.update({
            'POLICY_PATH': os.path.expanduser(get('policy_path')),  # ~/ 경로 허용, HF repo id 는 그대로
            'DATASET_NAME': get('dataset_name'),
            'TASK': get('task'),
            'EPISODE_TIME': str(get('episode_time')),
            'HF_USER': get('hf_user'),
            'DISPLAY_DATA': 'false',  # rerun 화면은 손목 카메라를 끊으므로 끔
        })
        self.log_path = os.path.join(get('log_dir'), 'eval_latest.log')
        self.get_logger().info(f'eval.sh 실행 (policy={env["POLICY_PATH"]}), 로그: {self.log_path}')
        # 에피소드 1개, 새 세션으로 띄워서 stop 시 자식 프로세스까지 함께 종료
        return subprocess.Popen(
            ['bash', get('eval_script'), '1'], env=env,
            stdin=subprocess.DEVNULL, stdout=open(self.log_path, 'w'), stderr=subprocess.STDOUT,
            start_new_session=True)

    def wait_proc(self):
        rc = self.proc.wait()
        with self.lock:
            self.proc = None
            stopped = self.stop_requested
        # 하위 프로세스 출력 마지막 두 줄을 ROS 로그에도 남김 (신호 확인 모드에선 모터 위치)
        with open(self.log_path, errors='replace') as f:
            lines = [ln.strip() for ln in f if ln.strip()]
        if lines:
            self.get_logger().info('OMX: ' + ' / '.join(lines[-2:]))
        if stopped:
            self.publish('fail:stopped')
        elif rc == 0:
            self.publish('done')
        else:
            self.publish(f'fail:rc={rc} (로그: {self.log_path})')

    def stop(self):
        with self.lock:
            if self.proc is None:
                return
            self.stop_requested = True
            os.killpg(self.proc.pid, signal.SIGINT)  # lerobot 이 토크를 끄고 정리하도록 SIGINT 먼저


def main(args=None):
    rclpy.init(args=args)
    node = OmxPacker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # Ctrl+C 가 두 번 들어와 정리 중 traceback 나는 것 방지
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
