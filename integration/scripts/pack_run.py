#!/usr/bin/env python3
"""진열대 앞 → 포장대 → 도킹 → OMX → 후진 → home 을 한 번에 실행하고, 여러 번 반복해 결과를 기록 (단위 연동 시험용).

  python3 integration/scripts/pack_run.py                       # 생활용품 진열대(daily) 앞에서 출발, 10회
  python3 integration/scripts/pack_run.py --start food --runs 3 # 식품 진열대 앞에서 출발, 3회

전제: 핑키 bringup·Nav2(도메인 51), 카메라 노드, 노트북의 omx_packer 가 켜져 있다.
회차마다 Enter 를 누르면 시작한다. 핑키를 출발 위치에 직접 놓거나, 'a' 를 눌러 지금 위치에서 출발 위치로
자동으로 보낸다 (이 이동은 기록 시간에 넣지 않음). 상자 뚜껑은 회차마다 다시 열어 둔다.
도킹은 팀원 v5.0(feature/amr-docking) README 방식 그대로: 포장대 앞까지 Nav2 로 간 뒤
precision_docking.launch.py 를 실행하면 바로 도킹을 시작하고, 끝나면 /docking_completed 에 true 를 낸다.
도킹 노드는 켜져 있는 동안 /cmd_vel 을 계속 보내므로 Nav2 주행 중에는 켜지 않고, 도킹이 끝나면 끈다.
OMX 는 정해진 시간(integration.yaml 의 episode_time) 동안 정책을 돌리고 끝난다. 작업 완료를 스스로 판단하지 않는다.
결과는 회차마다 --log-dir 의 CSV 와 Markdown 표에 덧붙인다.
Ctrl+C: Nav2 목표·도킹·OMX 를 모두 취소하고 바퀴에 0 을 보낸 뒤 끝낸다.
"""
import argparse
import csv
import math
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import BackUp, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from PIL import Image
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'amr', 'pinky_navigation', 'map', 'arena_map.pgm')
# map 좌표 (x, y, yaw). 출발 위치는 integration.yaml 의 피킹 구역 pose, home 은 home_pose
STARTS = {'daily': (0.13, 1.45, -3.14), 'food': (1.13, 1.01, -1.57)}
HOME = (0.09, 0.26, 0.0)
PACK = (0.97, 0.23, -0.30)  # 두 OMX 사이 마커를 바라보는 방향
MARKER_ID = 5
# 도킹 뒤 포장대에서 뒤로 빠지는 거리 (m). 너무 짧으면 제자리 회전할 때 OMX 팔에 걸려 home 으로 못 감 (--backup 으로 변경)
# 후진 뒤 home 쪽으로 제자리 회전까지 하고 출발함 (포장대 앞에서 Nav2 가 혼자 경로를 못 만드는 경우가 많아서)
BACKUP_DIST = 0.40
# 팀원 README 의 실제 로봇 실행 명령 그대로 (마커 ID 만 현장 값)
DOCK_CMD = ("bash -ic 'source ~/couping_dock_ws/install/setup.bash && ros2 launch pinky_precision_docking "
            "precision_docking.launch.py target_marker_id:={marker} target_distance:={dist} "
            "distance_tolerance:=0.01 center_tolerance:=0.005'")
# 기록 표의 구간 (이름, 표 제목)
STAGES = [('locate', '위치확인'), ('to_pack', '포장대 주행'), ('dock', '도킹'), ('omx', 'OMX'),
          ('backup', '후진'), ('to_home', 'home 복귀')]


OMX_LOG = os.path.expanduser('~/couping_ws/log_omx/run_act_latest.log')


def omx_log_reason(status):
    """omx_packer 실패 상태에 적힌 로그에서 원인 한 줄을 뽑음 (예: 오른팔 모터 응답 없음)."""
    m = re.search(r'로그: ([^)]+)\)', status)
    if not m or not os.path.exists(m.group(1)):
        return ''
    text = open(m.group(1), errors='replace').read()
    port = re.search(r"motor check failed on port '([^']+)'", text)
    if port:
        missing = re.findall(r'^\s*- (\d+) \(expected', text, re.M)
        return f' · {port.group(1).split("/")[-1]} 모터 응답 없음 (ID {",".join(missing)})'
    errs = re.findall(r'^\w*(?:Error|Exception): .*$', text, re.M)
    return f' · {errs[-1][:120]}' if errs else ''


class StageError(RuntimeError):
    """어느 구간에서 실패했는지 함께 기록."""

    def __init__(self, stage, reason):
        super().__init__(reason)
        self.stage = stage


class Run:
    def __init__(self, args):
        self.args = args
        # Ctrl+C 를 rclpy 가 가로채 통신을 먼저 닫으면 abort() 의 정지 명령이 못 나감 → 직접 처리
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.n = rclpy.create_node('pack_run')
        self.scan = None
        self.amcl = None
        self.docked = False
        self.packer = []
        self.n.create_subscription(Bool, '/docking_completed', lambda m: setattr(self, 'docked', self.docked or m.data),
                                   QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.n.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda p: setattr(self, 'amcl', p), 10)
        self.n.create_subscription(String, '/couping/packer/status', lambda m: self.packer.append(m.data), 10)
        self.cmd = self.n.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pub = self.n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.packer_pub = self.n.create_publisher(String, '/couping/packer/cmd', 10)
        self.nav = ActionClient(self.n, NavigateToPose, '/navigate_to_pose')
        self.backup = ActionClient(self.n, BackUp, '/backup')
        self.spin_ac = ActionClient(self.n, Spin, '/spin')
        self.goal = None
        self.dock_proc = None
        self.dock_lines = []
        self.dock_summary = ''
        self.t0 = time.time()
        im = np.array(Image.open(MAP))
        self.map_h, self.map_w = im.shape
        self.map_dist = cv2.distanceTransform(((im >= 50) * 255).astype(np.uint8), cv2.DIST_L2, 3) * 0.02

    # ---------- 공통 ----------
    def log(self, msg):
        print(f'[{time.time() - self.t0:6.1f}s] {msg}', flush=True)

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def call(self, srv_type, name, req, timeout=10.0):
        c = self.n.create_client(srv_type, name)
        if not c.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{name} 서비스 없음')
        f = c.call_async(req)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=timeout)
        return f.result()

    def stop_wheels(self):
        for _ in range(10):
            self.cmd.publish(Twist())
            time.sleep(0.03)

    def clear_costmaps(self):
        for name in ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap'):
            self.call(ClearEntireCostmap, name, ClearEntireCostmap.Request())

    def wait_action(self, client, goal, stage, timeout, what):
        """액션 목표를 보내고 결과까지 기다림. 실패하면 StageError."""
        if not client.wait_for_server(timeout_sec=10):
            raise StageError(stage, f'{what} 액션 서버 없음')
        f = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, f, timeout_sec=10)
        if f.result() is None or not f.result().accepted:
            raise StageError(stage, f'{what}: Nav2 가 목표를 받지 않음')
        self.goal = f.result()
        res = self.goal.get_result_async()
        t = time.time()
        while not res.done():
            rclpy.spin_once(self.n, timeout_sec=0.1)
            if time.time() - t > timeout:
                raise StageError(stage, f'{what}: {timeout:.0f}초 초과')
        self.goal = None
        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            code = getattr(res.result().result, 'error_code', '')
            raise StageError(stage, f'{what}: Nav2 실패 (status {res.result().status}{f", error {code}" if code else ""})')

    def drive(self, pose, stage, what, timeout=90):
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.n.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = pose[0], pose[1]
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.wait_action(self.nav, g, stage, timeout, what)

    # ---------- ① 출발 위치 확인 ----------
    def fit(self, x, y, yaw, bx, by):
        """LiDAR 점(로봇 기준)을 (x, y, yaw) 에 놓았을 때 지도 벽과 5cm 안으로 겹치는 비율."""
        res, ox, oy = 0.02, -0.196, -0.377
        mx = x + bx * math.cos(yaw) - by * math.sin(yaw)
        my = y + bx * math.sin(yaw) + by * math.cos(yaw)
        c = ((mx - ox) / res).astype(int)
        rw = (self.map_h - 1 - (my - oy) / res).astype(int)
        ins = (c >= 0) & (c < self.map_w) & (rw >= 0) & (rw < self.map_h)
        d = np.full(len(mx), 1.0)
        d[ins] = self.map_dist[rw[ins], c[ins]]
        return float(np.mean(d < 0.05))

    def locate(self, start):
        """출발 위치 근처에서 LiDAR 가 지도와 가장 잘 맞는 자리를 찾아 Nav2(AMCL) 위치로 맞춤."""
        self.scan = None
        end = time.time() + 5
        while self.scan is None and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if self.scan is None:
            raise StageError('locate', 'LiDAR(/scan) 없음')
        s = self.scan
        r = np.array(s.ranges)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > 0.18) & (r < 3.0)
        a, r = a[ok], r[ok]
        bx = -0.017 + r * np.cos(a + math.pi)  # 핑키 LiDAR 는 180도 돌아 있음
        by = r * np.sin(a + math.pi)
        x0, y0, t0 = start
        best = max((self.fit(x, y, t, bx, by), x, y, t)
                   for x in np.arange(x0 - 0.10, x0 + 0.101, 0.02)
                   for y in np.arange(y0 - 0.10, y0 + 0.101, 0.02)
                   for t in np.arange(t0 - 0.15, t0 + 0.151, 0.03))
        fit, x, y, t = best
        self.log(f'출발 위치 확인: ({x:.2f}, {y:.2f}, {t:.2f}) 에서 LiDAR 점의 {fit*100:.0f}% 가 지도 벽과 일치')
        if fit < 0.85:
            raise StageError('locate', f'출발 위치가 아님 (지도 일치 {fit*100:.0f}% < 85%)')
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = float(x), float(y)
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(t / 2), math.cos(t / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2
        m.pose.covariance[35] = math.radians(3) ** 2
        end = time.time() + 10  # AMCL 과 연결된 뒤에 보내야 메시지가 사라지지 않음
        while self.init_pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if self.init_pub.get_subscription_count() == 0:
            raise StageError('locate', 'AMCL(/initialpose) 과 연결 안 됨')
        for _ in range(5):  # /amcl_pose 가 그 자리(10cm 안)로 바뀌었는지 확인, 아니면 다시 보냄
            self.amcl = None
            m.header.stamp = self.n.get_clock().now().to_msg()
            self.init_pub.publish(m)
            self.spin(1.5)
            if self.amcl is not None and math.hypot(self.amcl.pose.pose.position.x - x,
                                                    self.amcl.pose.pose.position.y - y) < 0.1:
                break
        else:
            raise StageError('locate', 'Nav2 위치를 출발 위치로 맞추지 못함')
        self.clear_costmaps()

    def use_manual_pose(self):
        """--manual-start: 사람이 RViz '2D Pose Estimate' 로 찍은 위치를 그대로 쓰고 장애물 지도만 비움."""
        self.spin(0.5)
        if self.amcl is not None:
            p = self.amcl.pose.pose
            yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
            self.log(f'RViz 로 찍은 위치 사용: ({p.position.x:.2f}, {p.position.y:.2f}, {yaw:.2f})')
        else:
            self.log('RViz 로 찍은 위치 사용 (/amcl_pose 는 아직 안 들어옴)')
        self.clear_costmaps()

    # ---------- ③ 도킹 ----------
    def _read_dock(self):
        for line in self.dock_proc.stdout:
            self.dock_lines.append(line)

    def dock(self):
        self.docked = False
        self.dock_lines = []
        self.dock_proc = subprocess.Popen(
            ['ssh', '-tt', '-o', 'BatchMode=yes', f'pinky@{self.args.pinky}',
             DOCK_CMD.format(marker=MARKER_ID, dist=self.args.dock_distance)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True, errors='replace')
        threading.Thread(target=self._read_dock, daemon=True).start()
        self.log(f'도킹 노드 실행 (마커 ID {MARKER_ID}, 목표 {self.args.dock_distance*100:.0f}cm)')
        t = time.time()
        while not self.docked:
            rclpy.spin_once(self.n, timeout_sec=0.05)
            text = ''.join(self.dock_lines)
            if '-> FAILED' in text or 'Phase] FAILED' in text:
                raise StageError('dock', '도킹 노드가 FAILED 보고')
            if self.dock_proc.poll() is not None and not self.docked:
                raise StageError('dock', '도킹 노드가 완료 전에 종료됨 (SSH·빌드 확인)')
            if time.time() - t > 120:
                raise StageError('dock', '120초 초과')
        time.sleep(1.0)
        keys = ('Final Docked', 'Final Lateral', 'Final Angular', 'Retries')
        lines = [re.sub(r'^\[[^\]]*\]\s*-?\s*', '', l.split(']: ')[-1].replace('\x1b[0m', '').strip())
                 for l in self.dock_lines if any(k in l for k in keys)]
        self.dock_summary = ' / '.join(lines)
        for s in lines:
            print('          ' + s, flush=True)
        self.stop_dock_node()

    def stop_dock_node(self):
        if self.dock_proc and self.dock_proc.poll() is None:
            self.dock_proc.terminate()
            try:
                self.dock_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.dock_proc.kill()
        self.dock_proc = None

    # ---------- ④ OMX ----------
    def omx(self):
        end = time.time() + 10
        while self.packer_pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if self.packer_pub.get_subscription_count() == 0:
            raise StageError('omx', 'omx_packer 노드 없음')
        self.packer.clear()
        self.packer_pub.publish(String(data='start'))
        self.log('OMX 포장 시작 (그리퍼 종료 판정 또는 최대 시간까지 정책 실행)')
        t = time.time()
        pos = 0  # OMX 로그에서 [couping] 줄(종료 조건·판정)을 그대로 보여줌
        while not any(s == 'done' or s.startswith('fail') for s in self.packer):
            rclpy.spin_once(self.n, timeout_sec=0.2)
            if os.path.exists(OMX_LOG) and os.path.getmtime(OMX_LOG) >= t:
                with open(OMX_LOG, errors='replace') as f:
                    if os.path.getsize(OMX_LOG) < pos:
                        pos = 0
                    f.seek(pos)
                    for line in f.read().splitlines():
                        if '[couping]' in line:
                            self.log('OMX ' + line[line.index('[couping]') + 10:].strip())
                    pos = f.tell()
            if time.time() - t > 400:  # CPU 실행은 끝난 뒤 영상 인코딩까지 몇 분 걸림
                raise StageError('omx', '400초 초과')
        last = [s for s in self.packer if s == 'done' or s.startswith('fail')][-1]
        if last != 'done':
            raise StageError('omx', f'omx_packer {last.split(" (로그")[0]}{omx_log_reason(last)}')

    # ---------- ⑤ 후진 ----------
    def back_up(self):
        self.reverse(self.args.backup)
        # 회전이 막히면 (Nav2 '앞에 충돌') 10cm 더 빠져서 한 번 더, 그래도 안 되면 회전 없이 home 주행으로 넘어감
        try:
            self.face_home()
        except StageError as e:
            self.log(f'{e} → 10cm 더 후진해서 다시 회전')
            try:
                self.reverse(0.10)
                self.face_home()
            except StageError as e2:
                self.log(f'{e2} → 회전 없이 home 주행 시도')

    def reverse(self, dist):
        g = BackUp.Goal()
        g.target.x = dist
        g.speed = 0.06
        allow = int(dist / 0.06) + 10
        g.time_allowance = Duration(sec=allow)
        self.wait_action(self.backup, g, 'backup', allow + 5, f'후진 {dist*100:.0f}cm')

    def face_home(self):
        """후진한 자리에서 home 쪽으로 제자리 회전. Nav2 가 회전 공간이 없다고 경로를 못 만드는 걸 막음."""
        self.spin(1.0)  # 후진 뒤 /amcl_pose 갱신 대기
        if self.amcl is None:
            self.log('home 방향 회전 생략 (/amcl_pose 없음)')
            return
        p = self.amcl.pose.pose
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        want = math.atan2(HOME[1] - p.position.y, HOME[0] - p.position.x)
        turn = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
        if abs(turn) < math.radians(15):
            return
        g = Spin.Goal()
        g.target_yaw = turn
        g.time_allowance = Duration(sec=15)
        self.wait_action(self.spin_ac, g, 'backup', 20, f'home 쪽으로 {math.degrees(turn):+.0f}° 회전')

    # ---------- 정리 ----------
    def abort(self):
        self.log('멈춤: Nav2 목표·도킹·OMX 취소, 바퀴 정지')
        try:
            if self.goal is not None:
                rclpy.spin_until_future_complete(self.n, self.goal.cancel_goal_async(), timeout_sec=3)
        except Exception:
            pass
        self.goal = None
        try:
            if self.dock_proc is not None:
                self.call(Trigger, '/docking/cancel', Trigger.Request(), timeout=3)
        except Exception:
            pass
        self.packer_pub.publish(String(data='stop'))
        self.stop_wheels()
        self.stop_dock_node()

    # ---------- 한 회차 ----------
    def one_run(self):
        """한 회차 실행. 반환: (성공 여부, 실패 구간, 이유, 구간별 초)."""
        start = STARTS[self.args.start]
        times = {}
        locate = (lambda: self.use_manual_pose()) if self.args.manual_start else (lambda: self.locate(start))
        steps = [('locate', locate),
                 ('to_pack', lambda: self.drive(PACK, 'to_pack', '포장대 주행')),
                 ('dock', self.dock),
                 ('omx', self.omx),
                 ('backup', self.back_up),
                 ('to_home', lambda: (self.clear_costmaps(), self.drive(HOME, 'to_home', 'home 복귀')))]
        names = dict(STAGES)
        for key, fn in steps:
            t = time.time()
            self.log(f'── {names[key]} 시작')
            try:
                fn()
            except StageError as e:
                times[key] = time.time() - t
                return False, names[e.stage], str(e), times
            except Exception as e:  # noqa: BLE001 (서비스 없음 등)
                times[key] = time.time() - t
                return False, names[key], str(e), times
            times[key] = time.time() - t
            self.log(f'── {names[key]} 완료 ({times[key]:.1f}초)')
        return True, '', '', times

    def go_to_start(self):
        """회차 사이 준비: 지금 위치에서 출발 위치로 보냄 (기록 안 함). 지금 위치는 Nav2 가 알고 있어야 함."""
        self.clear_costmaps()
        self.drive(STARTS[self.args.start], 'locate', '출발 위치로 이동', timeout=120)


def write_logs(log_dir, stamp, rows):
    os.makedirs(log_dir, exist_ok=True)
    head = ['회차', '시작 시각', '결과', '실패 구간', '실패 이유'] + [f'{t}(초)' for _, t in STAGES] + ['전체(초)', '도킹 결과']
    path_csv = os.path.join(log_dir, f'pack_run_{stamp}.csv')
    with open(path_csv, 'w', newline='', encoding='utf-8-sig') as f:
        w = csv.writer(f)
        w.writerow(head)
        w.writerows(rows)
    path_md = os.path.join(log_dir, f'pack_run_{stamp}.md')
    ok = sum(1 for r in rows if r[2] == '성공')
    with open(path_md, 'w', encoding='utf-8') as f:
        f.write(f'# 단위 연동 시험 (진열대 앞 → 포장대 → 도킹 → OMX → home) {stamp}\n\n')
        f.write(f'성공 {ok} / {len(rows)}회\n\n')
        f.write('| ' + ' | '.join(head) + ' |\n|' + '---|' * len(head) + '\n')
        for r in rows:
            f.write('| ' + ' | '.join(str(c).replace('|', '/') for c in r) + ' |\n')
    return path_csv, path_md


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', choices=sorted(STARTS), default='daily', help='출발할 진열대 (daily: 생활용품, food: 식품)')
    ap.add_argument('--runs', type=int, default=10)
    ap.add_argument('--dock-distance', type=float, default=0.10)
    ap.add_argument('--backup', type=float, default=BACKUP_DIST, help='OMX 뒤 후진 거리 (m)')
    ap.add_argument('--pinky', default='192.168.111.102')
    ap.add_argument('--log-dir', default=os.path.expanduser('~/couping_test_logs'))
    ap.add_argument('--manual-start', action='store_true',
                    help='출발 위치를 LiDAR 로 자동으로 맞추지 않고, RViz 2D Pose Estimate 로 직접 찍은 위치를 씀')
    ap.add_argument('--resume', help='이어서 기록할 CSV (예: ~/couping_test_logs/pack_run_1023_1558.csv). 회차 번호도 이어짐')
    args = ap.parse_args()
    stamp = datetime.now().strftime('%m%d_%H%M')
    rows = []
    if args.resume:
        path = os.path.expanduser(args.resume)
        with open(path, encoding='utf-8-sig') as f:
            rows = list(csv.reader(f))[1:]
        stamp = os.path.basename(path)[len('pack_run_'):-len('.csv')]
        args.log_dir = os.path.dirname(path)
        print(f'{path} 에 이어서 기록 ({len(rows)}회 완료됨)')
    run = Run(args)
    try:
        for k in range(len(rows) + 1, args.runs + 1):
            where = ('RViz 에서 2D Pose Estimate 로 핑키 위치를 찍은 뒤' if args.manual_start
                     else f'핑키를 {args.start} 진열대 앞에 놓은 뒤')
            ans = input(f'\n[{k}/{args.runs}회차] 상자 뚜껑을 열어 두고, {where} Enter '
                        f'(a: 지금 위치에서 자동으로 보내기, q: 끝내기) > ').strip().lower()
            if ans == 'q':
                break
            if ans == 'a':
                try:
                    run.go_to_start()
                except Exception as e:  # noqa: BLE001
                    run.log(f'출발 위치로 이동 실패: {e} → 직접 놓아 주세요')
                    run.abort()
                    input('핑키를 출발 위치에 놓고 Enter > ')
            began = datetime.now().strftime('%H:%M:%S')
            run.t0 = time.time()
            run.dock_summary = ''
            ok, stage, reason, times = run.one_run()
            if not ok:
                run.log(f'실패: [{stage}] {reason}')
                run.abort()
            total = sum(times.values())
            rows.append([k, began, '성공' if ok else '실패', stage, reason]
                        + [f'{times[s]:.1f}' if s in times else '' for s, _ in STAGES]
                        + [f'{total:.1f}', run.dock_summary])
            path_csv, path_md = write_logs(args.log_dir, stamp, rows)
            print(f'\n{k}회차 {"성공" if ok else "실패"} · 전체 {total:.1f}초 → 기록: {path_md}', flush=True)
    except KeyboardInterrupt:
        print('\nCtrl+C', flush=True)
        run.abort()
    finally:
        run.n.destroy_node()
        rclpy.try_shutdown()
    if rows:
        ok = sum(1 for r in rows if r[2] == '성공')
        print(f'\n끝: 성공 {ok} / {len(rows)}회. 기록 {os.path.join(args.log_dir, f"pack_run_{stamp}.md")}')


if __name__ == '__main__':
    main()
