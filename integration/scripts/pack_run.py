#!/usr/bin/env python3
"""home → 포장대 → 도킹 → OMX 를 한 번에 실행 (현장 시험용).

  python3 integration/scripts/pack_run.py --dock-distance 0.10

전제: 핑키 bringup·Nav2(도메인 51), 카메라 노드, 노트북의 omx_packer 가 켜져 있고 핑키는 home 에 있다.
도킹은 팀원 v5.0(feature/amr-docking) README 방식 그대로: 마커 약 30cm 앞까지 Nav2 로 간 뒤
precision_docking.launch.py 를 실행하면 바로 도킹을 시작하고, 끝나면 /docking_completed 에 true 를 낸다.
도킹 노드는 켜져 있는 동안 /cmd_vel 을 계속 보내므로 Nav2 주행 중에는 켜지 않고, 도킹이 끝나면 끈다.
Ctrl+C: Nav2 목표·도킹·OMX 를 모두 취소하고 바퀴에 0 을 보낸다.

10/22 오전 흐름(run_arena_mission --step 7 --no-home 으로 선반 피킹) 뒤에 이어서 쓰는 방법:
  python3 integration/scripts/pack_run.py --after-pick --dock-node laptop
  --after-pick       : home 확인을 건너뛰고 선반에서 언도킹한 자리에서 바로 포장대로 출발
  --from-dock        : 핑키를 포장대 앞에 직접 두고 도킹부터 시작 (home 확인·포장대 주행 생략)
  --dock-node laptop : 노트북에 이미 켜 둔 도킹 노드(main v6.0 precision_docking)를
                       /docking/target_marker_id + /docking/start 로 시작 (핑키에 v5 를 따로 띄우지 않음)
OMX 가 끝나면 포장대에서 후진(--undock, 기본 0.35m) 뒤 home(P) 으로 돌아온다 (--no-return 이면 생략).
"""
import argparse
import math
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from PIL import Image
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger

PINKY = 'pinky@192.168.111.102'
MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'amr', 'pinky_navigation', 'map', 'arena_map.pgm')
HOME = (0.10, 0.24, 0.03)
PACK = (0.771, 0.2436, 0.0467)  # 두 OMX 사이 마커 5 를 바라보는 방향. 10/27 핑키 인수인계 실측값 (이전 0.97, 0.23, -0.30)
MARKER_ID = 5
# 팀원 README 의 실제 로봇 실행 명령 그대로 (마커 ID 만 현장 값)
DOCK_CMD = ("bash -ic 'source ~/couping_dock_ws/install/setup.bash && ros2 launch pinky_precision_docking "
            "precision_docking.launch.py target_marker_id:={marker} target_distance:={dist} "
            "distance_tolerance:=0.01 center_tolerance:=0.005'")


class Run:
    def __init__(self, dock_distance, dock_node='pinky'):
        self.dock_distance = dock_distance
        self.dock_node = dock_node
        rclpy.init()
        self.n = rclpy.create_node('pack_run')
        self.scan = None
        self.docked = False
        self.packer = []
        self.n.create_subscription(Bool, '/docking_completed', lambda m: setattr(self, 'docked', self.docked or m.data),
                                   QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.n.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.n.create_subscription(String, '/couping/packer/status', lambda m: self.packer.append(m.data), 10)
        self.cmd = self.n.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pub = self.n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.packer_pub = self.n.create_publisher(String, '/couping/packer/cmd', 10)
        self.dock_id_pub = self.n.create_publisher(Int32, '/docking/target_marker_id', 10)
        self.odom = None
        self.n.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.nav = ActionClient(self.n, NavigateToPose, '/navigate_to_pose')
        self.nav_goal = None
        self.dock_proc = None
        self.dock_lines = []
        self.t0 = time.time()

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

    # ---------- 1. home 확인 ----------
    def check_home(self):
        im = np.array(Image.open(MAP)); h, w = im.shape; res, ox, oy = 0.02, -0.196, -0.377
        dist = cv2.distanceTransform(((im >= 50) * 255).astype(np.uint8), cv2.DIST_L2, 3) * res
        end = time.time() + 5
        while self.scan is None and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if self.scan is None:
            raise RuntimeError('LiDAR(/scan) 없음')
        s = self.scan; r = np.array(s.ranges); a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > 0.18) & (r < 3.0); a, r = a[ok], r[ok]
        bx = -0.017 + r * np.cos(a + math.pi); by = r * np.sin(a + math.pi)  # 핑키 LiDAR 는 180도 돌아 있음
        x, y, yaw = HOME
        mx = x + bx * math.cos(yaw) - by * math.sin(yaw); my = y + bx * math.sin(yaw) + by * math.cos(yaw)
        c = ((mx - ox) / res).astype(int); rw = (h - 1 - (my - oy) / res).astype(int)
        ins = (c >= 0) & (c < w) & (rw >= 0) & (rw < h); d = np.full(len(mx), 1.0); d[ins] = dist[rw[ins], c[ins]]
        fit = float(np.mean(d < 0.05))
        self.log(f'home 확인: LiDAR 점의 {fit*100:.0f}% 가 맵 벽과 일치')
        if fit < 0.85:
            raise RuntimeError('핑키가 home 에 있지 않은 것 같아요 (85% 미만). 바닥 표시 자리에 놓아 주세요')
        m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
        m.pose.pose.position.x, m.pose.pose.position.y = HOME[0], HOME[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(HOME[2] / 2), math.cos(HOME[2] / 2)
        m.pose.covariance[0] = m.pose.covariance[7] = 0.05 ** 2; m.pose.covariance[35] = math.radians(3) ** 2
        end = time.time() + 10  # AMCL 과 연결된 뒤에 보내야 메시지가 사라지지 않음
        while self.init_pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if self.init_pub.get_subscription_count() == 0:
            raise RuntimeError('AMCL(/initialpose) 과 연결 안 됨')
        self.spin(0.5)
        self.amcl = None
        self.n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda p: setattr(self, 'amcl', p), 10)
        for _ in range(5):  # 받았는지 /amcl_pose 로 확인, 안 됐으면 다시 보냄
            m.header.stamp = self.n.get_clock().now().to_msg()
            self.init_pub.publish(m); self.spin(1.5)
            if self.amcl is not None and math.hypot(self.amcl.pose.pose.position.x - HOME[0], self.amcl.pose.pose.position.y - HOME[1]) < 0.1:
                break
        else:
            raise RuntimeError('Nav2 위치를 home 으로 맞추지 못함')
        p = self.amcl.pose.pose.position
        self.log(f'Nav2 위치 확인: ({p.x:.2f}, {p.y:.2f})')
        for name in ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap'):
            self.call(ClearEntireCostmap, name, ClearEntireCostmap.Request())
        self.log('Nav2 위치를 home 으로 맞추고 장애물 지도를 비움')

    # ---------- 2. 포장대 주행 ----------
    def drive_to_pack(self):
        self.drive(PACK, '포장대')

    def clear_costmaps(self):
        for name in ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap'):
            self.call(ClearEntireCostmap, name, ClearEntireCostmap.Request())
        self.log('장애물 지도를 비움 (선반에서 이어서 출발)')

    def drive(self, pose, name):
        if not self.nav.wait_for_server(timeout_sec=10):
            raise RuntimeError('Nav2 액션 서버 없음')
        g = NavigateToPose.Goal(); g.pose.header.frame_id = 'map'; g.pose.header.stamp = self.n.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = pose[0], pose[1]
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        f = self.nav.send_goal_async(g); rclpy.spin_until_future_complete(self.n, f, timeout_sec=10)
        if f.result() is None or not f.result().accepted:
            raise RuntimeError('Nav2 가 목표를 받지 않음')
        self.nav_goal = f.result(); t = time.time(); res = self.nav_goal.get_result_async()
        self.log(f'{name}(으)로 출발')
        while not res.done():
            rclpy.spin_once(self.n, timeout_sec=0.1)
            if time.time() - t > 90:
                raise RuntimeError(f'{name} 주행 90초 초과')
        self.nav_goal = None
        if res.result().status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f'{name} 주행 실패 (status {res.result().status})')
        self.log(f'{name} 도착 ({time.time() - t:.1f}초)')

    # ---------- 3. 도킹 ----------
    def _read_dock(self):
        for line in self.dock_proc.stdout:
            self.dock_lines.append(line)

    def dock(self):
        if self.dock_node == 'laptop':
            return self.dock_laptop()
        t = time.time()
        self.docked = False
        self.dock_proc = subprocess.Popen(
            ['ssh', '-tt', '-o', 'BatchMode=yes', PINKY, DOCK_CMD.format(marker=MARKER_ID, dist=self.dock_distance)],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True, errors='replace')
        threading.Thread(target=self._read_dock, daemon=True).start()
        self.log(f'도킹 노드 실행 (v5.0 launch, 마커 ID {MARKER_ID}, 목표 {self.dock_distance*100:.0f}cm) → 켜지면 바로 도킹 시작')
        start_idx = 0
        while True:
            rclpy.spin_once(self.n, timeout_sec=0.05)
            if self.docked:  # 완료 신호: /docking_completed == true
                break
            text = ''.join(self.dock_lines[start_idx:])
            if '-> FAILED' in text or 'Phase] FAILED' in text:
                raise RuntimeError('도킹 실패 (도킹 노드가 FAILED 보고)')
            if time.time() - t > 120:
                raise RuntimeError('도킹 120초 초과')
        time.sleep(1.0)
        summary = [l.strip() for l in self.dock_lines[start_idx:] if any(k in l for k in ('Final Docked', 'Final Lateral', 'Final Angular', 'Retries'))]
        self.log(f'도킹 완료: /docking_completed = true ({time.time() - t:.1f}초)')
        for s in summary:
            print('          ' + s.split(']: ')[-1].replace('\x1b[0m', ''), flush=True)
        self.stop_dock_node()

    def dock_laptop(self):
        """노트북에 켜 둔 도킹 노드(v6.0)에 마커를 지정하고 시작 (팀원 run_arena_mission 과 같은 방식)."""
        t = time.time()
        self.spin(1.0)  # run_arena_mission 처럼 Nav2 도착 뒤 1초 대기
        for _ in range(3):
            self.dock_id_pub.publish(Int32(data=MARKER_ID))
            self.spin(0.01)
        self.docked = False
        self.dock_proc = None
        r = self.call(Trigger, '/docking/start', Trigger.Request())
        self.docking_laptop = True
        self.log(f'도킹 시작 (노트북 v6.0 노드, 마커 ID {MARKER_ID}): {r.message if r else "응답 없음"}')
        while not self.docked:
            rclpy.spin_once(self.n, timeout_sec=0.05)
            if time.time() - t > 120:
                raise RuntimeError('도킹 120초 초과 (마커를 못 찾았으면 도킹 노드 로그의 FAILED 확인)')
        self.docking_laptop = False
        self.log(f'도킹 완료: /docking_completed = true ({time.time() - t:.1f}초)')

    def stop_dock_node(self):
        if self.dock_proc and self.dock_proc.poll() is None:
            self.dock_proc.terminate()
            try:
                self.dock_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.dock_proc.kill()
        self.dock_proc = None

    # ---------- 4. OMX ----------
    def omx(self):
        t = time.time(); end = t + 10
        while self.packer_pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.1)
        if self.packer_pub.get_subscription_count() == 0:
            raise RuntimeError('OMX 포장 노드(omx_packer) 없음')
        self.packer.clear(); self.packer_pub.publish(String(data='start')); self.log('OMX 포장 시작')
        while not any(s == 'done' or s.startswith('fail') for s in self.packer):
            rclpy.spin_once(self.n, timeout_sec=0.2)
            if time.time() - t > 400:  # CPU 실행은 끝난 뒤 영상 인코딩까지 몇 분 걸림
                raise RuntimeError('OMX 400초 초과')
        self.log(f'OMX {self.packer[-1]} ({time.time() - t:.1f}초)')
        if self.packer[-1] != 'done':
            raise RuntimeError('OMX 실패')

    # ---------- 5. 후진 · home(P) 복귀 ----------
    def undock(self, distance, speed=0.05):
        """odom 기준으로 곧게 후진 (run_arena_mission 의 undock_backup 과 같은 방식)."""
        self.odom = None
        end = time.time() + 3
        while self.odom is None and time.time() < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)
        if self.odom is None:
            raise RuntimeError('/odom 없음')
        p0 = self.odom.pose.pose.position
        back = Twist(); back.linear.x = -abs(speed)
        t = time.time()
        self.log(f'포장대에서 후진 {distance*100:.0f}cm')
        while True:
            rclpy.spin_once(self.n, timeout_sec=0.02)
            p = self.odom.pose.pose.position
            if math.hypot(p.x - p0.x, p.y - p0.y) >= distance or time.time() - t > 30:
                break
            self.cmd.publish(back)
        self.stop_wheels()

    def return_home(self, undock_distance):
        self.undock(undock_distance)
        self.drive(HOME, 'home(P)')

    # ---------- 정리 ----------
    def abort(self):
        self.log('멈춤: Nav2 목표·도킹·OMX 취소, 바퀴 정지')
        try:
            if self.nav_goal is not None:
                rclpy.spin_until_future_complete(self.n, self.nav_goal.cancel_goal_async(), timeout_sec=3)
        except Exception:
            pass
        try:
            if self.dock_proc is not None or getattr(self, 'docking_laptop', False):
                self.call(Trigger, '/docking/cancel', Trigger.Request(), timeout=3)
        except Exception:
            pass
        self.packer_pub.publish(String(data='stop'))
        self.stop_wheels()
        self.stop_dock_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dock-distance', type=float, default=0.10)
    ap.add_argument('--after-pick', action='store_true', help='선반 피킹(run_arena_mission) 뒤 이어서: home 확인 생략')
    ap.add_argument('--from-dock', action='store_true', help='포장대 앞에 둔 상태에서 도킹부터 시작')
    ap.add_argument('--dock-node', choices=('pinky', 'laptop'), default='pinky',
                    help='pinky: 핑키에서 v5 launch 실행(원래 방식) / laptop: 노트북에 켜 둔 v6.0 노드 사용')
    ap.add_argument('--undock', type=float, default=0.35, help='OMX 뒤 포장대에서 후진 거리 m (팀원 미션 packaging 값)')
    ap.add_argument('--no-return', action='store_true', help='OMX 뒤 home(P) 복귀 생략')
    args = ap.parse_args()
    run = Run(args.dock_distance, args.dock_node)
    ok = False
    try:
        if not args.from_dock:
            if args.after_pick:
                run.clear_costmaps()
            else:
                run.check_home()
            run.drive_to_pack()
        run.dock(); run.omx()
        if not args.no_return:
            run.return_home(args.undock)
        ok = True
        run.log('전체 완료')
    except KeyboardInterrupt:
        print('\nCtrl+C', flush=True)
    except Exception as e:  # noqa: BLE001
        run.log(f'중단: {e}')
    finally:
        if not ok:
            run.abort()
        run.n.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
