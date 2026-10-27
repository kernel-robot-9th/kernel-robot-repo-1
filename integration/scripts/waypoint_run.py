"""Nav2 waypoint driving only (10/27). Docking / Doosan pick / OMX are mock states (no command sent, no undock).

  python3 integration/scripts/waypoint_run.py                 # P -> shelf1_m7 -> shelf2 -> packaging -> home
  python3 integration/scripts/waypoint_run.py --route shelf1_m8,shelf2,packaging,home --mock-sec 2
Waypoints come from run_arena_mission.WAYPOINTS (same coordinates the mission uses).
For each leg: Nav2 result, drive time, final pose error vs the waypoint (from TF map->base_footprint).
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from pinky_precision_docking.run_arena_mission import WAYPOINTS
from tf2_ros import Buffer, TransformListener

MOCK = {'shelf1': 'docking (mock) -> Doosan pick (mock)', 'shelf2': 'docking (mock) -> Doosan pick (mock)',
        'packaging': 'docking (mock) -> OMX packing (mock)'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--route', default='shelf1_m7,shelf2,packaging,home')
    ap.add_argument('--mock-sec', type=float, default=1.0, help='time spent in each mock state')
    ap.add_argument('--timeout', type=float, default=120.0)
    args = ap.parse_args()

    rclpy.init()
    nav = BasicNavigator()
    tf = Buffer()
    TransformListener(tf, nav, spin_thread=True)

    def pose_now():
        try:
            t = tf.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
            return t.translation.x, t.translation.y, yaw_of(t.rotation)
        except Exception:
            return None

    rows, t_all = [], time.time()
    for name in [s.strip() for s in args.route.split(',') if s.strip()]:
        x, y, yaw = WAYPOINTS[name]
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = nav.get_clock().now().to_msg()
        g.pose.position.x, g.pose.position.y = x, y
        g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        print(f'[{time.time() - t_all:6.1f}s] -> {name} ({x:.3f}, {y:.3f}, {math.degrees(yaw):+.1f} deg)', flush=True)
        t0 = time.time()
        if not nav.goToPose(g):
            print('   goal rejected', flush=True)
            rows.append((name, 'REJECTED', 0.0, None))
            break
        while not nav.isTaskComplete():
            if time.time() - t0 > args.timeout:
                nav.cancelTask()
            time.sleep(0.1)
        res = nav.getResult()
        dt = time.time() - t0
        p = pose_now()
        err = None
        if p:
            err = (math.hypot(p[0] - x, p[1] - y) * 100,
                   math.degrees(math.atan2(math.sin(p[2] - yaw), math.cos(p[2] - yaw))))
        ok = res == TaskResult.SUCCEEDED
        print(f'   {"OK" if ok else "FAIL"} {res.name} in {dt:.1f}s'
              + (f', error {err[0]:.1f} cm / {err[1]:+.1f} deg' if err else ''), flush=True)
        rows.append((name, res.name, dt, err))
        if not ok:
            break
        zone = name.split('_')[0]
        if zone in MOCK:
            print(f'   {MOCK[zone]}: {args.mock_sec:.0f}s', flush=True)
            time.sleep(args.mock_sec)

    print(f'\n=== waypoint test: {time.time() - t_all:.1f}s total ===')
    for name, res, dt, err in rows:
        print(f'  {name:10s} {res:10s} {dt:6.1f}s' + (f'  {err[0]:5.1f} cm {err[1]:+6.1f} deg' if err else ''))
    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
