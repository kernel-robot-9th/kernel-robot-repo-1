#!/usr/bin/python3
"""
mirror_pinky_pose.py : 실제 Pinky 의 위치를 Gazebo 의 가상 Pinky 에 부드럽게 반영한다.

    실제 Pinky --/odom->/real/odom(연속, 부드러움)  --\
               --/amcl_pose(드물게, 정확) --+--(domain_bridge)--> 이 노드 --set_pose--> Gazebo 'pinky'

* 평소 움직임은 /odom 으로, 누적 오차는 /amcl_pose 로 서서히 보정한다 (map->odom 보정을 천천히 따라감).
* /real/odom 이 없으면 /amcl_pose 만 부드럽게(보간) 따라간다.
* map 좌표 -> Gazebo 월드 좌표 변환:
    calibrate_on_first:=true  첫 위치를 (anchor_x, anchor_y, anchor_yaw) 에 맞춘다 (기본)
    calibrate_on_first:=false offset_x/offset_y/offset_yaw 를 직접 사용:
        gazebo = R(offset_yaw) * map + (offset_x, offset_y)
* 실제 -> 가상 한 방향. 실제 쪽에는 아무것도 publish 하지 않는다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import SetEntityPose


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def compose(a, b):  # a ∘ b
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-(c * a[0] + s * a[1]), -(-s * a[0] + c * a[1]), wrap(-a[2]))


def blend(cur, tgt, alpha):
    return (cur[0] + alpha * (tgt[0] - cur[0]),
            cur[1] + alpha * (tgt[1] - cur[1]),
            wrap(cur[2] + alpha * wrap(tgt[2] - cur[2])))


class PinkyPoseMirror(Node):
    def __init__(self):
        super().__init__('mirror_pinky_pose')
        p = self.declare_parameter
        self.world = p('world', 'pinky_factory').value
        self.entity = p('entity', 'pinky').value
        self.z = float(p('z', 0.002).value)
        self.calib = bool(p('calibrate_on_first', True).value)
        self.anchor = (float(p('anchor_x', 0.0).value), float(p('anchor_y', 0.0).value),
                       float(p('anchor_yaw', 0.0).value))
        self.theta = float(p('offset_yaw', 0.0).value)
        self.tx = float(p('offset_x', 0.0).value)
        self.ty = float(p('offset_y', 0.0).value)
        rate = max(float(p('rate_hz', 30.0).value), 1.0)
        self.alpha = float(p('smooth', 0.03).value)  # 틱마다 보정을 따라가는 비율
        self.calibrated = not self.calib
        self.use_odom = bool(p('use_odom', True).value)
        self.debug = bool(p('debug', False).value)
        self.n = 0
        self.last_sent = None

        self.odom = None      # 최신 /odom (x,y,yaw)
        self.t_target = None  # amcl 기준 map->odom 보정
        self.t_cur = None     # 부드럽게 따라가는 map->odom
        self.direct_target = None  # odom 이 없을 때 amcl 직접
        self.direct_cur = None
        self.busy = False

        self.cli = self.create_client(SetEntityPose, f'/world/{self.world}/set_pose')
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, qos)
        self.create_subscription(Odometry, '/real/odom', self.on_odom, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"대기 중: /amcl_pose + /real/odom -> '{self.entity}' "
                               f"(calibrate_on_first={self.calib}, anchor={self.anchor})")

    def on_odom(self, msg):
        pp = msg.pose.pose
        if self.use_odom:
            self.odom = (pp.position.x, pp.position.y, yaw_from_quat(pp.orientation))

    def on_amcl(self, msg):
        pp = msg.pose.pose
        m = (pp.position.x, pp.position.y, yaw_from_quat(pp.orientation))
        self.direct_target = m
        if self.debug:
            self.get_logger().info(f'[amcl] x={m[0]:.2f} y={m[1]:.2f} yaw={m[2]:.2f}')
        if self.odom is not None:
            self.t_target = compose(m, inverse(self.odom))

    def current_map_pose(self):
        if self.odom is not None and self.t_target is not None:
            self.t_cur = self.t_target if self.t_cur is None else blend(self.t_cur, self.t_target, self.alpha)
            return compose(self.t_cur, self.odom)
        if self.direct_target is not None:
            self.direct_cur = (self.direct_target if self.direct_cur is None
                               else blend(self.direct_cur, self.direct_target, self.alpha))
            return self.direct_cur
        return None

    def tick(self):
        mp = self.current_map_pose()
        if mp is None:
            return
        if not self.calibrated:
            ax, ay, ayaw = self.anchor
            self.theta = ayaw - mp[2]
            c, s = math.cos(self.theta), math.sin(self.theta)
            self.tx = ax - (c * mp[0] - s * mp[1])
            self.ty = ay - (s * mp[0] + c * mp[1])
            self.calibrated = True
            self.get_logger().info(
                f'보정 완료: offset_x={self.tx:.3f} offset_y={self.ty:.3f} offset_yaw={self.theta:.3f} '
                '(다음부터 이 값을 직접 넣으면 보정 없이 실행 가능)')
        self.n += 1
        if self.debug and self.n % 15 == 0:
            o = self.odom
            self.get_logger().info(
                f'[map] x={mp[0]:.2f} y={mp[1]:.2f} yaw={mp[2]:.2f} | odom={None if o is None else tuple(round(v, 2) for v in o)} '
                f'| srv_ready={self.cli.service_is_ready()}')
        if self.busy or not self.cli.service_is_ready():
            return
        c, s = math.cos(self.theta), math.sin(self.theta)
        gx = c * mp[0] - s * mp[1] + self.tx
        gy = s * mp[0] + c * mp[1] + self.ty
        gyaw = mp[2] + self.theta
        # 거의 안 움직였으면 다시 놓지 않는다 (같은 자리에 계속 놓으면 물리가 흔들려 떨림/점프로 보임)
        ls = self.last_sent
        if ls is not None and abs(gx - ls[0]) < 0.002 and abs(gy - ls[1]) < 0.002 and abs(wrap(gyaw - ls[2])) < 0.004:
            return
        self.last_sent = (gx, gy, gyaw)
        req = SetEntityPose.Request()
        req.entity.name = self.entity
        req.entity.type = 2  # MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = gx, gy, self.z
        req.pose.orientation.z = math.sin(gyaw / 2.0)
        req.pose.orientation.w = math.cos(gyaw / 2.0)
        self.busy = True
        def done(f):
            self.busy = False
            if self.debug:
                r = f.result()
                self.get_logger().info(f'[set_pose] ok={None if r is None else r.success} -> x={gx:.2f} y={gy:.2f} yaw={gyaw:.2f}', throttle_duration_sec=1.0)
        self.cli.call_async(req).add_done_callback(done)


def main():
    rclpy.init()
    rclpy.spin(PinkyPoseMirror())


if __name__ == '__main__':
    main()
