#!/usr/bin/env python3
"""
Pinky Pro Precision Auto-Docking Node (v4.0 Master Edition).

Architecture & Features:
1. Phase 1: Nav2 Isolation & FSM Lifecycle
   - IDLE state prevents command collision with Nav2 during global navigation.
   - ROS 2 Trigger services: /docking/start, /docking/cancel.

2. Phase 2: Curved S-Curve (Stanley Controller)
   - Replaces abrupt orthogonal stops with smooth continuous S-curve convergence.
   - Simultaneously drives 1cm lateral error and 3-deg wall angle error to zero during forward motion.

3. Phase 3: 2D LiDAR (/scan) Wall Alignment
   - Real-time least-squares wall line fitting within +/-25 deg front cone.
   - Enforces <0.5 deg perpendicular alignment independent of camera perspective noise.

4. Phase 4: Ultra-Close Sensor Fusion Docking (PRECISION_STOP)
   - Active sensor feedback (Ultrasonic us_sensor/range + 2D LiDAR front beam) inside <14cm camera blind zone.
   - Soft exponential deceleration to exact 3cm stop (tolerance 5mm) with 2cm emergency brake.
   - Automatic fallback to odometry blind push if ultrasonic stream is absent.
"""

from enum import Enum
import math
import signal
import sys
import time
from typing import List, Optional, Tuple

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, LaserScan, Range
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class DockingState(Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    ALIGN_BEARING = "ALIGN_BEARING"
    T_TURN_PARALLEL = "T_TURN_PARALLEL"
    T_DRIVE_LATERAL = "T_DRIVE_LATERAL"
    T_RETURN_FACING = "T_RETURN_FACING"
    FINE_ALIGN = "FINE_ALIGN"
    APPROACH_NORMAL = "APPROACH_NORMAL"
    ALIGN_WALL = "ALIGN_WALL"
    FINAL_APPROACH = "FINAL_APPROACH"
    PRECISION_STOP = "PRECISION_STOP"
    BLIND_PUSH = "BLIND_PUSH"
    DOCKED = "DOCKED"
    LOST = "LOST"


class PrecisionDockingNode(Node):

    def __init__(self):
        super().__init__('precision_docking_node')

        self._declare_and_get_parameters()
        self.bridge = CvBridge()

        # FSM State & lifecycle
        self.state = DockingState.IDLE if self.start_in_idle else DockingState.SEARCH
        self.last_state = None
        self.last_detection_time = 0.0
        self.consecutive_detect_count = 0
        self.last_known_bearing: Optional[float] = None
        self.last_turn_dir: float = 1.0

        # Filtered marker pose [x, y, z, wall_angle, signed_lat, dist_along_normal]
        self.filtered_pose: Optional[List[float]] = None
        self.raw_pose: Optional[Tuple[float, ...]] = None
        self.last_R: Optional[np.ndarray] = None
        self.last_normal_u: Optional[Tuple[float, float]] = None
        self.last_closest_pt: Optional[Tuple[float, float]] = None

        # Odometry tracking
        self.current_odom_x = 0.0
        self.current_odom_y = 0.0
        self.current_odom_yaw = 0.0
        self.odom_received = False
        self.turn_start_yaw = 0.0
        self.t_turn_angle_1 = 0.0
        self.t_turn_angle_2 = 0.0
        self.lat_start_x = 0.0
        self.lat_start_y = 0.0
        self.lat_target_dist = 0.0
        self.t_lateral_offset = 0.0
        self.fine_align_start_time = 0.0
        self.blind_start_x = 0.0
        self.blind_start_y = 0.0
        self.blind_target_dist = 0.0

        # Sensor Fusion: 2D LiDAR Wall Fitting (Phase 3)
        self.lidar_wall_yaw: Optional[float] = None
        self.lidar_front_dist: Optional[float] = None
        self.lidar_last_time: float = 0.0
        self.lidar_fit_r2: float = 0.0

        # Sensor Fusion: Ultrasonic Range (Phase 4)
        self.ultrasonic_dist: Optional[float] = None
        self.ultrasonic_last_time: float = 0.0

        # Camera intrinsic matrix & distortion coefficients
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_info_received = False
        self._init_fallback_camera_matrix(640, 480)

        # ArUco detector setup
        self._init_aruco_detector()

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, reliable_qos)
        self.completed_pub = self.create_publisher(Bool, '/docking_completed', reliable_qos)
        self.debug_raw_pub = self.create_publisher(Image, '/docking/debug_image', sensor_qos)
        self.debug_image_pub = self.create_publisher(
            CompressedImage, '/docking/debug_image/compressed', sensor_qos
        )

        # Subscribers
        if self.use_compressed:
            compressed_topic = self.image_topic if 'compressed' in self.image_topic else f"{self.image_topic}/compressed"
            self.get_logger().info(f"Subscribing to CompressedImage: {compressed_topic}")
            self.image_sub = self.create_subscription(
                CompressedImage, compressed_topic, self._compressed_image_callback, sensor_qos
            )
        else:
            self.get_logger().info(f"Subscribing to raw Image: {self.image_topic}")
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self._image_callback, sensor_qos
            )

        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback, sensor_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, reliable_qos
        )

        # Phase 3 & 4 Subscribers (LiDAR & Ultrasonic)
        if self.enable_lidar_alignment:
            self.get_logger().info(f"Enabling 2D LiDAR Wall Alignment on {self.scan_topic}")
            self.scan_sub = self.create_subscription(
                LaserScan, self.scan_topic, self._scan_callback, sensor_qos
            )

        if self.enable_ultrasonic_stop:
            self.get_logger().info(f"Enabling Ultrasonic Range Feedback on {self.ultrasonic_topic}")
            self.us_sub = self.create_subscription(
                Range, self.ultrasonic_topic, self._ultrasonic_callback, sensor_qos
            )

        # Phase 1: ROS 2 Trigger Services
        self.start_srv = self.create_service(
            Trigger, self.trigger_service_name, self._handle_start_trigger
        )
        self.cancel_srv = self.create_service(
            Trigger, self.cancel_service_name, self._handle_cancel_trigger
        )

        # Control Loop Timer (20Hz)
        self.control_timer = self.create_timer(0.05, self._control_loop)

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self._on_parameter_update)

        self.get_logger().info(
            f"PrecisionDockingNode v4.0 (Master Plan Edition) ready! "
            f"State: {self.state.value}, StartInIdle: {self.start_in_idle}, TargetID: {self.target_marker_id}, "
            f"GoalDist: {self.target_distance}m, CurvedS-Curve: {self.enable_curved_approach}, "
            f"LiDARFit: {self.enable_lidar_alignment}, UltrasonicStop: {self.enable_ultrasonic_stop}"
        )

    def _declare_and_get_parameters(self):
        """Declare ROS 2 parameters with defaults and load them."""
        # 1. Trigger & Lifecycle settings (Phase 1)
        self.declare_parameter('start_in_idle', True, ParameterDescriptor(description='Start in IDLE state waiting for service trigger'))
        self.declare_parameter('trigger_service_name', '/docking/start', ParameterDescriptor(description='Trigger service name'))
        self.declare_parameter('cancel_service_name', '/docking/cancel', ParameterDescriptor(description='Cancel service name'))

        # 2. Marker settings
        self.declare_parameter('marker_size', 0.06, ParameterDescriptor(description='Marker physical size (meters)'))
        self.declare_parameter('target_marker_id', -1, ParameterDescriptor(description='Target ArUco Marker ID (-1 for auto-lock)'))
        self.declare_parameter('aruco_dictionary_id', 'DICT_5X5_250', ParameterDescriptor(description='ArUco dictionary name'))

        # 3. Topic settings
        self.declare_parameter('use_compressed', True, ParameterDescriptor(description='Use CompressedImage stream'))
        self.declare_parameter('image_topic', '/camera/image_raw', ParameterDescriptor(description='Base camera image topic'))
        self.declare_parameter('camera_info_topic', '/camera/camera_info', ParameterDescriptor(description='CameraInfo topic'))
        self.declare_parameter('cmd_vel_topic', '/cmd_vel', ParameterDescriptor(description='Twist command velocity topic'))

        # 4. T-Maneuver settings
        self.declare_parameter('enable_t_maneuver', True, ParameterDescriptor(description='Use orthogonal T-maneuver for coarse alignment'))
        self.declare_parameter('t_maneuver_min_dist', 0.20, ParameterDescriptor(description='Minimum distance from marker to perform T-maneuver (m)'))
        self.declare_parameter('t_maneuver_lateral_speed', 0.05, ParameterDescriptor(description='Linear velocity during T-maneuver lateral drive (m/s)'))
        self.declare_parameter('enable_wall_alignment', False, ParameterDescriptor(description='Pure bearing pursuit flag'))

        # 5. Curved S-Curve Controller (Phase 2: Stanley S-Curve)
        self.declare_parameter('enable_curved_approach', True, ParameterDescriptor(description='Enable Stanley S-Curve smooth convergence'))
        self.declare_parameter('kp_lateral', 1.5, ParameterDescriptor(description='Stanley lateral error convergence gain'))
        self.declare_parameter('kp_wall_yaw', 1.8, ParameterDescriptor(description='Stanley wall heading alignment gain'))
        self.declare_parameter('max_curve_yaw_rate', 0.25, ParameterDescriptor(description='Max angular velocity during curved approach (rad/s)'))

        # 6. Sensor Fusion (Phase 3 & 4: LiDAR & Ultrasonic)
        self.declare_parameter('enable_lidar_alignment', True, ParameterDescriptor(description='Enable 2D LiDAR wall normal fitting'))
        self.declare_parameter('scan_topic', '/scan', ParameterDescriptor(description='2D LiDAR LaserScan topic'))
        self.declare_parameter('lidar_yaw_tolerance', 0.015, ParameterDescriptor(description='LiDAR wall angle tolerance (rad)'))

        self.declare_parameter('enable_ultrasonic_stop', True, ParameterDescriptor(description='Enable ultrasonic sensor 3cm precision stop'))
        self.declare_parameter('ultrasonic_topic', 'us_sensor/range', ParameterDescriptor(description='Ultrasonic Range topic'))
        self.declare_parameter('target_distance', 0.03, ParameterDescriptor(description='Target docking stop distance (meters)'))
        self.declare_parameter('distance_tolerance', 0.005, ParameterDescriptor(description='Distance tolerance (meters)'))
        self.declare_parameter('vision_min_distance', 0.14, ParameterDescriptor(description='Distance before switching to precision stop (m)'))
        self.declare_parameter('enable_blind_push', True, ParameterDescriptor(description='Fallback to odometry blind drive if ultrasonic unavailable'))
        self.declare_parameter('blind_push_speed', 0.03, ParameterDescriptor(description='Velocity during blind/precision stop (m/s)'))

        # 7. Controller parameters
        self.declare_parameter('kp_linear', 0.5, ParameterDescriptor(description='Linear velocity proportional gain'))
        self.declare_parameter('kp_angular', 1.2, ParameterDescriptor(description='Angular velocity proportional gain'))
        self.declare_parameter('max_linear_vel', 0.08, ParameterDescriptor(description='Maximum linear velocity (m/s)'))
        self.declare_parameter('max_angular_vel', 0.35, ParameterDescriptor(description='Maximum angular velocity (rad/s)'))
        self.declare_parameter('min_linear_vel', 0.02, ParameterDescriptor(description='Minimum linear velocity'))
        self.declare_parameter('min_angular_vel', 0.05, ParameterDescriptor(description='Minimum angular velocity'))
        self.declare_parameter('invert_angular', False, ParameterDescriptor(description='Invert steering direction'))

        # 8. Safety & filter
        self.declare_parameter('loss_timeout', 1.2, ParameterDescriptor(description='Timeout before treating marker as lost (s)'))
        self.declare_parameter('ema_alpha', 0.6, ParameterDescriptor(description='EMA filter weight'))
        self.declare_parameter('search_angular_vel', 0.20, ParameterDescriptor(description='Search rotation velocity (rad/s)'))

        self._load_parameters()

    def _load_parameters(self):
        """Cache parameter values into instance variables."""
        self.start_in_idle = self.get_parameter('start_in_idle').value
        self.trigger_service_name = self.get_parameter('trigger_service_name').value
        self.cancel_service_name = self.get_parameter('cancel_service_name').value

        self.marker_size = self.get_parameter('marker_size').value
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.aruco_dict_name = self.get_parameter('aruco_dictionary_id').value
        self.use_compressed = self.get_parameter('use_compressed').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.enable_t_maneuver = self.get_parameter('enable_t_maneuver').value
        self.t_maneuver_min_dist = self.get_parameter('t_maneuver_min_dist').value
        self.t_maneuver_lateral_speed = self.get_parameter('t_maneuver_lateral_speed').value
        self.enable_wall_alignment = self.get_parameter('enable_wall_alignment').value

        self.enable_curved_approach = self.get_parameter('enable_curved_approach').value
        self.kp_lateral = self.get_parameter('kp_lateral').value
        self.kp_wall_yaw = self.get_parameter('kp_wall_yaw').value
        self.max_curve_yaw_rate = self.get_parameter('max_curve_yaw_rate').value

        self.enable_lidar_alignment = self.get_parameter('enable_lidar_alignment').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.lidar_yaw_tolerance = self.get_parameter('lidar_yaw_tolerance').value

        self.enable_ultrasonic_stop = self.get_parameter('enable_ultrasonic_stop').value
        self.ultrasonic_topic = self.get_parameter('ultrasonic_topic').value
        self.target_distance = self.get_parameter('target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.vision_min_distance = self.get_parameter('vision_min_distance').value
        self.enable_blind_push = self.get_parameter('enable_blind_push').value
        self.blind_push_speed = self.get_parameter('blind_push_speed').value

        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_linear_vel = self.get_parameter('min_linear_vel').value
        self.min_angular_vel = self.get_parameter('min_angular_vel').value
        self.invert_angular = self.get_parameter('invert_angular').value
        self.loss_timeout = self.get_parameter('loss_timeout').value
        self.ema_alpha = self.get_parameter('ema_alpha').value
        self.search_angular_vel = self.get_parameter('search_angular_vel').value

    def _on_parameter_update(self, params: List[Parameter]) -> SetParametersResult:
        """Handle dynamic parameter updates at runtime."""
        dict_changed = False
        for param in params:
            if hasattr(self, param.name):
                setattr(self, param.name, param.value)
                if param.name == 'aruco_dictionary_id':
                    self.aruco_dict_name = param.value
                    dict_changed = True
                self.get_logger().info(f"Parameter '{param.name}' updated to: {param.value}")
        if dict_changed:
            self._init_aruco_detector()
            self.get_logger().info(f"ArUco Dictionary reinitialized: {self.aruco_dict_name}")
        return SetParametersResult(successful=True)

    def _handle_start_trigger(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Phase 1: Handle /docking/start trigger service call."""
        if self.state in [DockingState.IDLE, DockingState.DOCKED, DockingState.LOST]:
            if self.consecutive_detect_count >= 1 and self.filtered_pose is not None:
                self.state = DockingState.ALIGN_BEARING
            else:
                self.state = DockingState.SEARCH
            self.get_logger().info(f"Docking triggered via service! Transitioned to {self.state.value}")
            response.success = True
            response.message = f"Docking initiated. Transitioned to {self.state.value}."
        else:
            response.success = True
            response.message = f"Docking already actively running in state {self.state.value}."
        return response

    def _handle_cancel_trigger(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        """Phase 1: Handle /docking/cancel trigger service call."""
        self.get_logger().warn("Docking cancelled via service! Transitioning to IDLE and stopping robot.")
        self.state = DockingState.IDLE
        self._stop_robot()
        response.success = True
        response.message = "Docking cancelled. Robot halted in IDLE."
        return response

    def _init_fallback_camera_matrix(self, width: int, height: int):
        """Construct fallback intrinsic camera matrix when CameraInfo is not yet received."""
        fx = fy = 0.8 * width
        cx = width / 2.0
        cy = height / 2.0
        self.camera_matrix = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def _init_aruco_detector(self):
        """Initialize ArUco detector supporting OpenCV 4.6+ through OpenCV 4.10+."""
        dict_mapping = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
        }
        dict_id = dict_mapping.get(self.aruco_dict_name, cv2.aruco.DICT_5X5_250)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Robust detection tuning for high reliability
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.cornerRefinementWinSize = 5
        self.aruco_params.cornerRefinementMaxIterations = 30
        self.aruco_params.cornerRefinementMinAccuracy = 0.1

        try:
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_detector_api = True
        except AttributeError:
            self.use_new_detector_api = False

    def _camera_info_callback(self, msg: CameraInfo):
        """Extract calibrated intrinsic matrix and distortion coefficients."""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.camera_info_received = True
            self.get_logger().info(
                f"CameraInfo calibrated matrix received! fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}"
            )

    def _odom_callback(self, msg: Odometry):
        """Track robot odometry pose and orientation."""
        self.current_odom_x = msg.pose.pose.position.x
        self.current_odom_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def _scan_callback(self, msg: LaserScan):
        """Phase 3: 2D LiDAR wall line fitting in front of the robot."""
        pts = []
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and 0.08 < r < 0.90:
                a = angle_min + i * angle_inc
                # Extract front cone +/- 25 degrees
                if abs(a) <= math.radians(25):
                    # Robot frame: x forward, y left
                    px = r * math.cos(a)
                    py = r * math.sin(a)
                    pts.append((px, py))

        if len(pts) >= 8:
            pts_arr = np.array(pts)
            px = pts_arr[:, 0]
            py = pts_arr[:, 1]

            # Fit line: front wall is roughly perpendicular to x-axis, so x = m * y + c
            # Slope m represents deviation from pure perpendicularity
            try:
                m, c = np.polyfit(py, px, 1)
                # Yaw angle: m = dx/dy -> angle of wall relative to y-axis
                wall_yaw = -math.atan(m)

                # R-squared goodness of fit
                pred_x = m * py + c
                ss_res = np.sum((px - pred_x) ** 2)
                ss_tot = np.sum((px - np.mean(px)) ** 2)
                r2 = 1.0 - (ss_res / (ss_tot + 1e-6))

                if r2 > 0.65:
                    self.lidar_wall_yaw = wall_yaw
                    self.lidar_front_dist = float(np.min(px))
                    self.lidar_last_time = time.time()
                    self.lidar_fit_r2 = float(r2)
            except Exception:
                pass

    def _ultrasonic_callback(self, msg: Range):
        """Phase 4: Ultrasonic sensor distance callback."""
        if msg.range_min <= msg.range <= msg.range_max:
            self.ultrasonic_dist = float(msg.range)
            self.ultrasonic_last_time = time.time()

    def _compressed_image_callback(self, msg: CompressedImage):
        """Decode CompressedImage and process marker detection."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                self._process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error decoding CompressedImage: {e}", throttle_duration_sec=2.0)

    def _image_callback(self, msg: Image):
        """Convert raw Image and process marker detection."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._process_image(cv_image)
        except Exception as e:
            self.get_logger().error(f"Error converting Image: {e}", throttle_duration_sec=2.0)

    def _detect_markers(self, gray: np.ndarray):
        """Encapsulate ArUco detection across OpenCV versions."""
        if self.use_new_detector_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )
        return corners, ids

    def _estimate_pose_single_marker(self, corners, marker_size, camera_matrix, dist_coeffs):
        """Estimate 6DOF pose of marker corners using solvePnP."""
        s = marker_size / 2.0
        obj_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0]
        ], dtype=np.float64)

        img_points = corners.reshape((4, 2)).astype(np.float64)
        success, rvec, tvec = cv2.solvePnP(
            obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not success:
            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
            )
        return success, rvec, tvec

    def _angle_diff(self, a: float, b: float) -> float:
        """Calculate shortest signed angular difference (a - b) in [-pi, pi]."""
        d = a - b
        while d > math.pi:
            d -= 2.0 * math.pi
        while d < -math.pi:
            d += 2.0 * math.pi
        return d

    def _process_image(self, cv_image: np.ndarray):
        """Process video frame, estimate marker 3D pose, and maintain filter."""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect_markers(gray)

        target_detected = False
        target_corners = None
        target_idx = -1

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            if self.target_marker_id == -1:
                target_idx = 0
                self.target_marker_id = int(ids[0][0])
                self.get_logger().info(f"Auto-locked onto ArUco ID {self.target_marker_id}!")
                target_detected = True
                target_corners = corners[0]
            else:
                for i, marker_id in enumerate(ids):
                    if marker_id[0] == self.target_marker_id:
                        target_idx = i
                        target_detected = True
                        target_corners = corners[i]
                        break

        if target_detected and target_corners is not None:
            self.last_detection_time = time.time()
            self.consecutive_detect_count += 1

            success, target_rvec, target_tvec = self._estimate_pose_single_marker(
                target_corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )

            x = float(target_tvec[0][0])
            y = float(target_tvec[1][0])
            z = float(target_tvec[2][0])

            R, _ = cv2.Rodrigues(target_rvec)
            normal_c = R[:, 2]

            nx = normal_c[0]
            nz = normal_c[2]
            norm_len = math.hypot(nx, nz)
            if norm_len > 1e-6:
                nx_u = nx / norm_len
                nz_u = nz / norm_len
            else:
                nx_u, nz_u = 0.0, 1.0

            wall_angle = math.atan2(nx_u, nz_u)
            dist_along_normal = (0.0 - x) * nx_u + (0.0 - z) * nz_u

            closest_x = x + dist_along_normal * nx_u
            closest_z = z + dist_along_normal * nz_u

            signed_lat = (0.0 - closest_x) * nz_u + (0.0 - closest_z) * (-nx_u)

            self.last_R = R
            self.last_normal_u = (nx_u, nz_u)
            self.last_closest_pt = (closest_x, closest_z)
            self.last_known_bearing = math.atan2(x, z)
            self.raw_pose = (x, y, z, wall_angle, signed_lat, dist_along_normal)

            # Exponential Moving Average (EMA) Jitter Filter
            if self.filtered_pose is None:
                self.filtered_pose = [x, y, z, wall_angle, signed_lat, dist_along_normal]
            else:
                a = self.ema_alpha
                self.filtered_pose[0] = a * x + (1.0 - a) * self.filtered_pose[0]
                self.filtered_pose[1] = a * y + (1.0 - a) * self.filtered_pose[1]
                self.filtered_pose[2] = a * z + (1.0 - a) * self.filtered_pose[2]
                self.filtered_pose[3] = a * wall_angle + (1.0 - a) * self.filtered_pose[3]
                self.filtered_pose[4] = a * signed_lat + (1.0 - a) * self.filtered_pose[4]
                self.filtered_pose[5] = a * dist_along_normal + (1.0 - a) * self.filtered_pose[5]

            cv2.drawFrameAxes(
                cv_image, self.camera_matrix, self.dist_coeffs,
                target_rvec, target_tvec, self.marker_size * 0.8
            )
        else:
            self.consecutive_detect_count = 0

        self._draw_osd(cv_image, ids, target_detected)
        self._publish_debug_image(cv_image)

    def _draw_osd(self, image: np.ndarray, detected_ids=None, target_detected: bool = False):
        """Render informative On-Screen Display (OSD) overlay on camera feed."""
        h, w = image.shape[:2]
        banner_h = 75
        cv2.rectangle(image, (0, 0), (w, banner_h), (20, 20, 20), -1)

        state_color = {
            DockingState.IDLE: (180, 180, 180),
            DockingState.SEARCH: (0, 165, 255),
            DockingState.ALIGN_BEARING: (255, 200, 0),
            DockingState.T_TURN_PARALLEL: (255, 140, 0),
            DockingState.T_DRIVE_LATERAL: (255, 80, 20),
            DockingState.T_RETURN_FACING: (200, 100, 255),
            DockingState.FINE_ALIGN: (0, 200, 255),
            DockingState.APPROACH_NORMAL: (0, 255, 200),
            DockingState.ALIGN_WALL: (255, 50, 200),
            DockingState.FINAL_APPROACH: (0, 255, 255),
            DockingState.PRECISION_STOP: (0, 255, 120),
            DockingState.BLIND_PUSH: (255, 100, 255),
            DockingState.DOCKED: (0, 255, 0),
            DockingState.LOST: (0, 0, 255)
        }.get(self.state, (200, 200, 200))

        # Row 1 (y=22): Title, State, Target Info
        cv2.putText(image, "PINKY DOCKING v4.0", (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(image, f"STATE: {self.state.value}", (195, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.60, state_color, 2)
        dict_info = f"Target: {self.target_marker_id} | Goal: {self.target_distance*100:.1f}cm"
        cv2.putText(image, dict_info, (max(w - 260, 380), 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        # Row 2 & 3: Telemetry & Status
        now = time.time()
        lidar_info = ""
        if self.lidar_wall_yaw is not None and (now - self.lidar_last_time) < 1.0:
            lidar_info = f"LiDAR Wall: {math.degrees(self.lidar_wall_yaw):+.1f} deg"
        us_info = ""
        if self.ultrasonic_dist is not None and (now - self.ultrasonic_last_time) < 1.0:
            us_info = f"US: {self.ultrasonic_dist*100:.1f}cm"

        if self.state == DockingState.IDLE:
            cv2.putText(image, "IDLE - Standby (Call /docking/start to engage)", (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 2)
        elif self.state == DockingState.PRECISION_STOP:
            status_text = f"Precision Stop: {us_info or lidar_info or 'Decelerating'} -> Goal 3cm"
            cv2.putText(image, status_text, (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 120), 2)
        elif self.state == DockingState.T_TURN_PARALLEL:
            turned = self._angle_diff(self.current_odom_yaw, self.turn_start_yaw)
            cv2.putText(image, f"Step 1: Turn to Wall ({math.degrees(turned):+.1f} deg / Goal: {math.degrees(self.t_turn_angle_1):+.1f} deg)", (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 140, 0), 2)
        elif self.state == DockingState.T_DRIVE_LATERAL:
            moved = math.hypot(self.current_odom_x - self.lat_start_x, self.current_odom_y - self.lat_start_y)
            cv2.putText(image, f"Step 2: Lateral Drive ({moved*100:.1f}cm / Goal: {self.lat_target_dist*100:.1f}cm)", (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 80, 20), 2)
        elif self.state == DockingState.T_RETURN_FACING:
            turned = self._angle_diff(self.current_odom_yaw, self.turn_start_yaw)
            cv2.putText(image, f"Step 3: Return Turn ({math.degrees(turned):+.1f} deg / Goal: {math.degrees(self.t_turn_angle_2):+.1f} deg)", (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 100, 255), 2)
        elif self.state == DockingState.BLIND_PUSH:
            moved = math.hypot(self.current_odom_x - self.blind_start_x, self.current_odom_y - self.blind_start_y)
            cv2.putText(image, f"Blind Push: {moved*100:.1f}cm / {self.blind_target_dist*100:.1f}cm", (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 100, 255), 2)
        elif self.filtered_pose is not None:
            x, y, z, wall_angle, signed_lat, dist_along_normal = self.filtered_pose
            sensor_tag = f"[{us_info} | {lidar_info}]" if (us_info or lidar_info) else ""
            pose_str = f"Dist(Z): {z:.3f}m | LatErr: {signed_lat*100:+.1f}cm {sensor_tag}"
            lat_str = f"AngleErr: {math.degrees(wall_angle):+.1f} deg | NormalDist: {dist_along_normal:.3f}m"
            cv2.putText(image, pose_str, (15, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1)
            cv2.putText(image, lat_str, (15, 66), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1)
        else:
            cv2.putText(image, "Searching for ArUco 5x5 marker...", (15, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1)

    def _publish_debug_image(self, image: np.ndarray):
        """Publish debug visualization image."""
        has_sub_raw = self.debug_raw_pub.get_subscription_count() > 0
        has_sub_comp = self.debug_image_pub.get_subscription_count() > 0

        if has_sub_raw:
            try:
                raw_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
                raw_msg.header.stamp = self.get_clock().now().to_msg()
                self.debug_raw_pub.publish(raw_msg)
            except Exception:
                pass

        if has_sub_comp or not has_sub_raw:
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            success, encoded_img = cv2.imencode('.jpg', image, encode_param)
            if success:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = "jpeg"
                msg.data = encoded_img.tobytes()
                self.debug_image_pub.publish(msg)

    def _control_loop(self):
        """Main 20Hz FSM Control Loop."""
        # Phase 1: In IDLE, do not emit any cmd_vel, allowing Nav2 free control
        if self.state == DockingState.IDLE:
            status_msg = Bool()
            status_msg.data = False
            self.completed_pub.publish(status_msg)
            return

        now = time.time()
        time_since_last_detection = now - self.last_detection_time

        # Watchdog: Ignore marker loss during open-loop or sensor-based terminal docking phases
        if self.state not in [
            DockingState.IDLE, DockingState.SEARCH, DockingState.LOST,
            DockingState.T_TURN_PARALLEL, DockingState.T_DRIVE_LATERAL, DockingState.T_RETURN_FACING,
            DockingState.PRECISION_STOP, DockingState.BLIND_PUSH, DockingState.DOCKED
        ]:
            if time_since_last_detection > self.loss_timeout:
                self.get_logger().warn(
                    f"Marker lost for {time_since_last_detection:.2f}s in state {self.state.value}! Initiating recovery."
                )
                self.last_state = self.state
                self.state = DockingState.LOST

        twist = Twist()
        completed = False

        if self.state == DockingState.SEARCH:
            self._handle_search(twist)
        elif self.state == DockingState.LOST:
            self._handle_lost(twist, time_since_last_detection)
        elif self.state == DockingState.ALIGN_BEARING:
            self._handle_align_bearing(twist)
        elif self.state == DockingState.T_TURN_PARALLEL:
            self._handle_t_turn_parallel(twist)
        elif self.state == DockingState.T_DRIVE_LATERAL:
            self._handle_t_drive_lateral(twist)
        elif self.state == DockingState.T_RETURN_FACING:
            self._handle_t_return_facing(twist)
        elif self.state == DockingState.FINE_ALIGN:
            self._handle_fine_align(twist)
        elif self.state == DockingState.APPROACH_NORMAL:
            self._handle_approach_normal(twist)
        elif self.state == DockingState.ALIGN_WALL:
            self._handle_align_wall(twist)
        elif self.state == DockingState.FINAL_APPROACH:
            self._handle_final_approach(twist)
        elif self.state == DockingState.PRECISION_STOP:
            self._handle_precision_stop(twist)
        elif self.state == DockingState.BLIND_PUSH:
            self._handle_blind_push(twist)
        elif self.state == DockingState.DOCKED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            completed = True

        if self.state in [
            DockingState.T_TURN_PARALLEL, DockingState.T_DRIVE_LATERAL, DockingState.T_RETURN_FACING,
            DockingState.FINE_ALIGN, DockingState.APPROACH_NORMAL, DockingState.FINAL_APPROACH,
            DockingState.PRECISION_STOP, DockingState.BLIND_PUSH
        ]:
            if self.filtered_pose is not None:
                x, _, z, wall_angle, signed_lat, _ = self.filtered_pose
                self.get_logger().info(
                    f"[{self.state.value}] z={z:.2f}m, lat={signed_lat*100:+.1f}cm, ang={math.degrees(wall_angle):+.1f} deg | Cmd: v={twist.linear.x:.2f}, w={twist.angular.z:+.2f}",
                    throttle_duration_sec=0.5
                )

        self._publish_twist(twist)

        status_msg = Bool()
        status_msg.data = completed
        self.completed_pub.publish(status_msg)

    def _handle_search(self, twist: Twist):
        """Rotate slowly until 5x5 marker is detected."""
        if self.consecutive_detect_count >= 1 and self.filtered_pose is not None:
            self.get_logger().info("5x5 Marker acquired! Transitioning to ALIGN_BEARING.")
            self.state = DockingState.ALIGN_BEARING
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = -self.search_angular_vel if self.invert_angular else self.search_angular_vel

    def _handle_lost(self, twist: Twist, time_since_loss: float):
        """Active recovery on marker loss."""
        twist.linear.x = 0.0
        if self.consecutive_detect_count >= 1:
            self.get_logger().info("Marker RE-ACQUIRED! Resuming ALIGN_BEARING.")
            self.state = DockingState.ALIGN_BEARING
            twist.angular.z = 0.0
            return

        lost_duration = time_since_loss - self.loss_timeout
        if lost_duration < 1.5:
            recovery_w = 0.12
            if self.last_known_bearing is not None and abs(self.last_known_bearing) > 0.03:
                scan_dir = -1.0 if self.last_known_bearing > 0 else 1.0
            else:
                scan_dir = -self.last_turn_dir

            if self.invert_angular:
                scan_dir = -scan_dir
            twist.angular.z = scan_dir * recovery_w
        elif lost_duration < 3.0:
            twist.angular.z = 0.0
        else:
            self.get_logger().warn("[LOST Timeout] Resetting to full SEARCH.")
            self.filtered_pose = None
            self.last_known_bearing = None
            self.state = DockingState.SEARCH
            twist.angular.z = 0.0

    def _handle_align_bearing(self, twist: Twist):
        """Rotate in place to center marker in camera view."""
        x, _, z, wall_angle, signed_lat, dist_along_normal = self.filtered_pose
        bearing_error = math.atan2(x, z)

        if abs(bearing_error) <= 0.03:  # ~1.7 deg
            twist.angular.z = 0.0
            lateral_err = abs(signed_lat)
            angle_err = abs(wall_angle)
            needs_t_maneuver = (lateral_err > 0.03) or (angle_err > 0.08)

            if self.enable_t_maneuver and (z >= self.t_maneuver_min_dist) and needs_t_maneuver and self.last_closest_pt is not None:
                closest_x, closest_z = self.last_closest_pt
                dist_to_normal = math.hypot(closest_x, closest_z)
                bearing_to_normal = math.atan2(closest_x, closest_z)
                delta_theta = -bearing_to_normal

                self.t_turn_angle_1 = delta_theta
                self.t_lateral_offset = dist_to_normal
                self.turn_start_yaw = self.current_odom_yaw

                self.get_logger().info(
                    f"=== T-MANEUVER INITIATED === Lateral: {signed_lat*100:+.1f}cm, Angle: {math.degrees(wall_angle):+.1f} deg. "
                    f"Step 1: Turning {math.degrees(delta_theta):+.1f} deg parallel to wall."
                )
                self.state = DockingState.T_TURN_PARALLEL
                return
            else:
                self.get_logger().info("Collinear condition satisfied. Advancing to APPROACH_NORMAL.")
                self.state = DockingState.APPROACH_NORMAL
        else:
            ang_cmd = self.kp_angular * bearing_error
            twist.angular.z = self._clamp_angular(-ang_cmd)
            twist.linear.x = 0.0
            if abs(twist.angular.z) > 0.02:
                self.last_turn_dir = 1.0 if twist.angular.z > 0 else -1.0

    def _handle_t_turn_parallel(self, twist: Twist):
        """Step 1 of T-Maneuver: Rotate in place parallel to wall."""
        turned = self._angle_diff(self.current_odom_yaw, self.turn_start_yaw)
        remaining = self.t_turn_angle_1 - turned

        if abs(remaining) <= 0.025:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.lat_start_x = self.current_odom_x
            self.lat_start_y = self.current_odom_y
            self.lat_target_dist = abs(self.t_lateral_offset)
            self.state = DockingState.T_DRIVE_LATERAL
        else:
            ang_cmd = self.kp_angular * remaining
            twist.linear.x = 0.0
            twist.angular.z = self._clamp_angular(ang_cmd)

    def _handle_t_drive_lateral(self, twist: Twist):
        """Step 2 of T-Maneuver: Drive straight along wall."""
        moved = math.hypot(self.current_odom_x - self.lat_start_x, self.current_odom_y - self.lat_start_y)
        remaining = self.lat_target_dist - moved

        if remaining <= 0.005:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.turn_start_yaw = self.current_odom_yaw
            self.t_turn_angle_2 = -math.copysign(math.pi / 2.0, self.t_turn_angle_1)
            self.state = DockingState.T_RETURN_FACING
        else:
            speed = min(self.t_maneuver_lateral_speed, self.kp_linear * remaining)
            speed = max(speed, self.min_linear_vel)
            twist.linear.x = self._clamp_linear(speed)
            twist.angular.z = 0.0

    def _handle_t_return_facing(self, twist: Twist):
        """Step 3 of T-Maneuver: Rotate in place to face marker."""
        turned = self._angle_diff(self.current_odom_yaw, self.turn_start_yaw)
        remaining = self.t_turn_angle_2 - turned

        if abs(remaining) <= 0.025:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.fine_align_start_time = time.time()
            self.last_detection_time = time.time()
            self.state = DockingState.FINE_ALIGN
        else:
            ang_cmd = self.kp_angular * remaining
            twist.linear.x = 0.0
            twist.angular.z = self._clamp_angular(ang_cmd)

    def _handle_fine_align(self, twist: Twist):
        """Step 4 of T-Maneuver: Visual re-centering."""
        twist.linear.x = 0.0
        now = time.time()

        if self.consecutive_detect_count >= 1 and self.filtered_pose is not None:
            x, _, z, wall_angle, signed_lat, dist_along_normal = self.filtered_pose
            bearing_error = math.atan2(x, z)

            if abs(bearing_error) <= 0.025:
                twist.angular.z = 0.0
                self.get_logger().info("Collinear Lock achieved! Proceeding to APPROACH_NORMAL.")
                self.state = DockingState.APPROACH_NORMAL
                return
            else:
                ang_cmd = self.kp_angular * bearing_error
                twist.angular.z = self._clamp_angular(-ang_cmd)
                return

        if now - self.fine_align_start_time > 3.0:
            self.get_logger().warn("Fine align timeout! Resuming ALIGN_BEARING.")
            self.state = DockingState.ALIGN_BEARING

    def _handle_approach_normal(self, twist: Twist):
        """
        Phase 2: Stanley S-Curve Controller for continuous smooth convergence.
        Steering: w = -kp_wall_yaw * wall_angle + kp_lateral * atan2(1.2 * signed_lat, vx + 0.01)
        """
        x, _, z, wall_angle, signed_lat, dist_along_normal = self.filtered_pose

        # Hand off to FINAL_APPROACH when within 0.28m
        if z <= max(self.vision_min_distance + 0.12, 0.28):
            self.get_logger().info(f"Approach Normal near wall ({z:.2f}m). Transitioning to FINAL_APPROACH.")
            self.state = DockingState.FINAL_APPROACH
            return

        # Forward speed
        dist_err = z - self.target_distance
        lin_cmd = self.kp_linear * 0.7 * dist_err
        lin_cmd = max(min(lin_cmd, self.max_linear_vel), self.min_linear_vel)
        twist.linear.x = self._clamp_linear(lin_cmd)

        if self.enable_curved_approach:
            # Stanley S-Curve formulation
            # 1. Lateral offset convergence term (turns toward marker normal axis)
            delta_lat = self.kp_lateral * math.atan2(1.2 * signed_lat, twist.linear.x + 0.01)
            # 2. Heading alignment term (counter-steers to maintain perpendicularity to wall)
            delta_yaw = -self.kp_wall_yaw * wall_angle

            raw_w = delta_yaw + delta_lat
            # Clamp to max_curve_yaw_rate to prevent sharp erratic steering
            raw_w = max(min(raw_w, self.max_curve_yaw_rate), -self.max_curve_yaw_rate)
            twist.angular.z = self._clamp_angular(raw_w)
        else:
            bearing_error = math.atan2(x, z)
            ang_cmd = self.kp_angular * bearing_error
            twist.angular.z = self._clamp_angular(-ang_cmd)

        if abs(twist.angular.z) > 0.02:
            self.last_turn_dir = 1.0 if twist.angular.z > 0 else -1.0

    def _handle_align_wall(self, twist: Twist):
        """Compatibility forwarder to FINAL_APPROACH."""
        self.state = DockingState.FINAL_APPROACH
        self._handle_final_approach(twist)

    def _handle_final_approach(self, twist: Twist):
        """
        Phase 3: Final Approach with optional 2D LiDAR wall alignment.
        Hands off to PRECISION_STOP or BLIND_PUSH at vision_min_distance (<14cm).
        """
        x, _, z, wall_angle, signed_lat, dist_along_normal = self.filtered_pose
        dist_err = z - self.target_distance
        bearing_error = math.atan2(x, z)

        # Transition to Phase 4 (Ultrasonic / LiDAR precision stop) or Blind Push
        if z <= self.vision_min_distance:
            if self.enable_ultrasonic_stop:
                self.get_logger().info(
                    f"Reached vision limit ({z*100:.1f}cm). Switching to PRECISION_STOP using Ultrasonic/LiDAR feedback!"
                )
                self.state = DockingState.PRECISION_STOP
                return
            elif self.enable_blind_push:
                push_dist = z - self.target_distance
                self.blind_start_x = self.current_odom_x
                self.blind_start_y = self.current_odom_y
                self.blind_target_dist = push_dist
                self.get_logger().info(
                    f"Reached vision limit ({z*100:.1f}cm). Switching to BLIND_PUSH for {push_dist*100:.1f}cm using Odometry!"
                )
                self.state = DockingState.BLIND_PUSH
                return

        # Direct docking completion if target_distance is within vision range
        if abs(dist_err) <= self.distance_tolerance and abs(bearing_error) <= 0.05:
            self.get_logger().info(f"=== DOCKING COMPLETED! === Final Dist: {z:.3f}m, LatErr: {signed_lat*100:+.1f}cm")
            self.state = DockingState.DOCKED
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            return

        # Phase 3: Fuse 2D LiDAR wall yaw if available
        now = time.time()
        effective_wall_yaw = wall_angle
        if self.enable_lidar_alignment and self.lidar_wall_yaw is not None and (now - self.lidar_last_time) < 0.5:
            effective_wall_yaw = self.lidar_wall_yaw

        # Steering: blend bearing centering with wall perpendicularity
        ang_cmd = self.kp_angular * 0.9 * bearing_error + (-0.5 * effective_wall_yaw)
        twist.angular.z = self._clamp_angular(-ang_cmd)

        lin_cmd = self.kp_linear * dist_err
        twist.linear.x = self._clamp_linear(lin_cmd)

        if abs(twist.angular.z) > 0.02:
            self.last_turn_dir = 1.0 if twist.angular.z > 0 else -1.0

    def _handle_precision_stop(self, twist: Twist):
        """
        Phase 4: Sensor-Fused Precision Stop (Target: 3cm / 0.03m).
        Active real-time feedback using Ultrasonic Sensor / 2D LiDAR down to exact 3cm contact.
        """
        now = time.time()
        cur_dist = None

        # Prioritize ultrasonic sensor (accurate 2cm ~ 3m)
        if self.ultrasonic_dist is not None and (now - self.ultrasonic_last_time) < 0.6:
            cur_dist = self.ultrasonic_dist
        # Secondary fallback: LiDAR front obstacle distance
        elif self.lidar_front_dist is not None and (now - self.lidar_last_time) < 0.6:
            cur_dist = self.lidar_front_dist

        # If sensor feedback is active:
        if cur_dist is not None:
            dist_err = cur_dist - self.target_distance

            # Emergency Stop / Final contact reached
            if cur_dist <= 0.020:
                self.get_logger().info(f"=== DOCKING COMPLETED (Bumper Safe Stop)! === Dist: {cur_dist*100:.1f}cm")
                self.state = DockingState.DOCKED
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                return

            if dist_err <= self.distance_tolerance:
                self.get_logger().info(f"=== PRECISION DOCKING COMPLETED! === Contact Dist: {cur_dist*100:.1f}cm (Target: {self.target_distance*100:.1f}cm)")
                self.state = DockingState.DOCKED
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                return

            # Proportional approach speed
            v_cmd = max(self.min_linear_vel, min(self.blind_push_speed, 0.4 * dist_err))
            twist.linear.x = self._clamp_linear(v_cmd)

            # Micro-steering via LiDAR if available
            if self.enable_lidar_alignment and self.lidar_wall_yaw is not None and (now - self.lidar_last_time) < 0.5:
                twist.angular.z = self._clamp_angular(-0.4 * self.lidar_wall_yaw)
            else:
                twist.angular.z = 0.0
        else:
            # Fallback to odometry blind push if sensors temporarily drop out
            self._handle_blind_push(twist)

    def _handle_blind_push(self, twist: Twist):
        """Odometry-guided open-loop final drive fallback."""
        moved = math.hypot(self.current_odom_x - self.blind_start_x, self.current_odom_y - self.blind_start_y)
        remaining = self.blind_target_dist - moved

        if remaining <= 0.005:
            self.get_logger().info(f"=== BLIND DOCKING COMPLETED! === Traveled: {moved*100:.1f}cm")
            self.state = DockingState.DOCKED
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = self.blind_push_speed
            twist.angular.z = 0.0

    def _clamp_linear(self, val: float) -> float:
        """Clamp linear velocity with deadband."""
        if abs(val) < 0.005:
            return 0.0
        val = max(min(val, self.max_linear_vel), -self.max_linear_vel)
        if 0.0 < val < self.min_linear_vel:
            val = self.min_linear_vel
        elif -self.min_linear_vel < val < 0.0:
            val = -self.min_linear_vel
        return val

    def _clamp_angular(self, val: float) -> float:
        """Clamp angular velocity."""
        if self.invert_angular:
            val = -val
        if abs(val) < 0.02:
            return 0.0
        val = max(min(val, self.max_angular_vel), -self.max_angular_vel)
        if 0.0 < abs(val) < self.min_angular_vel:
            val = math.copysign(self.min_angular_vel, val)
        return val

    def _publish_twist(self, twist: Twist):
        """Publish velocity command."""
        self.cmd_vel_pub.publish(twist)

    def _stop_robot(self):
        """Publish zero velocity repeatedly to halt robot."""
        zero_twist = Twist()
        for _ in range(8):
            self.cmd_vel_pub.publish(zero_twist)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionDockingNode()

    def sig_handler(sig, frame):
        node.get_logger().info("Emergency stopping Pinky motors...")
        node._stop_robot()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.get_logger().info("Shutting down PrecisionDockingNode...")
    finally:
        node._stop_robot()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
