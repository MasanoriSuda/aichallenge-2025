#!/usr/bin/env python3

import yaml
from typing import List, Tuple, Optional, NamedTuple
import dataclasses
from scipy import sparse
from scipy.sparse import dia_matrix
import numpy as np
import copy
import os
import shutil
from datetime import datetime

# ROS 2
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Empty, Bool, Float32MultiArray, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Pose2D, Point, Vector3
from std_msgs.msg import ColorRGBA

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

# autoware
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from v2x_msgs.msg import V2XVehiclePositionArray
from multi_purpose_mpc_ros.v2x_vehicle_tracker import (
    V2XVehicleTracker,
    predictions_to_obstacles,
)
from multi_purpose_mpc_ros.v2x_stop_planner import (
    V2XStopConfig,
    V2XStopPlanner,
    V2XStopResult,
)
from multi_purpose_mpc_ros.v2x_overtake_planner import (
    V2XOvertakeConfig,
    V2XOvertakePlanner,
    V2XOvertakeResult,
)

# Multi_Purpose_MPC
from multi_purpose_mpc_ros.core.map import Map, Obstacle
from multi_purpose_mpc_ros.core.reference_path import ReferencePath
from multi_purpose_mpc_ros.core.spatial_bicycle_models import BicycleModel
from multi_purpose_mpc_ros.core.MPC import MPC
from multi_purpose_mpc_ros.core.utils import load_waypoints, kmh_to_m_per_sec, load_ref_path

# Project
from multi_purpose_mpc_ros.common import convert_to_namedtuple, file_exists
from multi_purpose_mpc_ros.simulation_logger import SimulationLogger
from multi_purpose_mpc_ros.obstacle_manager import ObstacleManager
from multi_purpose_mpc_ros.exexution_stats import ExecutionStats
from multi_purpose_mpc_ros_msgs.msg import AckermannControlBoostCommand, PathConstraints, BorderCells
from multi_purpose_mpc_ros.tools.reference_velocity_configulator import ReferenceVelocityConfigulator


RED = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
YELLOW = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
CYAN = ColorRGBA(r=0.0, g=156.0 / 255.0, b=209.0 / 255.0, a=1.0)

def array_to_ackermann_control_command(stamp, u: np.ndarray, acc: float) -> AckermannControlCommand:
    msg = AckermannControlCommand()
    msg.stamp = stamp
    msg.lateral.stamp = stamp
    msg.lateral.steering_tire_angle = u[1]
    msg.lateral.steering_tire_rotation_rate = 2.0
    msg.longitudinal.stamp = stamp
    msg.longitudinal.speed = u[0]
    msg.longitudinal.acceleration = acc
    return msg

def yaw_from_quaternion(q: Quaternion):
    sqx = q.x * q.x
    sqy = q.y * q.y
    sqz = q.z * q.z
    sqw = q.w * q.w

    # Cases derived from https://orbitalstation.wordpress.com/tag/quaternion/
    sarg = -2 * (q.x*q.z - q.w*q.y) / (sqx + sqy + sqz + sqw) # normalization added from urdfom_headers

    if sarg <= -0.99999:
        yaw = -2. * np.arctan2(q.y, q.x)
    elif sarg >= 0.99999:
        yaw = 2. * np.arctan2(q.y, q.x)
    else:
        yaw = np.arctan2(2. * (q.x*q.y + q.w*q.z), sqw + sqx - sqy - sqz)

    return yaw

def odom_to_pose_2d(odom: Odometry) -> Pose2D:
    pose = Pose2D()
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.theta = yaw_from_quaternion(odom.pose.pose.orientation)

    return pose

@dataclasses.dataclass
class MPCConfig:
    N: int
    Q: dia_matrix
    R: dia_matrix
    QN: dia_matrix
    v_max: float
    a_min: float
    a_max: float
    ay_max: float
    delta_max: float
    steer_rate_max: float
    control_rate: float
    steering_tire_angle_gain_var: float
    accel_low_pass_gain: float
    steer_low_pass_gain: float
    wp_id_offset: int
    use_max_kappa_pred: bool


class MPCController(Node):

    PKG_PATH: str = get_package_share_directory('multi_purpose_mpc_ros') + "/"
    # MAX_LAPS = 6
    MAX_LAPS = 10000
    BUG_VEL = 40.0 # km/h
    BUG_ACC = 400.0

    SHOW_PLOT_ANIMATION = False
    PLOT_RESULTS = False
    ANIMATION_INTERVAL = 20

    KP = 100.0

    def __init__(self, config_path: str, ref_vel_config_path: Optional[str]) -> None:
        super().__init__("mpc_controller") # type: ignore

        # declare parameters
        self.declare_parameter("use_boost_acceleration", False)
        self.declare_parameter("use_obstacle_avoidance", False)
        self.declare_parameter("use_v2x_stop", True)
        self.declare_parameter("use_v2x_overtake", False)
        self.declare_parameter("use_stats", False)

        # get parameters
        self.use_sim_time = self.get_parameter("use_sim_time").get_parameter_value().bool_value
        self.USE_BUG_ACC = self.get_parameter("use_boost_acceleration").get_parameter_value().bool_value
        self.USE_OBSTACLE_AVOIDANCE = self.get_parameter("use_obstacle_avoidance").get_parameter_value().bool_value
        self.USE_V2X_STOP = self.get_parameter("use_v2x_stop").get_parameter_value().bool_value
        self.USE_V2X_OVERTAKE = self.get_parameter("use_v2x_overtake").get_parameter_value().bool_value
        self.use_stats = self.get_parameter("use_stats").get_parameter_value().bool_value

        self._config_path = config_path
        self._ref_vel_config_path: Optional[str] = ref_vel_config_path
        self._cfg = self._load_config()
        self._odom: Optional[Odometry] = None
        self._enable_control = True
        self._initialize()
        self._setup_parameters_callback()
        self._setup_pub_sub()

        if self.use_sim_time:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("use_sim_time is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_BUG_ACC:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_BUG_ACC is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_OBSTACLE_AVOIDANCE:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_OBSTACLE_AVOIDANCE is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_V2X_STOP:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_V2X_STOP is enabled!")
            self.get_logger().warn("------------------------------------")
        if self.USE_V2X_OVERTAKE:
            self.get_logger().warn("------------------------------------")
            self.get_logger().warn("USE_V2X_OVERTAKE is enabled!")
            self.get_logger().warn("------------------------------------")

    def _load_config(self) -> NamedTuple:

        # logging content
        with open(self._config_path, "r") as f:
            config_content = f.read()
            self.get_logger().info(
                "\n" +
                "----- config.yaml -----\n"+
                config_content + "\n" +
                "-----------------------")

        if self._ref_vel_config_path is not None:
            with open(self._ref_vel_config_path, "r") as f:
                ref_vel_config_content = f.read()
                self.get_logger().info(
                    "\n" +
                    "----- ref_vel.yaml -----\n"+
                    ref_vel_config_content + "\n" +
                    "-----------------------")

        with open(self._config_path, "r") as f:
            cfg: NamedTuple = convert_to_namedtuple(yaml.safe_load(f)) # type: ignore

        # Check if the files exist
        mandatory_files = [cfg.map.yaml_path, cfg.waypoints.csv_path] # type: ignore
        for file_path in mandatory_files:
            file_exists(self.in_pkg_share(file_path))
        return cfg

    def _create_reference_path_from_autoware_trajectory(self, trajectory: Trajectory) -> Optional[ReferencePath]:
        wp_x = [0] * len(trajectory.points)
        wp_y = [0] * len(trajectory.points)
        for i, p in enumerate(trajectory.points):
            wp_x[i] = p.pose.position.x
            wp_y[i] = p.pose.position.y

        cfg_ref_path = self._cfg.reference_path # type: ignore
        reference_path = ReferencePath(
            self._map,
            wp_x,
            wp_y,
            cfg_ref_path.resolution,
            cfg_ref_path.smoothing_distance,
            cfg_ref_path.max_width,
            cfg_ref_path.circular)

        mpc_config = self._mpc_cfg
        speed_profile_constraints = {
            "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
            "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}

        if not reference_path.compute_speed_profile(speed_profile_constraints):
            return None

        return reference_path

    def _setup_parameters_callback(self) -> None:
        def declatre_parameters():
            cfg_mpc = self._cfg.mpc
            self.declare_parameter("v_max", cfg_mpc.v_max)
            self.declare_parameter("steering_tire_angle_gain_var", cfg_mpc.steering_tire_angle_gain_var)
            self.declare_parameter("Q0", cfg_mpc.Q[0])
            self.declare_parameter("Q1", cfg_mpc.Q[1])
            self.declare_parameter("Q2", cfg_mpc.Q[2])
            self.declare_parameter("R0", cfg_mpc.R[0])
            self.declare_parameter("R1", cfg_mpc.R[1])
            self.declare_parameter("QN0", cfg_mpc.QN[0])
            self.declare_parameter("QN1", cfg_mpc.QN[1])
            self.declare_parameter("QN2", cfg_mpc.QN[2])

            mpc_cfg = self._mpc_cfg
            self.declare_parameter("ay_max", mpc_cfg.ay_max)
            self.declare_parameter("accel_low_pass_gain", mpc_cfg.accel_low_pass_gain)
            self.declare_parameter("steer_low_pass_gain", mpc_cfg.steer_low_pass_gain)
            self.declare_parameter("wp_id_offset", mpc_cfg.wp_id_offset)

        def param_cb(parameters):
            cfg_mpc = self._cfg.mpc # type: ignore
            mpc_cfg = self._mpc_cfg

            def update_Q(index: int, value: float):
                cfg_mpc.Q[index] = value
                mpc_cfg.Q = sparse.diags(cfg_mpc.Q)
                self._mpc.update_Q(mpc_cfg.Q)
                self.get_logger().warn(f"Q[{index}] was updated to '{value}'")

            def update_R(index: int, value: float):
                cfg_mpc.R[index] = value
                mpc_cfg.R = sparse.diags(cfg_mpc.R)
                self._mpc.update_R(mpc_cfg.R)
                self.get_logger().warn(f"R[{index}] was updated to '{value}'")

            def update_QN(index: int, value: float):
                cfg_mpc.QN[index] = value
                mpc_cfg.QN = sparse.diags(cfg_mpc.QN)
                self._mpc.update_QN(mpc_cfg.QN)
                self.get_logger().warn(f"QN[{index}] was updated to '{value}'")

            for param in parameters:
                if param.name == "v_max" and param.type_ == Parameter.Type.DOUBLE:
                    v_max_mps = kmh_to_m_per_sec(param.value)
                    mpc_cfg.v_max = v_max_mps
                    self._mpc.update_v_max(v_max_mps)
                    v_ref: List[float] = [v_max_mps] * len(self._reference_path.waypoints)
                    self._reference_path.set_v_ref(v_ref)

                    self.get_logger().warn(f"v_max was updated to '{param.value}' [km/h]")

                elif param.name == "steering_tire_angle_gain_var" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steering_tire_angle_gain_var = param.value
                    self.get_logger().warn(f"steering_tire_angle_gain_var was updated to '{param.value}'")

                elif param.name == "Q0" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(0, param.value)
                elif param.name == "Q1" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(1, param.value)
                elif param.name == "Q2" and param.type_ == Parameter.Type.DOUBLE:
                    update_Q(2, param.value)


                elif param.name == "R0" and param.type_ == Parameter.Type.DOUBLE:
                    update_R(0, param.value)
                elif param.name == "R1" and param.type_ == Parameter.Type.DOUBLE:
                    update_R(1, param.value)

                elif param.name == "QN0" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(0, param.value)
                elif param.name == "QN1" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(1, param.value)
                elif param.name == "QN2" and param.type_ == Parameter.Type.DOUBLE:
                    update_QN(2, param.value)

                elif param.name == "ay_max" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.ay_max = param.value
                    self._mpc.update_ay_max(param.value)
                    self.get_logger().warn(f"ay_max was updated to '{param.value}'")

                elif param.name == "accel_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.accel_low_pass_gain = param.value
                    self.get_logger().warn(f"accel_low_pass_gain was updated to '{param.value}'")

                elif param.name == "steer_low_pass_gain" and param.type_ == Parameter.Type.DOUBLE:
                    mpc_cfg.steer_low_pass_gain = param.value
                    self.get_logger().warn(f"steer_low_pass_gain was updated to '{param.value}'")

                elif param.name == "wp_id_offset" and param.type_ == Parameter.Type.INTEGER:
                    mpc_cfg.wp_id_offset = param.value
                    self._mpc.update_wp_id_offset(param.value)
                    self.get_logger().warn(f"wp_id_offset was updated to '{param.value}'")


            return SetParametersResult(successful=True)

        declatre_parameters()
        self.add_on_set_parameters_callback(param_cb)

    def _initialize(self) -> None:
        def create_map() -> Map:
            return Map(self.in_pkg_share(self._cfg.map.yaml_path)) # type: ignore

        def create_ref_path(map: Map) -> ReferencePath:
            cfg_ref_path = self._cfg.reference_path # type: ignore

            is_ref_path_given = cfg_ref_path.csv_path != "" # type: ignore
            if is_ref_path_given:
                print("Using given reference path")
                wp_x, wp_y, _, _ = load_ref_path(self.in_pkg_share(self._cfg.reference_path.csv_path)) # type: ignore
                return ReferencePath(
                    map,
                    wp_x,
                    wp_y,
                    cfg_ref_path.resolution,
                    cfg_ref_path.smoothing_distance,
                    cfg_ref_path.max_width,
                    cfg_ref_path.circular)

            else:
                print("Using waypoints to create reference path")
                wp_x, wp_y = load_waypoints(self.in_pkg_share(self._cfg.waypoints.csv_path)) # type: ignore

                return ReferencePath(
                    map,
                    wp_x,
                    wp_y,
                    cfg_ref_path.resolution,
                    cfg_ref_path.smoothing_distance,
                    cfg_ref_path.max_width,
                    cfg_ref_path.circular)


        def create_obstacles() -> List[Obstacle]:
            use_csv_obstacles = self._cfg.obstacles.csv_path != "" # type: ignore
            if use_csv_obstacles:
                obstacles_file_path = self.in_pkg_share(self._cfg.obstacles.csv_path) # type: ignore
                obs_x, obs_y = load_waypoints(obstacles_file_path)
                obstacles = []
                for cx, cy in zip(obs_x, obs_y):
                    obstacles.append(Obstacle(cx=cx, cy=cy, radius=self._cfg.obstacles.radius)) # type: ignore
                self._obstacle_manager = ObstacleManager(self._map, obstacles)
                return obstacles
            else:
                return []

        def create_car(ref_path: ReferencePath) -> BicycleModel:
            cfg_model = self._cfg.bicycle_model # type: ignore
            return BicycleModel(
                ref_path,
                cfg_model.length,
                cfg_model.width,
                1.0 / self._cfg.mpc.control_rate) # type: ignore

        def create_mpc(car: BicycleModel) -> Tuple[MPCConfig, MPC]:
            cfg_mpc = self._cfg.mpc # type: ignore

            mpc_cfg = MPCConfig(
                cfg_mpc.N,
                sparse.diags(cfg_mpc.Q),
                sparse.diags(cfg_mpc.R),
                sparse.diags(cfg_mpc.QN),
                kmh_to_m_per_sec(self.BUG_VEL if self.USE_BUG_ACC else cfg_mpc.v_max),
                cfg_mpc.a_min,
                cfg_mpc.a_max,
                cfg_mpc.ay_max,
                np.deg2rad(cfg_mpc.delta_max_deg),
                cfg_mpc.steer_rate_max,
                cfg_mpc.control_rate,
                cfg_mpc.steering_tire_angle_gain_var,
                cfg_mpc.accel_low_pass_gain,
                cfg_mpc.steer_low_pass_gain,
                cfg_mpc.wp_id_offset,
                cfg_mpc.use_max_kappa_pred)

            state_constraints = {
                "xmin": np.array([-np.inf, -np.inf, -np.inf]),
                "xmax": np.array([np.inf, np.inf, np.inf])}
            input_constraints = {
                "umin": np.array([0.0, -np.tan(mpc_cfg.delta_max) / car.length]),
                "umax": np.array([mpc_cfg.v_max, np.tan(mpc_cfg.delta_max) / car.length])}

            # mpcからのsteer指令出力は、gainを掛けて出力され、その状態で車体のsteer rate limit が適用されるため、
            # mpcの制御計算におけるsteer_rate_maxは、実際のsteer_rate_maxをgainで除した値で設定する
            scaled_steer_rate_max = mpc_cfg.steer_rate_max / mpc_cfg.steering_tire_angle_gain_var

            mpc = MPC(
                car,
                mpc_cfg.N,
                mpc_cfg.Q,
                mpc_cfg.R,
                mpc_cfg.QN,
                state_constraints,
                input_constraints,
                mpc_cfg.ay_max,
                scaled_steer_rate_max,
                mpc_cfg.wp_id_offset,
                self.USE_OBSTACLE_AVOIDANCE,
                self._cfg.reference_path.use_path_constraints_topic,
                mpc_cfg.use_max_kappa_pred)

            return mpc_cfg, mpc

        def compute_speed_profile(car: BicycleModel, mpc_config: MPCConfig) -> None:
            speed_profile_constraints = {
                "a_min": mpc_config.a_min, "a_max": mpc_config.a_max,
                "v_min": 0.0, "v_max": mpc_config.v_max, "ay_max": mpc_config.ay_max}
            car.reference_path.compute_speed_profile(speed_profile_constraints)

        def create_ref_vel_configulator() -> Optional[ReferenceVelocityConfigulator]:
            if self._ref_vel_config_path is None:
                return None
            return ReferenceVelocityConfigulator(self, self._config_path, self._ref_vel_config_path)

        self._map = create_map()
        self._reference_path = create_ref_path(self._map)
        self._car = create_car(self._reference_path)
        self._mpc_cfg, self._mpc = create_mpc(self._car)
        compute_speed_profile(self._car, self._mpc_cfg)

        self._ref_vel_configulator: Optional[ReferenceVelocityConfigulator] = create_ref_vel_configulator()

        self._trajectory: Optional[Trajectory] = None
        self._path_constraints = None
        self._waypoint_xy = np.asarray(
            [(wp.x, wp.y) for wp in self._reference_path.waypoints],
            dtype=np.float64)
        self._waypoint_widths = self._get_waypoint_widths()
        self._nominal_v_ref = self._get_nominal_v_ref()
        self._v2x_tracker: Optional[V2XVehicleTracker] = None
        self._v2x_stop_planner: Optional[V2XStopPlanner] = None
        self._last_v2x_stop_result: Optional[V2XStopResult] = None
        self._last_v2x_stop_log_time = -float("inf")
        self._v2x_overtake_planner: Optional[V2XOvertakePlanner] = None
        self._last_v2x_overtake_result: Optional[V2XOvertakeResult] = None
        self._last_v2x_overtake_log_time = -float("inf")
        self._v2x_overtake_log_throttle_sec = 1.0
        self._v2x_overtake_cfg: Optional[V2XOvertakeConfig] = None
        self._v2x_overtake_mpc_steer_rate: Optional[float] = None
        self._v2x_overtake_constraint_transition_horizon_ratio = 1.0
        self._v2x_overtake_constraint_initial_progress = 0.0
        self._v2x_overtake_standby_offset_m = 0.0
        self._v2x_overtake_standby_side = "right"
        self._v2x_overtake_prepare_speed_cap_mps: Optional[float] = None
        self._v2x_overtake_lateral_ready_threshold_m = 0.0
        self._v2x_overtake_steer_override_enabled = False
        self._v2x_overtake_steer_override_min_abs_rad = 0.0
        self._v2x_overtake_steer_override_until_ey_m = 0.0
        self._last_overtake_steer_override_log_time = -float("inf")
        self._last_overtake_control_fault_log_time = -float("inf")
        self._base_mpc_max_steering_rate = float(self._mpc.max_steering_rate)
        self._overtake_lateral_offset_m = 0.0
        self._last_overtake_constraint_debug = None

        # Obstacles
        if self.USE_OBSTACLE_AVOIDANCE or self.USE_V2X_STOP or self.USE_V2X_OVERTAKE:
            v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
            self._v2x_tracker = V2XVehicleTracker(
                v_max_safety=float(v2x_cfg.v_max_safety),
                position_jump_threshold=float(v2x_cfg.position_jump_threshold),
                warn_callback=self.get_logger().warn,
            )

        if self.USE_OBSTACLE_AVOIDANCE:
            self._static_obstacles: List[Obstacle] = create_obstacles()
            self._dynamic_obstacles: List[Obstacle] = []
            self._obstacles_updated = bool(self._static_obstacles)
            v2x_cfg = self._cfg.v2x_obstacle_avoidance  # type: ignore
            self._v2x_vehicle_radius = float(v2x_cfg.vehicle_radius)
            mpc_N = int(self._cfg.mpc.N)  # type: ignore
            t_horizon = mpc_N / float(self._cfg.mpc.control_rate)  # type: ignore
            self._v2x_t_samples = [
                k * t_horizon / max(mpc_N - 1, 1) for k in range(mpc_N)
            ]
            # コリドー外の V2X 障害物で MPC のコリドー狭窄/反転が起きないよう、
            # ref-path 近傍のみに絞り込む。閾値 = max_width/2 + vehicle_radius + 余白。
            ref_max_width = float(self._cfg.reference_path.max_width)  # type: ignore
            self._v2x_corridor_threshold_sq = (
                ref_max_width / 2.0 + self._v2x_vehicle_radius + 0.5
            ) ** 2

        if self.USE_V2X_STOP:
            if not hasattr(self._cfg, "v2x_stop"):
                self.get_logger().warn("v2x_stop config missing; disabling V2X stop")
                self.USE_V2X_STOP = False
            elif not bool(getattr(self._cfg.v2x_stop, "enabled", True)):  # type: ignore
                self.USE_V2X_STOP = False
            else:
                stop_cfg = V2XStopConfig.from_config(
                    self._cfg.v2x_stop,  # type: ignore
                    a_min_mps2=self._mpc_cfg.a_min)
                self._v2x_stop_planner = V2XStopPlanner(stop_cfg)
                self._v2x_stop_log_throttle_sec = float(
                    getattr(self._cfg.v2x_stop, "log_throttle_sec", 1.0))  # type: ignore

        if self.USE_V2X_OVERTAKE:
            if not hasattr(self._cfg, "v2x_overtake"):
                self.get_logger().warn("v2x_overtake config missing; disabling V2X overtake")
                self.USE_V2X_OVERTAKE = False
            elif not bool(getattr(self._cfg.v2x_overtake, "enabled", True)):  # type: ignore
                self.USE_V2X_OVERTAKE = False
            else:
                overtake_cfg = V2XOvertakeConfig.from_config(
                    self._cfg.v2x_overtake,  # type: ignore
                    vehicle_width_m=float(self._cfg.bicycle_model.width))  # type: ignore
                self._v2x_overtake_cfg = overtake_cfg
                self._v2x_overtake_planner = V2XOvertakePlanner(overtake_cfg)
                self._v2x_overtake_log_throttle_sec = float(
                    getattr(self._cfg.v2x_overtake, "log_throttle_sec", 1.0))  # type: ignore
                raw_steer_rate = float(
                    getattr(self._cfg.v2x_overtake, "overtake_steer_rate_max", 0.0))  # type: ignore
                if raw_steer_rate > 0.0:
                    steering_gain = max(float(self._mpc_cfg.steering_tire_angle_gain_var), 1e-6)
                    self._v2x_overtake_mpc_steer_rate = raw_steer_rate / steering_gain
                transition_ratio = float(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "constraint_transition_horizon_ratio",
                    1.0))
                self._v2x_overtake_constraint_transition_horizon_ratio = float(
                    np.clip(transition_ratio, 0.05, 1.0))
                initial_progress = float(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "constraint_initial_progress",
                    0.0))
                self._v2x_overtake_constraint_initial_progress = float(
                    np.clip(initial_progress, 0.0, 1.0))
                self._v2x_overtake_standby_offset_m = abs(float(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "standby_lateral_offset_m",
                    0.0)))
                standby_side = str(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "standby_side",
                    overtake_cfg.preferred_side)).lower()
                if standby_side not in ("left", "right"):
                    standby_side = overtake_cfg.preferred_side
                self._v2x_overtake_standby_side = standby_side
                prepare_speed_cap_kmph = float(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "prepare_speed_cap_kmph",
                    0.0))
                if prepare_speed_cap_kmph > 0.0:
                    self._v2x_overtake_prepare_speed_cap_mps = kmh_to_m_per_sec(
                        prepare_speed_cap_kmph)
                self._v2x_overtake_lateral_ready_threshold_m = float(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "lateral_ready_threshold_m",
                    0.0))
                self._v2x_overtake_steer_override_enabled = bool(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "steer_override_enabled",
                    False))
                self._v2x_overtake_steer_override_min_abs_rad = abs(float(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "steer_override_min_abs_rad",
                    0.0)))
                self._v2x_overtake_steer_override_until_ey_m = float(getattr(
                    self._cfg.v2x_overtake,  # type: ignore
                    "steer_override_until_ey_m",
                    self._v2x_overtake_lateral_ready_threshold_m))

        # Laps
        self._current_laps = 1
        self._last_lap_time = 0.0
        self._lap_times = [None] * (self.MAX_LAPS + 1) # +1 means include lap 0

        # condition
        self._last_condition = None
        self._last_colliding_time = None

        # stats
        self._stats = ExecutionStats(self.get_logger(), window_size=50, record_count_threshold=1000)

        # save config
        if self._cfg.common.save_config:
            self._save_config()

    def _save_config(self) -> None:
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        dst_dir = self.PKG_PATH + f"log/{now}"
        os.makedirs(dst_dir, exist_ok=True)
        shutil.copy(self._config_path, os.path.join(dst_dir, "config.yaml"))

    def _setup_pub_sub(self) -> None:
        # Publishers
        if self.USE_BUG_ACC:
          self._command_pub = self.create_publisher(
            AckermannControlBoostCommand, "/boost_commander/command", 1)
        else:
          self._command_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1)
          self._command_raw_pub = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd_raw", 1)
          print("use normal ackermann control command")

        # NOTE:評価環境での可視化のためにダミーのトピック名を使用
        self._mpc_pred_pub = self.create_publisher(
            MarkerArray, "/mpc/prediction", 1)
        self._mpc_pred_pub_dummy = self.create_publisher(
            MarkerArray, "/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/virtual_wall", 1)

        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        # NOTE:評価環境での可視化のためにダミーのトピック名を使用
        self._ref_path_pub = self.create_publisher(
            MarkerArray, "/mpc/ref_path", latching_qos)
        self._ref_path_pub_dummy = self.create_publisher(
            MarkerArray, "/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound", latching_qos)

        # Subscribers
        self._odom_sub = self.create_subscription(
            Odometry, "/localization/kinematic_state", self._odom_callback, 1)
        self._control_mode_request_sub = self.create_subscription(
            Bool, "control/control_mode_request_topic", self._control_mode_request_callback, 1)
        # simple_trajectory_generator publishes with BEST_EFFORT/KEEP_LAST(1) — match it
        # so the subscription is QoS-compatible (rclpy default is RELIABLE).
        trajectory_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._trajectory_sub = self.create_subscription(
            Trajectory, "planning/scenario_planning/trajectory", self._trajectory_callback, trajectory_qos)
        self._stop_request_sub = self.create_subscription(
            Empty, "/control/mpc/stop_request", self._stop_request_callback, 1)

        if self.use_sim_time:
            self._awsim_status_sub = self.create_subscription(
                Float32MultiArray, "/awsim/status", self._awsim_status_callback, 1)
            self._condition_sub = self.create_subscription(
                Int32, "/aichallenge/pitstop/condition", self._condition_callback, 1)

        if self.USE_OBSTACLE_AVOIDANCE:
            if self._cfg.reference_path.use_path_constraints_topic: # type: ignore
                self._path_constraints_sub = self.create_subscription(
                    PathConstraints, "/path_constraints_provider/path_constraints", self._path_constraints_callback, 1)

            if self._cfg.reference_path.use_border_cells_topic: # type: ignore
                self._border_cells_sub = self.create_subscription(
                    BorderCells, "/path_constraints_provider/border_cells", self._border_cells_callback, 1)

        if self.USE_OBSTACLE_AVOIDANCE or self.USE_V2X_STOP or self.USE_V2X_OVERTAKE:
            self._v2x_sub = self.create_subscription(
                V2XVehiclePositionArray,
                "/v2x/vehicle_positions",
                self._v2x_callback,
                1)

    def _create_ackerman_control_command(self, stamp, u, acc, bug_acc_enabled):
        v_cmd = u[0]
        steer_cmd = u[1]

        ackerman_cmd = array_to_ackermann_control_command(stamp.to_msg(), [v_cmd, steer_cmd], acc)

        if not self.USE_BUG_ACC:
            return ackerman_cmd

        ackerman_boost_cmd = AckermannControlBoostCommand()
        ackerman_boost_cmd.command = ackerman_cmd
        ackerman_boost_cmd.boost_mode = bug_acc_enabled
        return ackerman_boost_cmd

    def _publish_control_command(self, stamp, u, acc, bug_acc_enabled):
        cmd = self._create_ackerman_control_command(stamp, u, acc, bug_acc_enabled)

        # publish raw control command
        self._command_raw_pub.publish(cmd)

        # compensate steering angle for the real vehicle
        # AWSIMにおいても後段のactuation_cmd_converter でgainを考慮した指令を生成するため、実機/sim問わず
        # gain を掛ける
        cmd.lateral.steering_tire_angle *= self._mpc_cfg.steering_tire_angle_gain_var
        self._command_pub.publish(cmd)


    def _odom_callback(self, msg: Odometry) -> None:
        self._odom = msg

    def _control_mode_request_callback(self, msg):
        if msg.data and not self._enable_control:
            self.get_logger().info("Control mode request received")
            self._enable_control = True

    def _path_constraints_callback(self, msg: PathConstraints):
        self._reference_path.set_path_constraints(
            msg.upper_bounds, msg.lower_bounds, msg.rows, msg.cols)

    def _v2x_callback(self, msg: V2XVehiclePositionArray) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self._v2x_tracker is not None:
            self._v2x_tracker.update(msg)

        if self._v2x_stop_planner is not None:
            self._v2x_stop_planner.update_v2x(msg, now_sec=now_sec)

        if self._v2x_overtake_planner is not None:
            self._v2x_overtake_planner.update_v2x(msg, now_sec=now_sec)

        if self.USE_OBSTACLE_AVOIDANCE and self._v2x_tracker is not None:
            predictions = self._v2x_tracker.predict_all(self._v2x_t_samples)
            self._dynamic_obstacles = predictions_to_obstacles(
                predictions, self._v2x_vehicle_radius)
            self._obstacles_updated = True

    def _filter_obstacles_to_corridor(self, obstacles: List[Obstacle]) -> List[Obstacle]:
        if not obstacles or self._waypoint_xy.size == 0:
            return obstacles
        thr_sq = self._v2x_corridor_threshold_sq
        wps = self._waypoint_xy
        kept: List[Obstacle] = []
        for ob in obstacles:
            dxy = wps - np.array([ob.cx, ob.cy], dtype=np.float64)
            if np.min(np.einsum('ij,ij->i', dxy, dxy)) <= thr_sq:
                kept.append(ob)
        return kept

    def _border_cells_callback(self, msg: BorderCells):
        self._reference_path.set_border_cells(
            msg.dynamic_upper_bounds, msg.dynamic_lower_bounds, msg.rows, msg.cols)

    def _trajectory_callback(self, msg):
        self._trajectory = msg

    def _awsim_status_callback(self, msg):
        laps = int(msg.data[1])
        lap_time = msg.data[2]
        # section = int(msg.data[3])

        if self._current_laps is None:
            self._current_laps = 1 if laps == 0 else laps

        if laps > self._current_laps:
            self.get_logger().info(f'\033[32mLap {self._current_laps} completed! Lap time: {self._last_lap_time} s\033[0m')
            self._lap_times[self._current_laps] = self._last_lap_time
            self._current_laps = laps

        self._last_lap_time = lap_time

    def _condition_callback(self, msg: Int32):
        if self._last_condition is None:
            self._last_condition = msg.data

        diff_condition = msg.data - self._last_condition
        if diff_condition > 30.0:
            self._last_colliding_time = self.get_clock().now()
            self.get_logger().warning(f"Collision detected!")
        self._last_condition = msg.data

    def _stop_request_callback(self, msg: Empty) -> None:
        if self._enable_control:
            self.get_logger().warn(f"Stop request received {self._enable_control}")
            self._enable_control = False

    def _wait_until_clock_received(self) -> None:
        if self.use_sim_time:
            self.get_logger().info(f"wait until clock received...")
            rate = self.create_rate(10)
            rate.sleep()
            self.get_logger().info(f">> OK!")

    def _wait_until_message_received(self, message_getter, message_name: str, timeout: float, rate_hz: int = 30) -> None:

        t_start = self.get_clock().now()
        rate = self.create_rate(rate_hz)

        self.get_logger().info(f"wait until {message_name} received...")

        while message_getter() is None:
            now = self.get_clock().now()
            if (now - t_start).nanoseconds > timeout * 1e9:
                self.get_logger().info(f"now: {now}, t_start: {t_start}")
                raise TimeoutError(f"Timeout while waiting for {message_name} message")
            rate.sleep()

        self.get_logger().info(f">> OK!")

    def _wait_until_odom_received(self, timeout: float = 30.) -> None:
        self._wait_until_message_received(lambda: self._odom, 'odometry', timeout)

    def _wait_until_trajectory_received(self, timeout: float = 30.) -> None:
        if self._cfg.reference_path.update_by_topic:
            self._wait_until_message_received(lambda: self._trajectory, 'trajectory', timeout)

    def _wait_until_path_constraints_received(self, timeout: float = 30.) -> None:
        if self.USE_OBSTACLE_AVOIDANCE and self._cfg.reference_path.use_path_constraints_topic: # type: ignore
            self._wait_until_message_received(lambda: self._reference_path.path_constraints, 'path constraints', timeout)

    def _publish_mpc_pred_marker(self, x_pred, y_pred):
        pred_marker_array = MarkerArray()
        m_base = Marker()
        m_base.header.frame_id = "map"
        m_base.ns = "mpc_pred"
        m_base.type = Marker.SPHERE
        m_base.action = Marker.ADD
        m_base.pose.position.z = 0.0
        m_base.scale = Vector3(x=0.5, y=0.5, z=0.5)
        m_base.color = self._pred_marker_color
        for i in range(len(x_pred)):
            m = copy.deepcopy(m_base)
            m.id = i
            m.pose.position.x = x_pred[i]
            m.pose.position.y = y_pred[i]
            pred_marker_array.markers.append(m) # type: ignore
        self._mpc_pred_pub.publish(pred_marker_array)
        self._mpc_pred_pub_dummy.publish(pred_marker_array)

    def _publish_ref_path_marker(self, ref_path: ReferencePath):
        WP_SPHERE_ENABLED = False

        ref_path_marker_array = MarkerArray()

        m_base = Marker()
        m_base.header.frame_id = "map"
        m_base.ns = "ref_path"
        m_base.type = Marker.LINE_STRIP
        m_base.action = Marker.ADD
        m_base.pose.position.z = 0.0
        m_base.scale.x = 0.2
        m_base.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)

        for i in range(len(ref_path.waypoints) - 1):
            m = copy.deepcopy(m_base)
            m.id = i
            start = Point()
            start.x = ref_path.waypoints[i].x
            start.y = ref_path.waypoints[i].y
            end = Point()
            end.x = ref_path.waypoints[i + 1].x
            end.y = ref_path.waypoints[i + 1].y
            m.points.append(start) # type: ignore
            m.points.append(end) # type: ignore
            ref_path_marker_array.markers.append(m) # type: ignore

        if WP_SPHERE_ENABLED:
            spheres = Marker()
            spheres.header.frame_id = "map"
            spheres.ns = "ref_path_point"
            spheres.type = Marker.SPHERE_LIST
            spheres.action = Marker.ADD
            radius = 0.2
            spheres.scale = Vector3(x=radius, y=radius, z=radius)
            spheres.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7)
            for i in range(len(ref_path.waypoints) - 1):
                p = Point()
                p.x = ref_path.waypoints[i].x
                p.y = ref_path.waypoints[i].y
                p.z = 0.
                spheres.points.append(p) #type: ignore
            ref_path_marker_array.markers.append(spheres) # type: ignore

        self._ref_path_pub.publish(ref_path_marker_array)
        self._ref_path_pub_dummy.publish(ref_path_marker_array)

    def _get_nominal_v_ref(self) -> List[float]:
        return [
            float(wp.v_ref) if wp.v_ref is not None else self._configured_v_max_mps()
            for wp in self._reference_path.waypoints
        ]

    def _configured_v_max_mps(self) -> float:
        return max(0.0, float(self._mpc_cfg.v_max))

    def _get_waypoint_widths(self) -> np.ndarray:
        fallback_half_width = float(self._cfg.reference_path.max_width) / 2.0  # type: ignore

        def width_or_fallback(value: Optional[float], fallback: float) -> float:
            if value is None:
                return fallback
            try:
                return max(abs(float(value)), 0.0)
            except (TypeError, ValueError):
                return fallback

        return np.asarray(
            [
                (
                    width_or_fallback(wp.ub, fallback_half_width),
                    width_or_fallback(wp.lb, fallback_half_width),
                )
                for wp in self._reference_path.waypoints
            ],
            dtype=np.float64,
        )

    def _apply_speed_limits(self, pose: Pose2D, ego_v_mps: float, now_sec: float, dt: float) -> None:
        configured_v_max_mps = self._configured_v_max_mps()
        base_v_mps = configured_v_max_mps
        has_ref_vel_config = self._ref_vel_configulator is not None
        if self._ref_vel_configulator is not None:
            ref_vel_kmph = self._ref_vel_configulator.get_ref_vel(self._mpc.model.wp_id)
            base_v_mps = min(kmh_to_m_per_sec(ref_vel_kmph), configured_v_max_mps)

        effective_v_mps = base_v_mps
        overtake_bypasses_stop = False
        overtake_steer_active = False
        overtake_result: Optional[V2XOvertakeResult] = None
        overtake_target_offset_m = self._overtake_standby_offset()
        if self.USE_V2X_OVERTAKE and self._v2x_overtake_planner is not None:
            result = self._v2x_overtake_planner.compute_behavior(
                ego_x=pose.x,
                ego_y=pose.y,
                ego_yaw=pose.theta,
                ego_v_mps=ego_v_mps,
                reference_xy=self._waypoint_xy,
                reference_widths=self._waypoint_widths,
                now_sec=now_sec,
                base_speed_mps=base_v_mps,
                current_lateral_offset_m=self._current_path_lateral_offset_m(),
                velocity_lookup=(
                    self._v2x_tracker.velocity
                    if self._v2x_tracker is not None
                    else None
                ),
            )
            self._last_v2x_overtake_result = result
            overtake_result = result

            if result.active:
                overtake_target_offset_m = result.target_lateral_offset_m
            effective_v_mps = min(effective_v_mps, result.speed_cap_mps)
            overtake_bypasses_stop = result.state in (
                "prepare_overtake",
                "overtaking",
                "return_to_line",
            )
            if overtake_bypasses_stop:
                effective_v_mps = self._limit_overtake_prepare_speed(
                    effective_v_mps,
                    result,
                )
            overtake_steer_active = (
                overtake_bypasses_stop
                or abs(overtake_target_offset_m) > 0.05
                or abs(self._overtake_lateral_offset_m) > 0.05
            )
        else:
            overtake_target_offset_m = 0.0

        if self.USE_V2X_OVERTAKE:
            self._update_overtake_lateral_offset(overtake_target_offset_m, dt)
            self._apply_overtake_steer_rate(overtake_steer_active)
            if (
                overtake_result is not None
                or overtake_steer_active
                or abs(self._overtake_lateral_offset_m) > 1e-3
            ):
                self._apply_overtake_lateral_constraints(self._overtake_lateral_offset_m)
            else:
                self._last_overtake_constraint_debug = None
        else:
            self._overtake_lateral_offset_m = 0.0
            self._apply_overtake_steer_rate(False)
            self._last_overtake_constraint_debug = None
        if overtake_result is not None:
            self._log_v2x_overtake_result(overtake_result, now_sec)

        if (
            not overtake_bypasses_stop
            and self.USE_V2X_STOP
            and self._v2x_stop_planner is not None
        ):
            result = self._v2x_stop_planner.compute_speed_cap(
                ego_x=pose.x,
                ego_y=pose.y,
                ego_yaw=pose.theta,
                ego_v_mps=ego_v_mps,
                reference_xy=self._waypoint_xy,
                now_sec=now_sec,
                base_speed_mps=base_v_mps,
                velocity_lookup=(
                    self._v2x_tracker.velocity
                    if self._v2x_tracker is not None
                    else None
                ),
            )
            self._last_v2x_stop_result = result
            effective_v_mps = min(effective_v_mps, result.speed_cap_mps)
            self._log_v2x_stop_result(result, now_sec)

        effective_v_mps = float(np.clip(effective_v_mps, 0.0, configured_v_max_mps))
        self._mpc.update_v_max(effective_v_mps)
        if has_ref_vel_config:
            v_ref: List[float] = [effective_v_mps] * len(self._reference_path.waypoints)
        else:
            v_ref = [min(v, effective_v_mps) for v in self._nominal_v_ref]
        self._reference_path.set_v_ref(v_ref)

    def _apply_overtake_steer_rate(self, active: bool) -> None:
        steer_rate = self._base_mpc_max_steering_rate
        if active and self._v2x_overtake_mpc_steer_rate is not None:
            steer_rate = max(steer_rate, self._v2x_overtake_mpc_steer_rate)
        self._mpc.max_steering_rate = steer_rate

    def _overtake_standby_offset(self) -> float:
        if not self.USE_V2X_OVERTAKE or self._v2x_overtake_cfg is None:
            return 0.0
        if self._v2x_overtake_standby_offset_m <= 1e-3:
            return 0.0

        side = self._v2x_overtake_standby_side
        sign = 1.0 if side == "left" else -1.0
        offset_m = sign * self._v2x_overtake_standby_offset_m
        if not self._is_overtake_offset_wall_safe(offset_m):
            return 0.0
        return offset_m

    def _is_overtake_offset_wall_safe(self, offset_m: float) -> bool:
        if self._v2x_overtake_cfg is None:
            return False
        if abs(offset_m) <= 1e-3 or self._waypoint_widths.size == 0:
            return False

        side_idx = 0 if offset_m > 0.0 else 1
        n_widths = len(self._waypoint_widths)
        start_idx = int(self._car.wp_id) % n_widths
        horizon_count = max(1, int(np.ceil(
            self._v2x_overtake_cfg.wall_check_horizon_m
            / max(float(self._cfg.reference_path.resolution), 1e-3))))  # type: ignore
        indices = [(start_idx + i) % n_widths for i in range(min(horizon_count, n_widths))]
        available_m = min(float(self._waypoint_widths[i][side_idx]) for i in indices)
        required_m = (
            abs(offset_m)
            + float(self._cfg.bicycle_model.width) * 0.5  # type: ignore
            + self._v2x_overtake_cfg.wall_safety_margin_m
            + self._v2x_overtake_cfg.min_wall_clearance_m
        )
        return available_m >= required_m

    def _limit_overtake_prepare_speed(
        self,
        effective_v_mps: float,
        result: V2XOvertakeResult,
    ) -> float:
        if self._v2x_overtake_prepare_speed_cap_mps is None:
            return effective_v_mps
        if result.side not in ("left", "right"):
            return effective_v_mps

        spatial_state = self._car.t2s(
            reference_waypoint=self._reference_path.get_waypoint(self._car.wp_id),
            reference_state=self._car.temporal_state,
        )
        current_ey = float(spatial_state.e_y)
        threshold = abs(self._v2x_overtake_lateral_ready_threshold_m)
        if result.side == "right" and current_ey > -threshold:
            return min(effective_v_mps, self._v2x_overtake_prepare_speed_cap_mps)
        if result.side == "left" and current_ey < threshold:
            return min(effective_v_mps, self._v2x_overtake_prepare_speed_cap_mps)
        return effective_v_mps

    def _current_path_lateral_offset_m(self) -> float:
        spatial_state = self._car.t2s(
            reference_waypoint=self._reference_path.get_waypoint(self._car.wp_id),
            reference_state=self._car.temporal_state,
        )
        return float(spatial_state.e_y)

    def _apply_overtake_steer_override(self, u: np.ndarray, now_sec: float) -> np.ndarray:
        if not self._v2x_overtake_steer_override_enabled:
            return u
        if self._v2x_overtake_steer_override_min_abs_rad <= 1e-3:
            return u

        result = self._last_v2x_overtake_result
        if result is None or result.state not in ("prepare_overtake", "overtaking"):
            return u
        if result.side not in ("left", "right"):
            return u

        spatial_state = self._car.t2s(
            reference_waypoint=self._reference_path.get_waypoint(self._car.wp_id),
            reference_state=self._car.temporal_state,
        )
        current_ey = float(spatial_state.e_y)
        steer_abs = min(
            self._v2x_overtake_steer_override_min_abs_rad,
            float(self._mpc_cfg.delta_max),
        )

        applied = False
        original_steer = float(u[1])
        until_ey = abs(self._v2x_overtake_steer_override_until_ey_m)
        signed_until_ey = -until_ey if result.side == "right" else until_ey
        if result.side == "right" and current_ey > -until_ey:
            u[1] = min(float(u[1]), -steer_abs)
            applied = True
        elif result.side == "left" and current_ey < until_ey:
            u[1] = max(float(u[1]), steer_abs)
            applied = True

        if applied and now_sec - self._last_overtake_steer_override_log_time >= 1.0:
            self._last_overtake_steer_override_log_time = now_sec
            self.get_logger().info(
                "V2X overtake steer override: "
                f"side={result.side} ey={current_ey:.2f}m "
                f"raw_steer={original_steer:.3f}rad "
                f"cmd_steer={float(u[1]):.3f}rad "
                f"until_ey={signed_until_ey:.2f}m")

        return u

    def _update_overtake_lateral_offset(self, target_offset_m: float, dt: float) -> None:
        if self._v2x_overtake_cfg is None:
            self._overtake_lateral_offset_m = 0.0
            return
        dt = max(float(dt), 0.0)
        rate_mps = float(self._v2x_overtake_cfg.lateral_offset_rate_mps)
        if dt <= 0.0 or rate_mps <= 0.0:
            return
        max_step = rate_mps * dt
        diff = target_offset_m - self._overtake_lateral_offset_m
        if abs(diff) <= max_step:
            self._overtake_lateral_offset_m = float(target_offset_m)
        else:
            self._overtake_lateral_offset_m += float(np.sign(diff) * max_step)

        max_offset = abs(self._v2x_overtake_cfg.lateral_offset_m)
        self._overtake_lateral_offset_m = float(np.clip(
            self._overtake_lateral_offset_m,
            -max_offset,
            max_offset,
        ))

    def _apply_overtake_lateral_constraints(self, offset_m: float) -> None:
        if self._reference_path.path_constraints is None:
            return
        if self._reference_path.path_constraints[0].size == 0:
            return

        upper_constraints = self._reference_path.path_constraints[0]
        lower_constraints = self._reference_path.path_constraints[1]
        n_rows, n_cols = upper_constraints.shape
        row_idx = (
            int(self._car.wp_id)
            + int(self._mpc_cfg.wp_id_offset)
            + 1
        ) % n_rows

        if self._v2x_overtake_cfg is not None:
            half_width = self._v2x_overtake_cfg.constraint_half_width_m
        else:
            half_width = 0.55

        upper_bounds = []
        lower_bounds = []
        upper_cells = []
        lower_cells = []
        transition_ratio = self._v2x_overtake_constraint_transition_horizon_ratio
        transition_cols = max(1, int(np.ceil(float(max(n_cols, 1)) * transition_ratio)))
        initial_progress = self._v2x_overtake_constraint_initial_progress
        current_spatial_state = self._car.t2s(
            reference_waypoint=self._reference_path.get_waypoint(self._car.wp_id),
            reference_state=self._car.temporal_state,
        )
        current_ey = float(current_spatial_state.e_y)

        for n in range(n_cols):
            wp = self._reference_path.get_waypoint(row_idx + n)
            ub_sm = float(wp.ub) - self._car.safety_margin
            lb_sm = float(wp.lb) + self._car.safety_margin
            if ub_sm < lb_sm:
                ub_sm = 0.0
                lb_sm = 0.0

            if abs(offset_m) > 1e-3:
                progress = max(
                    initial_progress,
                    min(1.0, float(n + 1) / float(transition_cols)))
                center = current_ey + progress * (offset_m - current_ey)
                upper = min(ub_sm, center + half_width)
                lower = max(lb_sm, center - half_width)
                if offset_m > 0.0 and center >= 0.0:
                    lower = max(lower, 0.0)
                elif offset_m < 0.0 and center <= 0.0:
                    upper = min(upper, 0.0)
                if upper < lower:
                    upper = ub_sm
                    lower = lb_sm
            else:
                upper = ub_sm
                lower = lb_sm

            upper_bounds.append(upper)
            lower_bounds.append(lower)

            angle_ub = np.mod(np.pi / 2 + wp.psi + np.pi, 2 * np.pi) - np.pi
            angle_lb = np.mod(-np.pi / 2 + wp.psi + np.pi, 2 * np.pi) - np.pi
            upper_cells.append((
                wp.x + upper * np.cos(angle_ub),
                wp.y + upper * np.sin(angle_ub),
            ))
            lower_cells.append((
                wp.x - lower * np.cos(angle_lb),
                wp.y - lower * np.sin(angle_lb),
            ))

        upper_constraints[row_idx] = np.asarray(upper_bounds)
        lower_constraints[row_idx] = np.asarray(lower_bounds)
        self._last_overtake_constraint_debug = {
            "row": row_idx,
            "upper0": float(upper_bounds[0]) if upper_bounds else None,
            "lower0": float(lower_bounds[0]) if lower_bounds else None,
            "center0": float((upper_bounds[0] + lower_bounds[0]) / 2.0) if upper_bounds else None,
            "upperN": float(upper_bounds[-1]) if upper_bounds else None,
            "lowerN": float(lower_bounds[-1]) if lower_bounds else None,
            "centerN": float((upper_bounds[-1] + lower_bounds[-1]) / 2.0) if upper_bounds else None,
            "ego_ey": current_ey,
            "transition_cols": transition_cols,
            "initial_progress": initial_progress,
        }

        try:
            self._reference_path.border_cells.dynamic_upper_bounds[row_idx] = np.asarray(upper_cells)
            self._reference_path.border_cells.dynamic_lower_bounds[row_idx] = np.asarray(lower_cells)
        except (AttributeError, IndexError, TypeError):
            pass

    def _abort_overtake_on_control_fault(
        self,
        u,
        max_delta: float,
        now_sec: float,
    ) -> bool:
        result = self._last_v2x_overtake_result
        if (
            not self.USE_V2X_OVERTAKE
            or self._v2x_overtake_planner is None
            or result is None
            or result.state not in ("prepare_overtake", "overtaking", "return_to_line")
        ):
            return False

        invalid_control = False
        try:
            u_array = np.asarray(u, dtype=float)
            invalid_control = (
                u_array.size < 2
                or not np.all(np.isfinite(u_array))
                or not np.isfinite(float(max_delta))
            )
        except (TypeError, ValueError):
            invalid_control = True

        infeasible = int(getattr(self._mpc, "infeasibility_counter", 0)) > 0
        if not invalid_control and not infeasible:
            return False

        reason = "mpc_control_fault" if invalid_control else "mpc_infeasible"
        self._v2x_overtake_planner.force_abort(reason)
        self._last_v2x_overtake_result = V2XOvertakeResult(
            active=True,
            state="abort",
            speed_cap_mps=0.0,
            target_lateral_offset_m=0.0,
            reason=reason,
            vehicle_id=result.vehicle_id,
            side=result.side,
            gap_m=result.gap_m,
            signed_gap_m=result.signed_gap_m,
            lateral_offset_m=result.lateral_offset_m,
            relative_speed_mps=result.relative_speed_mps,
            target_speed_mps=result.target_speed_mps,
            left_wall_margin_m=result.left_wall_margin_m,
            right_wall_margin_m=result.right_wall_margin_m,
        )
        self._overtake_lateral_offset_m = 0.0
        self._apply_overtake_steer_rate(False)
        self._last_overtake_constraint_debug = None

        if now_sec - self._last_overtake_control_fault_log_time >= 1.0:
            self._last_overtake_control_fault_log_time = now_sec
            self.get_logger().warn(f"V2X overtake abort: {reason}")
        return True

    def _log_v2x_stop_result(self, result: V2XStopResult, now_sec: float) -> None:
        throttle_sec = getattr(self, "_v2x_stop_log_throttle_sec", 1.0)
        if now_sec - self._last_v2x_stop_log_time < throttle_sec:
            return
        self._last_v2x_stop_log_time = now_sec

        if result.active:
            self.get_logger().info(
                "V2X stop: "
                f"{result.reason} target={result.vehicle_id} "
                f"gap={result.gap_m:.2f}m "
                f"lat={result.lateral_offset_m:.2f}m "
                f"v_cap={result.speed_cap_mps:.2f}m/s")
        else:
            self.get_logger().info("V2X stop: clear")

    def _log_v2x_overtake_result(self, result: V2XOvertakeResult, now_sec: float) -> None:
        throttle_sec = getattr(self, "_v2x_overtake_log_throttle_sec", 1.0)
        if now_sec - self._last_v2x_overtake_log_time < throttle_sec:
            return
        self._last_v2x_overtake_log_time = now_sec

        def fmt(value: Optional[float], unit: str = "") -> str:
            if value is None:
                return f"--{unit}"
            return f"{value:.2f}{unit}"

        if result.active:
            constraint_debug = self._last_overtake_constraint_debug or {}
            self.get_logger().info(
                "V2X overtake: "
                f"{result.state} reason={result.reason} "
                f"target={result.vehicle_id} side={result.side or '--'} "
                f"gap={fmt(result.gap_m, 'm')} "
                f"signed_gap={fmt(result.signed_gap_m, 'm')} "
                f"lat={fmt(result.lateral_offset_m, 'm')} "
                f"offset={result.target_lateral_offset_m:.2f}m "
                f"offset_cmd={self._overtake_lateral_offset_m:.2f}m "
                f"v_cap={result.speed_cap_mps:.2f}m/s "
                f"wall_l={fmt(result.left_wall_margin_m, 'm')} "
                f"wall_r={fmt(result.right_wall_margin_m, 'm')} "
                f"ey={fmt(constraint_debug.get('ego_ey'), 'm')} "
                f"mpc_steer_rate={self._mpc.max_steering_rate:.2f}rad/s "
                f"transition_cols={constraint_debug.get('transition_cols', '--')} "
                f"initial_progress={constraint_debug.get('initial_progress', '--')} "
                f"corridor0=[{fmt(constraint_debug.get('lower0'), 'm')},"
                f"{fmt(constraint_debug.get('upper0'), 'm')}] "
                f"center0={fmt(constraint_debug.get('center0'), 'm')} "
                f"corridorN=[{fmt(constraint_debug.get('lowerN'), 'm')},"
                f"{fmt(constraint_debug.get('upperN'), 'm')}] "
                f"centerN={fmt(constraint_debug.get('centerN'), 'm')}")
        else:
            self.get_logger().info("V2X overtake: clear")

    def _control(self):
        now = self.get_clock().now()
        t = (now - self._t_start).nanoseconds / 1e9
        dt = (now - self._last_t).nanoseconds / 1e9

        self._last_t = now
        self._loop += 1

        # record and print execution stats
        if self.use_stats:
            self._stats.record()

        # self.get_logger().info("loop")
        self._control_rate.sleep()

        if self._loop % 100 == 0:
            # update reference path
            if self._cfg.reference_path.update_by_topic: # type: ignore
                new_referece_path = self._create_reference_path_from_autoware_trajectory(self._trajectory)
                if new_referece_path is not None:
                    self._car.reference_path = new_referece_path
                    self._car.update_reference_path(self._car.reference_path)
                    self._reference_path = new_referece_path
                    self._waypoint_xy = np.asarray(
                        [(wp.x, wp.y) for wp in self._reference_path.waypoints],
                        dtype=np.float64)
                    self._waypoint_widths = self._get_waypoint_widths()
                    self._nominal_v_ref = self._get_nominal_v_ref()

            def plot_reference_path(car):
                import matplotlib.pyplot as plt
                import sys
                fig, ax = plt.subplots(1, 1)
                car.reference_path.show(ax)
                plt.show()
                sys.exit(1)
            # plot_reference_path(self._car)

        if self.USE_OBSTACLE_AVOIDANCE and self._obstacles_updated:
            self._obstacles_updated = False
            self._map.reset_map()
            filtered_dynamic = self._filter_obstacles_to_corridor(self._dynamic_obstacles)
            self._map.add_obstacles(self._static_obstacles + filtered_dynamic)
            self._reference_path.reset_dynamic_constraints()

        is_colliding = False
        if self._last_colliding_time is not None:
            elapsed_from_last_colliding = (now - self._last_colliding_time).nanoseconds / 1e9
            if elapsed_from_last_colliding < 5.0:
                is_colliding = True

        pose = odom_to_pose_2d(self._odom) # type: ignore
        v = self._odom.twist.twist.linear.x

        self._car.update_states(pose.x, pose.y, pose.theta)
        # print(f"car x: {self._car.temporal_state.x}, y: {self._car.temporal_state.y}, psi: {self._car.temporal_state.psi}")
        # print(f"mpc x: {self._mpc.model.temporal_state.x}, y: {self._mpc.model.temporal_state.y}, psi: {self._mpc.model.temporal_state.psi}")

        self._apply_speed_limits(pose, v, now.nanoseconds / 1e9, dt)

        with self._stats.time_block("control"):
            u, max_delta = self._mpc.get_control()
            # self.get_logger().info(f"u: {u}")

        if self._abort_overtake_on_control_fault(u, max_delta, now.nanoseconds / 1e9):
            u = np.array([0.0, 0.0])
            max_delta = 0.0

        # override by brake command if control is disabled
        if not self._enable_control:
            last_v_cmd = self._last_u[0]
            if last_v_cmd < 0.5:
                u[0] = 0.0
            else:
                decel_v = last_v_cmd + self._mpc_cfg.a_min * dt
                u[0] = np.clip(decel_v, 0.0, self._configured_v_max_mps())

        if len(u) == 0:
            self.get_logger().error("No control signal", throttle_duration_sec=1)
            u = [0.0, 0.0]
            # continue

        acc = 0.
        bug_acc_enabled = False
        if self.USE_BUG_ACC:
            def deg2rad(deg):
                return deg * np.pi / 180.0

            if abs(v) > kmh_to_m_per_sec(44.0) or \
             (abs(v) > kmh_to_m_per_sec(38.0) and abs(max_delta) > deg2rad(12.0)):
                bug_acc_enabled = False
                acc = self._mpc_cfg.a_min / 3.0 * 2.0
                self._pred_marker_color = RED
            elif abs(v) > kmh_to_m_per_sec(41.0) or abs(u[1]) > deg2rad(10.0):
                bug_acc_enabled = False
                acc = self._mpc_cfg.a_max
                self._pred_marker_color = YELLOW
            else:
                bug_acc_enabled = True
                acc = 500.0
                self._pred_marker_color = CYAN
        else:
            acc =  self.KP * (u[0] - v)
            # print(f"v: {v}, u[0]: {u[0]}, acc: {acc}")
            acc = np.clip(acc, self._mpc_cfg.a_min, self._mpc_cfg.a_max)
        # u[0] = np.clip(last_u[0] + acc * dt, 0.0, self._mpc_cfg.v_max)

        # apply low pass filter to control signal
        acc = self._last_acc + (acc - self._last_acc) * self._mpc_cfg.accel_low_pass_gain
        u[1] = self._last_u[1] + (u[1] - self._last_u[1]) * self._mpc_cfg.steer_low_pass_gain
        u = self._apply_overtake_steer_override(u, now.nanoseconds / 1e9)

        self._last_acc = acc
        self._last_u[0] = u[0]
        self._last_u[1] = u[1]

        # update car state (use v for feedback actual speed)
        self._car.drive([v, u[1]])

        # Publish control command
        self._publish_control_command(now, u, acc, bug_acc_enabled)

        # Log states
        self._sim_logger.log(self._car, u, t)
        self._sim_logger.plot_animation(t, self._loop, self._current_laps, self._lap_times, is_colliding, u, self._mpc, self._car)

        # 約 0.25 秒ごとに予測結果を表示
        if (self._mpc.current_prediction is not None) and (self._loop % (self._mpc_cfg.control_rate // 4) == 0):
            self._publish_mpc_pred_marker(self._mpc.current_prediction[0], self._mpc.current_prediction[1]) # type: ignore

    def run(self) -> None:
        self._wait_until_clock_received()
        self._wait_until_odom_received()
        self._wait_until_trajectory_received()
        self._wait_until_path_constraints_received()

        # initialize car states
        pose = odom_to_pose_2d(self._odom) # type: ignore
        self._car.update_states(pose.x, pose.y, pose.theta)
        self._car.update_reference_path(self._car.reference_path)

        if self._ref_vel_configulator is None:
            self._publish_ref_path_marker(self._car.reference_path)

        self._pred_marker_color = CYAN

        # for i in range(10):
        #     self._obstacle_manager.push_next_obstacle()

        # initialize control states
        self._control_rate = self.create_rate(self._mpc_cfg.control_rate)
        self._sim_logger = SimulationLogger(
            self.get_logger(),
            self._car.temporal_state.x, self._car.temporal_state.y, self._cfg.sim_logger.animation_enabled, self.SHOW_PLOT_ANIMATION, self.PLOT_RESULTS, self.ANIMATION_INTERVAL) # type: ignore

        self._loop = 0
        self._last_acc = 0.0
        self._last_u = np.array([0.0, 0.0])
        self._t_start = self.get_clock().now()
        self._last_t = self._t_start

        self.get_logger().info("----------------------")
        self.get_logger().info("START!")
        self.get_logger().info("----------------------")

        while rclpy.ok() and (not self._sim_logger.stop_requested()):
            self._control()

    def stop(self):
        # Wait for stopping
        self.get_logger().warn("----------------------")
        self.get_logger().warn("Stopping...")
        self.get_logger().warn("----------------------")
        timeout_time = self.get_clock().now() + rclpy.time.Duration(seconds=5)
        while self._odom.twist.twist.linear.x > 0.1 and self.get_clock().now() < timeout_time:
            self._enable_control = False
            self._control()

        # Publish zero command to stop the car completely
        zero_cmd = self._create_ackerman_control_command(self.get_clock().now(), [0.0, 0.0], 0.0, False)
        self._command_pub.publish(zero_cmd)

        self.get_logger().warn(">> Stop Completed!")

        # show results
        self._sim_logger.show_results(self._current_laps, self._lap_times, self._car)

    @classmethod
    def in_pkg_share(cls, file_path: str) -> str:
        return cls.PKG_PATH + file_path
