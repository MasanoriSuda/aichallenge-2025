#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport

from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
from tiny_lidar_net_controller.speed_committed_teacher import (
    SpeedCommittedTeacherConfig,
)
from tiny_lidar_net_controller.longitudinal_safety import (
    SpeedAwareLongitudinalSafetyConfig,
)
from tiny_lidar_net_controller.recurrent_shadow_executor import (
    LatestWinsRecurrentShadowExecutor,
)
from tiny_lidar_net_controller.recurrent_shadow_process import (
    RecurrentShadowSubprocessEvaluator,
)
from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore


class TinyLidarNetNode(Node):
    """ROS 2 Node for TinyLidarNet autonomous driving control.

    This node subscribes to LaserScan messages, processes them using the
    TinyLidarNetCore logic, and publishes AckermannControlCommand messages.
    """

    def __init__(self):
        super().__init__('tiny_lidar_net_node')

        # --- Parameter Declaration ---
        self.declare_parameter('log_interval_sec', 5.0)
        self.declare_parameter('model.input_dim', 1080)
        self.declare_parameter('model.output_dim', 2)
        self.declare_parameter('model.architecture', 'large')
        self.declare_parameter('model.ckpt_path', '')
        self.declare_parameter('model.residual_ckpt_path', '')
        self.declare_parameter('model.residual_max_abs_delta_rad', 1.28)
        self.declare_parameter('model.residual_architecture', 'stateless')
        self.declare_parameter('model.spatial_shadow_ckpt_path', '')
        self.declare_parameter('model.spatial_shadow_expected_sha256', '')
        self.declare_parameter('model.spatial_shadow_hidden_dim', 128)
        self.declare_parameter('model.spatial_shadow_projection_dim', 128)
        self.declare_parameter('model.spatial_shadow_use_speed', True)
        self.declare_parameter('model.spatial_shadow_use_base_steering', False)
        self.declare_parameter('model.spatial_shadow_max_speed_mps', 12.0)
        self.declare_parameter('model.spatial_shadow_max_abs_delta_rad', 1.2)
        self.declare_parameter('model.spatial_authority_enabled', False)
        self.declare_parameter('model.spatial_authority_max_abs_delta_rad', 0.12)
        self.declare_parameter('model.recurrent_shadow_ckpt_path', '')
        self.declare_parameter('model.recurrent_shadow_expected_sha256', '')
        self.declare_parameter('model.recurrent_shadow_hidden_dim', 64)
        self.declare_parameter('model.recurrent_shadow_projection_dim', 128)
        self.declare_parameter('model.recurrent_shadow_use_speed', False)
        self.declare_parameter('model.recurrent_shadow_speed_embedding_dim', 16)
        self.declare_parameter('model.recurrent_shadow_max_speed_mps', 12.0)
        self.declare_parameter(
            'model.recurrent_shadow_max_abs_correction_rad', 0.64
        )
        self.declare_parameter(
            'model.recurrent_shadow_correction_deadband_rad', 0.02
        )
        self.declare_parameter('model.recurrent_authority_enabled', False)
        self.declare_parameter(
            'model.recurrent_authority_max_abs_correction_rad', 0.24
        )
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('acceleration', 0.1)
        self.declare_parameter('maximum_forward_speed_mps', 0.0)
        self.declare_parameter('control_mode', 'ai')
        self.declare_parameter('sensor_timeout_sec', 0.25)
        self.declare_parameter('watchdog_period_sec', 0.05)
        self.declare_parameter('startup_grace_sec', 2.0)
        self.declare_parameter('stale_brake_acceleration', -1.0)
        self.declare_parameter('spatial_shadow_speed_timeout_sec', 0.1)
        self.declare_parameter('max_steering_angle_rad', 0.64)
        self.declare_parameter('gap_teacher.trigger_distance_m', 7.0)
        self.declare_parameter('gap_teacher.slow_distance_m', 3.0)
        self.declare_parameter('gap_teacher.stop_distance_m', 1.5)
        self.declare_parameter('gap_teacher.bubble_margin_m', 0.25)
        self.declare_parameter('gap_teacher.vehicle_half_width_m', 0.725)
        self.declare_parameter('gap_teacher.steering_angle_gain', 0.75)
        self.declare_parameter('gap_teacher.side_start_angle_rad', 1.3)
        self.declare_parameter('gap_teacher.side_trigger_distance_m', 1.8)
        self.declare_parameter('gap_teacher.side_critical_distance_m', 0.9)
        self.declare_parameter('gap_teacher.side_cluster_points', 3)
        self.declare_parameter('gap_teacher.brake_acceleration_mps2', -1.0)
        self.declare_parameter(
            'speed_committed_teacher.reaction_time_sec', 0.25
        )
        self.declare_parameter(
            'speed_committed_teacher.preview_time_sec', 0.50
        )
        self.declare_parameter(
            'speed_committed_teacher.release_confirmation_samples', 5
        )
        self.declare_parameter(
            'speed_committed_teacher.side_switch_confirmation_samples', 2
        )
        self.declare_parameter(
            'speed_committed_teacher.minimum_commit_speed_mps', 0.50
        )
        self.declare_parameter(
            'longitudinal_safety.reaction_time_sec', 0.25
        )
        self.declare_parameter(
            'longitudinal_safety.effective_deceleration_mps2', 1.0
        )
        self.declare_parameter(
            'longitudinal_safety.speed_error_gain', 1.0
        )
        self.declare_parameter('debug', False)

        # --- Initialization ---
        input_dim = self.get_parameter('model.input_dim').value
        output_dim = self.get_parameter('model.output_dim').value
        architecture = self.get_parameter('model.architecture').value
        ckpt_path = self.get_parameter('model.ckpt_path').value
        residual_ckpt_path = self.get_parameter(
            'model.residual_ckpt_path'
        ).value
        residual_max_abs_delta_rad = float(
            self.get_parameter('model.residual_max_abs_delta_rad').value
        )
        residual_architecture = str(
            self.get_parameter('model.residual_architecture').value
        )
        spatial_shadow_ckpt_path = str(
            self.get_parameter('model.spatial_shadow_ckpt_path').value
        )
        spatial_shadow_expected_sha256 = str(
            self.get_parameter('model.spatial_shadow_expected_sha256').value
        )
        spatial_shadow_hidden_dim = int(
            self.get_parameter('model.spatial_shadow_hidden_dim').value
        )
        spatial_shadow_projection_dim = int(
            self.get_parameter('model.spatial_shadow_projection_dim').value
        )
        spatial_shadow_use_speed = bool(
            self.get_parameter('model.spatial_shadow_use_speed').value
        )
        spatial_shadow_use_base_steering = bool(
            self.get_parameter(
                'model.spatial_shadow_use_base_steering'
            ).value
        )
        spatial_shadow_max_speed_mps = float(
            self.get_parameter('model.spatial_shadow_max_speed_mps').value
        )
        spatial_shadow_max_abs_delta_rad = float(
            self.get_parameter('model.spatial_shadow_max_abs_delta_rad').value
        )
        spatial_authority_enabled = bool(
            self.get_parameter('model.spatial_authority_enabled').value
        )
        spatial_authority_max_abs_delta_rad = float(
            self.get_parameter(
                'model.spatial_authority_max_abs_delta_rad'
            ).value
        )
        recurrent_shadow_ckpt_path = str(
            self.get_parameter('model.recurrent_shadow_ckpt_path').value
        )
        recurrent_shadow_expected_sha256 = str(
            self.get_parameter(
                'model.recurrent_shadow_expected_sha256'
            ).value
        )
        recurrent_shadow_hidden_dim = int(
            self.get_parameter('model.recurrent_shadow_hidden_dim').value
        )
        recurrent_shadow_projection_dim = int(
            self.get_parameter('model.recurrent_shadow_projection_dim').value
        )
        recurrent_shadow_use_speed = bool(
            self.get_parameter('model.recurrent_shadow_use_speed').value
        )
        recurrent_shadow_speed_embedding_dim = int(
            self.get_parameter(
                'model.recurrent_shadow_speed_embedding_dim'
            ).value
        )
        recurrent_shadow_max_speed_mps = float(
            self.get_parameter('model.recurrent_shadow_max_speed_mps').value
        )
        recurrent_shadow_max_abs_correction_rad = float(
            self.get_parameter(
                'model.recurrent_shadow_max_abs_correction_rad'
            ).value
        )
        recurrent_shadow_correction_deadband_rad = float(
            self.get_parameter(
                'model.recurrent_shadow_correction_deadband_rad'
            ).value
        )
        recurrent_authority_enabled = bool(
            self.get_parameter('model.recurrent_authority_enabled').value
        )
        recurrent_authority_max_abs_correction_rad = float(
            self.get_parameter(
                'model.recurrent_authority_max_abs_correction_rad'
            ).value
        )
        max_range = self.get_parameter('max_range').value
        acceleration = self.get_parameter('acceleration').value
        maximum_forward_speed_mps = float(
            self.get_parameter('maximum_forward_speed_mps').value
        )
        control_mode = self.get_parameter('control_mode').value
        self.sensor_timeout_sec = float(
            self.get_parameter('sensor_timeout_sec').value
        )
        watchdog_period_sec = float(
            self.get_parameter('watchdog_period_sec').value
        )
        self.startup_grace_sec = float(
            self.get_parameter('startup_grace_sec').value
        )
        self.stale_brake_acceleration = float(
            self.get_parameter('stale_brake_acceleration').value
        )
        self.spatial_shadow_speed_timeout_sec = float(
            self.get_parameter('spatial_shadow_speed_timeout_sec').value
        )
        self.max_steering_angle_rad = float(
            self.get_parameter('max_steering_angle_rad').value
        )
        gap_teacher_config = GapTeacherConfig(
            trigger_distance_m=float(
                self.get_parameter('gap_teacher.trigger_distance_m').value
            ),
            slow_distance_m=float(
                self.get_parameter('gap_teacher.slow_distance_m').value
            ),
            stop_distance_m=float(
                self.get_parameter('gap_teacher.stop_distance_m').value
            ),
            bubble_margin_m=float(
                self.get_parameter('gap_teacher.bubble_margin_m').value
            ),
            vehicle_half_width_m=float(
                self.get_parameter('gap_teacher.vehicle_half_width_m').value
            ),
            max_steering_angle_rad=self.max_steering_angle_rad,
            steering_angle_gain=float(
                self.get_parameter('gap_teacher.steering_angle_gain').value
            ),
            side_start_angle_rad=float(
                self.get_parameter('gap_teacher.side_start_angle_rad').value
            ),
            side_trigger_distance_m=float(
                self.get_parameter('gap_teacher.side_trigger_distance_m').value
            ),
            side_critical_distance_m=float(
                self.get_parameter('gap_teacher.side_critical_distance_m').value
            ),
            side_cluster_points=int(
                self.get_parameter('gap_teacher.side_cluster_points').value
            ),
            brake_acceleration_mps2=float(
                self.get_parameter('gap_teacher.brake_acceleration_mps2').value
            ),
        )
        speed_committed_teacher_config = SpeedCommittedTeacherConfig(
            reaction_time_sec=float(
                self.get_parameter(
                    'speed_committed_teacher.reaction_time_sec'
                ).value
            ),
            preview_time_sec=float(
                self.get_parameter(
                    'speed_committed_teacher.preview_time_sec'
                ).value
            ),
            release_confirmation_samples=int(
                self.get_parameter(
                    'speed_committed_teacher.release_confirmation_samples'
                ).value
            ),
            side_switch_confirmation_samples=int(
                self.get_parameter(
                    'speed_committed_teacher.side_switch_confirmation_samples'
                ).value
            ),
            minimum_commit_speed_mps=float(
                self.get_parameter(
                    'speed_committed_teacher.minimum_commit_speed_mps'
                ).value
            ),
        )
        speed_aware_longitudinal_safety_config = (
            SpeedAwareLongitudinalSafetyConfig(
                reaction_time_sec=float(
                    self.get_parameter(
                        'longitudinal_safety.reaction_time_sec'
                    ).value
                ),
                effective_deceleration_mps2=float(
                    self.get_parameter(
                        'longitudinal_safety.effective_deceleration_mps2'
                    ).value
                ),
                speed_error_gain=float(
                    self.get_parameter(
                        'longitudinal_safety.speed_error_gain'
                    ).value
                ),
            )
        )

        self.debug = self.get_parameter('debug').value
        self.log_interval = self.get_parameter('log_interval_sec').value

        positive_parameters = {
            'log_interval_sec': self.log_interval,
            'sensor_timeout_sec': self.sensor_timeout_sec,
            'watchdog_period_sec': watchdog_period_sec,
            'startup_grace_sec': self.startup_grace_sec,
            'max_steering_angle_rad': self.max_steering_angle_rad,
            'model.residual_max_abs_delta_rad': residual_max_abs_delta_rad,
            'model.spatial_authority_max_abs_delta_rad': (
                spatial_authority_max_abs_delta_rad
            ),
            'spatial_shadow_speed_timeout_sec': (
                self.spatial_shadow_speed_timeout_sec
            ),
            'model.recurrent_shadow_max_speed_mps': (
                recurrent_shadow_max_speed_mps
            ),
            'model.recurrent_shadow_max_abs_correction_rad': (
                recurrent_shadow_max_abs_correction_rad
            ),
            'model.recurrent_authority_max_abs_correction_rad': (
                recurrent_authority_max_abs_correction_rad
            ),
        }
        for name, value in positive_parameters.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if (
            not np.isfinite(self.stale_brake_acceleration)
            or self.stale_brake_acceleration >= 0.0
        ):
            raise ValueError('stale_brake_acceleration must be finite and negative')

        try:
            self.core = TinyLidarNetCore(
                input_dim=input_dim,
                output_dim=output_dim,
                architecture=architecture,
                ckpt_path=ckpt_path,
                acceleration=acceleration,
                maximum_forward_speed_mps=maximum_forward_speed_mps,
                control_mode=control_mode,
                max_range=max_range,
                gap_teacher_config=gap_teacher_config,
                speed_committed_teacher_config=(
                    speed_committed_teacher_config
                ),
                speed_aware_longitudinal_safety_config=(
                    speed_aware_longitudinal_safety_config
                ),
                residual_ckpt_path=residual_ckpt_path,
                residual_max_abs_delta_rad=residual_max_abs_delta_rad,
                residual_architecture=residual_architecture,
                spatial_shadow_ckpt_path=spatial_shadow_ckpt_path,
                spatial_shadow_expected_sha256=spatial_shadow_expected_sha256,
                spatial_shadow_hidden_dim=spatial_shadow_hidden_dim,
                spatial_shadow_projection_dim=spatial_shadow_projection_dim,
                spatial_shadow_use_speed=spatial_shadow_use_speed,
                spatial_shadow_use_base_steering=(
                    spatial_shadow_use_base_steering
                ),
                spatial_shadow_max_speed_mps=spatial_shadow_max_speed_mps,
                spatial_shadow_max_abs_delta_rad=(
                    spatial_shadow_max_abs_delta_rad
                ),
                spatial_authority_enabled=spatial_authority_enabled,
                spatial_authority_max_abs_delta_rad=(
                    spatial_authority_max_abs_delta_rad
                ),
                recurrent_shadow_ckpt_path=recurrent_shadow_ckpt_path,
                recurrent_shadow_expected_sha256=(
                    recurrent_shadow_expected_sha256
                ),
                recurrent_shadow_hidden_dim=recurrent_shadow_hidden_dim,
                recurrent_shadow_projection_dim=(
                    recurrent_shadow_projection_dim
                ),
                recurrent_shadow_use_speed=recurrent_shadow_use_speed,
                recurrent_shadow_speed_embedding_dim=(
                    recurrent_shadow_speed_embedding_dim
                ),
                recurrent_shadow_max_speed_mps=(
                    recurrent_shadow_max_speed_mps
                ),
                recurrent_shadow_max_abs_correction_rad=(
                    recurrent_shadow_max_abs_correction_rad
                ),
                recurrent_shadow_correction_deadband_rad=(
                    recurrent_shadow_correction_deadband_rad
                ),
                recurrent_authority_enabled=recurrent_authority_enabled,
                recurrent_authority_max_abs_correction_rad=(
                    recurrent_authority_max_abs_correction_rad
                ),
            )
            recurrent_runtime_config = (
                self.core.recurrent_shadow_runtime_config
                or {
                    "hidden_dim": recurrent_shadow_hidden_dim,
                    "projection_dim": recurrent_shadow_projection_dim,
                    "use_speed": recurrent_shadow_use_speed,
                    "speed_embedding_dim": recurrent_shadow_speed_embedding_dim,
                    "max_speed_mps": recurrent_shadow_max_speed_mps,
                    "max_abs_correction_rad": (
                        recurrent_shadow_max_abs_correction_rad
                    ),
                    "correction_deadband_rad": (
                        recurrent_shadow_correction_deadband_rad
                    ),
                }
            )
            self.get_logger().info(
                f"Core initialized. Arch: {architecture}, Input: {input_dim}, "
                f"MaxRange: {max_range}, "
                f"Acceleration: {float(acceleration):.6f}, "
                "MaximumForwardSpeed: "
                f"{maximum_forward_speed_mps:.6f}, "
                f"ValidatedWeights: {self.core.loaded_parameter_count}, "
                "ResidualWeights: "
                f"{self.core.residual_loaded_parameter_count}, "
                f"ResidualEnabled: {self.core.residual_model is not None}, "
                f"ResidualArchitecture: {self.core.residual_architecture}, "
                "SpatialShadowWeights: "
                f"{self.core.spatial_shadow_loaded_parameter_count}, "
                "SpatialShadowEnabled: "
                f"{self.core.spatial_shadow_model is not None}, "
                "SpatialShadowConfig: "
                f"hidden={spatial_shadow_hidden_dim},"
                f"projection={spatial_shadow_projection_dim},"
                f"use_speed={int(spatial_shadow_use_speed)},"
                "use_base_steering="
                f"{int(spatial_shadow_use_base_steering)},"
                f"max_speed_mps={spatial_shadow_max_speed_mps:.6f},"
                f"max_delta_rad={spatial_shadow_max_abs_delta_rad:.6f},"
                "speed_timeout_sec="
                f"{self.spatial_shadow_speed_timeout_sec:.6f},"
                f"authority_enabled={int(spatial_authority_enabled)},"
                "authority_max_delta_rad="
                f"{spatial_authority_max_abs_delta_rad:.6f},"
                " RecurrentShadowWeights: "
                f"{self.core.recurrent_shadow_loaded_parameter_count},"
                " RecurrentShadowEnabled: "
                f"{self.core.recurrent_shadow_model is not None},"
                " RecurrentShadowConfig: "
                f"contract={self.core.recurrent_shadow_artifact_contract},"
                f"hidden={recurrent_runtime_config['hidden_dim']},"
                f"projection={recurrent_runtime_config['projection_dim']},"
                f"use_speed={int(recurrent_runtime_config['use_speed'])},"
                "speed_embedding="
                f"{recurrent_runtime_config['speed_embedding_dim']},"
                "max_speed_mps="
                f"{recurrent_runtime_config['max_speed_mps']:.6f},"
                "max_correction_rad="
                f"{recurrent_runtime_config['max_abs_correction_rad']:.6f},"
                "deadband_rad="
                f"{recurrent_runtime_config['correction_deadband_rad']:.6f},"
                f"authority_enabled={int(recurrent_authority_enabled)},"
                "authority_max_correction_rad="
                f"{recurrent_authority_max_abs_correction_rad:.6f}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to initialize core logic: {e}")
            raise e

        # --- Communication Setup ---
        self.inference_times = []
        self.scan_count = 0
        self.last_log_scan_count = 0
        self.gap_teacher_active_count = 0
        self.last_log_gap_teacher_active_count = 0
        self.longitudinal_safety_active_count = 0
        self.last_log_longitudinal_safety_active_count = 0
        self.speed_governor_active_count = 0
        self.last_log_speed_governor_active_count = 0
        self.last_log_time = self.get_clock().now()
        self.startup_time = self.get_clock().now()
        self.last_scan_time = None
        self.sensor_stale = False
        self.last_error_log_time = 0.0
        self.latest_wheel_speed_mps = None
        self.latest_wheel_speed_time = None
        self.shadow_admitted_count = 0
        self.last_log_shadow_admitted_count = 0
        self.shadow_skipped_count = 0
        self.last_log_shadow_skipped_count = 0
        self.shadow_error_count = 0
        self.last_log_shadow_error_count = 0
        self.shadow_corrections = []
        self.spatial_authority_applied_count = 0
        self.last_log_spatial_authority_applied_count = 0
        self.spatial_authority_clipped_count = 0
        self.last_log_spatial_authority_clipped_count = 0
        self.spatial_authority_corrections = []
        self.recurrent_shadow_admitted_count = 0
        self.last_log_recurrent_shadow_admitted_count = 0
        self.recurrent_shadow_skipped_count = 0
        self.last_log_recurrent_shadow_skipped_count = 0
        self.recurrent_shadow_error_count = 0
        self.last_log_recurrent_shadow_error_count = 0
        self.recurrent_shadow_corrections = []
        self.recurrent_authority_applied_count = 0
        self.last_log_recurrent_authority_applied_count = 0
        self.recurrent_authority_clipped_count = 0
        self.last_log_recurrent_authority_clipped_count = 0
        self.recurrent_authority_corrections = []
        self.recurrent_shadow_executor = None
        self.recurrent_shadow_process_identity = None
        self.defer_recurrent_shadow = bool(
            self.core.recurrent_shadow_model is not None
            and not self.core.recurrent_authority_enabled
        )
        self.recurrent_async_inference_times = []
        if self.defer_recurrent_shadow:
            recurrent_evaluator = None
            try:
                recurrent_evaluator = RecurrentShadowSubprocessEvaluator(
                    checkpoint_path=recurrent_shadow_ckpt_path,
                    expected_sha256=recurrent_shadow_expected_sha256,
                    expected_runtime_config=recurrent_runtime_config,
                    response_timeout_sec=self.sensor_timeout_sec,
                )
                self.recurrent_shadow_process_identity = (
                    recurrent_evaluator.identity
                )
                self.recurrent_shadow_executor = (
                    LatestWinsRecurrentShadowExecutor(
                        recurrent_evaluator,
                        max_result_age_sec=self.sensor_timeout_sec,
                    )
                )
                identity = self.recurrent_shadow_process_identity
                self.get_logger().info(
                    "Recurrent shadow execution: process-async-latest-wins, "
                    "authority=disabled, command_path=isolated, "
                    f"worker_pid={identity.pid}, "
                    "worker_openblas_threads="
                    f"{identity.openblas_threads}, "
                    f"artifact_sha256={identity.sha256}, "
                    f"artifact_contract={identity.artifact_contract}, "
                    "loaded_parameters="
                    f"{identity.loaded_parameter_count}"
                )
            except Exception as exc:
                if recurrent_evaluator is not None:
                    recurrent_evaluator.close()
                self.recurrent_shadow_error_count += 1
                self.get_logger().error(
                    "Recurrent shadow process unavailable; production "
                    f"command remains active without observation: {exc}"
                )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos
        )
        self.sub_wheel_speed = None
        if self.core.requires_wheel_speed:
            self.sub_wheel_speed = self.create_subscription(
                VelocityReport,
                "/vehicle/status/velocity_status",
                self._wheel_speed_callback,
                1,
            )
        self.pub_control = self.create_publisher(
            AckermannControlCommand, "/control/command/control_cmd", 1
        )
        self.watchdog_timer = self.create_timer(
            watchdog_period_sec, self._watchdog_callback
        )

        self.get_logger().info(
            "TinyLidarNetNode is ready: input=/scan, "
            "output=/control/command/control_cmd, "
            f"sensor_timeout={self.sensor_timeout_sec:.3f}s"
        )

    def scan_callback(self, msg: LaserScan):
        """Callback for LaserScan subscription.

        Processes the scan data via the core logic and publishes a control command.

        Args:
            msg (LaserScan): The incoming ROS 2 LaserScan message.
        """
        start_time = time.monotonic()

        # 1. Convert ROS message to Numpy
        # We pass the raw array; the core logic handles NaN/Inf and normalization.
        ranges = np.array(msg.ranges, dtype=np.float32)

        try:
            # 2. Process via Core Logic
            accel, steer = self.core.process(
                ranges,
                self._fresh_wheel_speed(),
                defer_recurrent_shadow=self.defer_recurrent_shadow,
            )
            if self.core.spatial_shadow_model is not None:
                if self.core.last_spatial_shadow_admitted:
                    self.shadow_admitted_count += 1
                    self.shadow_corrections.append(
                        self.core.last_spatial_shadow_correction_rad
                    )
                elif self.core.last_spatial_shadow_status.startswith(
                    "inference-error:"
                ):
                    self.shadow_error_count += 1
                else:
                    self.shadow_skipped_count += 1
                if self.core.last_spatial_authority_applied:
                    self.spatial_authority_applied_count += 1
                    self.spatial_authority_corrections.append(
                        self.core.last_spatial_authority_correction_rad
                    )
                if self.core.last_spatial_authority_clipped:
                    self.spatial_authority_clipped_count += 1
            gap_decision = self.core.last_gap_teacher_decision
            if gap_decision is not None and gap_decision.active:
                self.gap_teacher_active_count += 1
            safety_decision = self.core.last_longitudinal_safety_decision
            if safety_decision is not None and safety_decision.active:
                self.longitudinal_safety_active_count += 1
            governor_decision = self.core.last_speed_governor_decision
            if governor_decision is not None and governor_decision.active:
                self.speed_governor_active_count += 1
            steer = float(np.clip(
                steer,
                -self.max_steering_angle_rad,
                self.max_steering_angle_rad,
            ))

            # 3. Publish Command
            self._publish_command(accel, steer)
            now = self.get_clock().now()
            self.last_scan_time = now
            self.scan_count += 1
            if self.sensor_stale:
                self.get_logger().info("LiDAR stream recovered; resuming ML control")
                self.sensor_stale = False

            # Recurrent observation is accounted only after the production
            # command has been published.  Async results never feed back into
            # the current or a future command.
            if self.recurrent_shadow_executor is not None:
                self._collect_async_recurrent_results()
                sample = self.core.last_recurrent_shadow_sample
                if sample is None:
                    self.recurrent_shadow_skipped_count += 1
                else:
                    self.recurrent_shadow_skipped_count += (
                        self.recurrent_shadow_executor.submit(
                            self.scan_count, sample
                        )
                    )
            elif self.core.recurrent_shadow_model is not None:
                if self.core.last_recurrent_shadow_admitted:
                    self.recurrent_shadow_admitted_count += 1
                    self.recurrent_shadow_corrections.append(
                        self.core.last_recurrent_shadow_correction_rad
                    )
                elif self.core.last_recurrent_shadow_status.startswith(
                    "inference-error:"
                ):
                    self.recurrent_shadow_error_count += 1
                else:
                    self.recurrent_shadow_skipped_count += 1
                if self.core.last_recurrent_authority_applied:
                    self.recurrent_authority_applied_count += 1
                    self.recurrent_authority_corrections.append(
                        self.core.last_recurrent_authority_correction_rad
                    )
                if self.core.last_recurrent_authority_clipped:
                    self.recurrent_authority_clipped_count += 1
        except Exception as exc:
            self._log_inference_error(exc)
            self._publish_stop()

        # 4. Operational metrics
        duration_ms = (time.monotonic() - start_time) * 1000.0
        self.inference_times.append(duration_ms)
        self._log_performance_metrics()

    def _wheel_speed_callback(self, msg: VelocityReport):
        """Retain fresh wheel odometry without consuming fused localization."""
        speed = abs(float(msg.longitudinal_velocity))
        if not np.isfinite(speed):
            self.latest_wheel_speed_mps = None
            self.latest_wheel_speed_time = None
            return
        self.latest_wheel_speed_mps = speed
        self.latest_wheel_speed_time = self.get_clock().now()

    def _fresh_wheel_speed(self):
        if not self.core.requires_wheel_speed:
            return None
        if (
            self.latest_wheel_speed_mps is None
            or self.latest_wheel_speed_time is None
        ):
            return None
        age_sec = (
            self.get_clock().now() - self.latest_wheel_speed_time
        ).nanoseconds / 1e9
        if age_sec < 0.0 or age_sec > self.spatial_shadow_speed_timeout_sec:
            return None
        return self.latest_wheel_speed_mps

    def _collect_async_recurrent_results(self) -> None:
        """Move diagnostic worker results into main-thread telemetry only."""
        if self.recurrent_shadow_executor is None:
            return
        for result in self.recurrent_shadow_executor.drain_completed():
            self.recurrent_async_inference_times.append(result.inference_ms)
            self.core.last_recurrent_shadow_admitted = False
            self.core.last_recurrent_shadow_status = result.status
            if result.status == "ok" and result.evaluation is not None:
                self.core.record_recurrent_shadow_evaluation(
                    result.evaluation, retain_hidden=False
                )
                self.recurrent_shadow_admitted_count += 1
                self.recurrent_shadow_corrections.append(
                    result.evaluation.correction_rad
                )
            elif result.status.startswith("inference-error:"):
                self.recurrent_shadow_error_count += 1
            else:
                self.recurrent_shadow_skipped_count += 1
        self.core.recurrent_shadow_reset_count = (
            self.recurrent_shadow_executor.stats()["resets"]
        )

    def _publish_command(self, acceleration: float, steering: float):
        """Publish one finite Ackermann command."""
        if not np.isfinite(acceleration) or not np.isfinite(steering):
            raise ValueError("refusing to publish a non-finite control command")
        cmd = AckermannControlCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.longitudinal.acceleration = float(acceleration)
        cmd.lateral.steering_tire_angle = float(steering)
        self.pub_control.publish(cmd)

    def _publish_stop(self):
        """Invalidate a retained drive command when sensing or inference is unsafe."""
        self._publish_command(self.stale_brake_acceleration, 0.0)

    def _watchdog_callback(self):
        """Stop if no successful LiDAR inference has completed recently."""
        now = self.get_clock().now()
        if self.last_scan_time is None:
            elapsed_sec = (now - self.startup_time).nanoseconds / 1e9
            timeout_sec = self.startup_grace_sec
            reason = "no valid LiDAR scan received after startup"
        else:
            elapsed_sec = (now - self.last_scan_time).nanoseconds / 1e9
            timeout_sec = self.sensor_timeout_sec
            reason = "LiDAR stream is stale"

        if elapsed_sec <= timeout_sec:
            return

        if not self.sensor_stale:
            self.get_logger().error(
                f"{reason}: age={elapsed_sec:.3f}s limit={timeout_sec:.3f}s; "
                "publishing stop"
            )
            self.sensor_stale = True
            self.core.reset_residual_history()
            if self.recurrent_shadow_executor is not None:
                self.recurrent_shadow_executor.reset_history()
        self._publish_stop()

    def _log_inference_error(self, exc: Exception):
        now = time.monotonic()
        if now - self.last_error_log_time >= 1.0:
            self.get_logger().error(
                f"TinyLidarNet inference rejected; publishing stop: {exc}"
            )
            self.last_error_log_time = now

    def _log_performance_metrics(self):
        """Logs internal performance metrics at fixed intervals."""
        now = self.get_clock().now()
        elapsed_sec = (now - self.last_log_time).nanoseconds / 1e9

        if elapsed_sec > self.log_interval:
            if self.inference_times:
                avg_time = np.mean(self.inference_times)
                max_time = np.max(self.inference_times)
                inference_capacity_hz = 1000.0 / avg_time if avg_time > 0 else 0.0
                interval_scans = self.scan_count - self.last_log_scan_count
                interval_teacher_active = (
                    self.gap_teacher_active_count
                    - self.last_log_gap_teacher_active_count
                )
                interval_safety_active = (
                    self.longitudinal_safety_active_count
                    - self.last_log_longitudinal_safety_active_count
                )
                interval_speed_governor_active = (
                    self.speed_governor_active_count
                    - self.last_log_speed_governor_active_count
                )
                interval_shadow_admitted = (
                    self.shadow_admitted_count
                    - self.last_log_shadow_admitted_count
                )
                interval_shadow_skipped = (
                    self.shadow_skipped_count - self.last_log_shadow_skipped_count
                )
                interval_shadow_errors = (
                    self.shadow_error_count - self.last_log_shadow_error_count
                )
                interval_authority_applied = (
                    self.spatial_authority_applied_count
                    - self.last_log_spatial_authority_applied_count
                )
                interval_authority_clipped = (
                    self.spatial_authority_clipped_count
                    - self.last_log_spatial_authority_clipped_count
                )
                interval_recurrent_admitted = (
                    self.recurrent_shadow_admitted_count
                    - self.last_log_recurrent_shadow_admitted_count
                )
                interval_recurrent_skipped = (
                    self.recurrent_shadow_skipped_count
                    - self.last_log_recurrent_shadow_skipped_count
                )
                interval_recurrent_errors = (
                    self.recurrent_shadow_error_count
                    - self.last_log_recurrent_shadow_error_count
                )
                interval_recurrent_authority_applied = (
                    self.recurrent_authority_applied_count
                    - self.last_log_recurrent_authority_applied_count
                )
                interval_recurrent_authority_clipped = (
                    self.recurrent_authority_clipped_count
                    - self.last_log_recurrent_authority_clipped_count
                )
                scan_hz = interval_scans / elapsed_sec if elapsed_sec > 0.0 else 0.0

                teacher_status = ""
                decision = self.core.last_gap_teacher_decision
                if decision is not None:
                    teacher_status = (
                        f" gap_teacher_active={interval_teacher_active}/{interval_scans} "
                        f"front_m={decision.front_distance_m:.2f} "
                        f"left_side_m={decision.left_side_distance_m:.2f} "
                        f"right_side_m={decision.right_side_distance_m:.2f} "
                        f"gap_angle_rad={decision.target_angle_rad:.2f} "
                        f"gap_reason={decision.reason}"
                    )
                    if hasattr(decision, "supervisor_reason"):
                        teacher_status += (
                            " teacher_speed_mps="
                            f"{decision.speed_mps:.2f} "
                            "teacher_stop_m="
                            f"{decision.required_stop_distance_m:.2f} "
                            "teacher_trigger_m="
                            f"{decision.dynamic_trigger_distance_m:.2f} "
                            "teacher_side="
                            f"{decision.committed_side_sign}/"
                            f"{decision.proposed_side_sign} "
                            "teacher_supervisor="
                            f"{decision.supervisor_reason}"
                        )

                safety_status = ""
                safety_decision = self.core.last_longitudinal_safety_decision
                if safety_decision is not None:
                    safety_status = (
                        " longitudinal_safety_active="
                        f"{interval_safety_active}/{interval_scans} "
                        f"front_m={safety_decision.front_distance_m:.2f} "
                        f"safety_reason={safety_decision.reason}"
                    )
                    if hasattr(safety_decision, "safe_speed_mps"):
                        speed_value = (
                            "nan"
                            if safety_decision.speed_mps is None
                            else f"{safety_decision.speed_mps:.2f}"
                        )
                        safe_speed_value = (
                            "nan"
                            if safety_decision.safe_speed_mps is None
                            else f"{safety_decision.safe_speed_mps:.2f}"
                        )
                        safety_status += (
                            f" safety_speed_mps={speed_value}"
                            f" safety_safe_speed_mps={safe_speed_value}"
                        )

                speed_governor_status = ""
                governor_decision = self.core.last_speed_governor_decision
                if governor_decision is not None:
                    speed_value = (
                        "nan"
                        if governor_decision.speed_mps is None
                        else f"{governor_decision.speed_mps:.3f}"
                    )
                    speed_governor_status = (
                        " speed_governor_active="
                        f"{interval_speed_governor_active}/{interval_scans} "
                        f"speed_mps={speed_value} "
                        "speed_governor_reason="
                        f"{governor_decision.reason}"
                    )

                residual_status = ""
                if self.core.residual_model is not None:
                    residual_status = (
                        " residual_steer_rad="
                        f"{self.core.last_residual_correction_rad:.4f} "
                        "residual_gate="
                        f"{self.core.last_residual_gate_probability:.3f}"
                    )

                shadow_status = ""
                if self.core.spatial_shadow_model is not None:
                    corrections = np.asarray(
                        self.shadow_corrections, dtype=np.float64
                    )
                    mean_abs = (
                        float(np.mean(np.abs(corrections)))
                        if corrections.size
                        else 0.0
                    )
                    p95_abs = (
                        float(np.percentile(np.abs(corrections), 95))
                        if corrections.size
                        else 0.0
                    )
                    probabilities = (
                        self.core.last_spatial_shadow_direction_probabilities
                    )
                    authority_corrections = np.asarray(
                        self.spatial_authority_corrections, dtype=np.float64
                    )
                    authority_mean_abs = (
                        float(np.mean(np.abs(authority_corrections)))
                        if authority_corrections.size
                        else 0.0
                    )
                    authority_max_abs = (
                        float(np.max(np.abs(authority_corrections)))
                        if authority_corrections.size
                        else 0.0
                    )
                    shadow_status = (
                        " spatial_shadow="
                        f"{interval_shadow_admitted}/{interval_scans} "
                        f"shadow_skipped={interval_shadow_skipped} "
                        f"shadow_errors={interval_shadow_errors} "
                        f"shadow_mean_abs_rad={mean_abs:.5f} "
                        f"shadow_p95_abs_rad={p95_abs:.5f} "
                        "shadow_last_rad="
                        f"{self.core.last_spatial_shadow_correction_rad:.5f} "
                        "shadow_prob_lnr="
                        f"{probabilities[0]:.3f},"
                        f"{probabilities[1]:.3f},"
                        f"{probabilities[2]:.3f} "
                        f"shadow_status={self.core.last_spatial_shadow_status} "
                        "spatial_authority_enabled="
                        f"{int(self.core.spatial_authority_enabled)} "
                        "spatial_authority_applied="
                        f"{interval_authority_applied}/{interval_scans} "
                        "spatial_authority_clipped="
                        f"{interval_authority_clipped} "
                        "spatial_authority_mean_abs_rad="
                        f"{authority_mean_abs:.5f} "
                        "spatial_authority_max_abs_rad="
                        f"{authority_max_abs:.5f}"
                    )

                recurrent_status = ""
                if self.core.recurrent_shadow_model is not None:
                    recurrent_corrections = np.asarray(
                        self.recurrent_shadow_corrections, dtype=np.float64
                    )
                    recurrent_mean_abs = (
                        float(np.mean(np.abs(recurrent_corrections)))
                        if recurrent_corrections.size
                        else 0.0
                    )
                    recurrent_p95_abs = (
                        float(np.percentile(np.abs(recurrent_corrections), 95))
                        if recurrent_corrections.size
                        else 0.0
                    )
                    recurrent_authority_corrections = np.asarray(
                        self.recurrent_authority_corrections,
                        dtype=np.float64,
                    )
                    recurrent_authority_mean_abs = (
                        float(np.mean(np.abs(recurrent_authority_corrections)))
                        if recurrent_authority_corrections.size
                        else 0.0
                    )
                    recurrent_authority_max_abs = (
                        float(np.max(np.abs(recurrent_authority_corrections)))
                        if recurrent_authority_corrections.size
                        else 0.0
                    )
                    recurrent_status = (
                        " recurrent_shadow="
                        f"{interval_recurrent_admitted}/{interval_scans} "
                        "recurrent_skipped="
                        f"{interval_recurrent_skipped} "
                        "recurrent_errors="
                        f"{interval_recurrent_errors} "
                        "recurrent_mean_abs_rad="
                        f"{recurrent_mean_abs:.5f} "
                        "recurrent_p95_abs_rad="
                        f"{recurrent_p95_abs:.5f} "
                        "recurrent_last_rad="
                        f"{self.core.last_recurrent_shadow_correction_rad:.5f} "
                        "recurrent_raw_last_rad="
                        f"{self.core.last_recurrent_shadow_raw_correction_rad:.5f} "
                        "recurrent_hidden_norm="
                        f"{self.core.last_recurrent_shadow_hidden_norm:.5f} "
                        "recurrent_resets="
                        f"{self.core.recurrent_shadow_reset_count} "
                        "recurrent_status="
                        f"{self.core.last_recurrent_shadow_status} "
                        "recurrent_authority_enabled="
                        f"{int(self.core.recurrent_authority_enabled)} "
                        "recurrent_authority_applied="
                        f"{interval_recurrent_authority_applied}/{interval_scans} "
                        "recurrent_authority_clipped="
                        f"{interval_recurrent_authority_clipped} "
                        "recurrent_authority_mean_abs_rad="
                        f"{recurrent_authority_mean_abs:.5f} "
                        "recurrent_authority_max_abs_rad="
                        f"{recurrent_authority_max_abs:.5f}"
                    )
                    if self.recurrent_shadow_executor is not None:
                        async_stats = self.recurrent_shadow_executor.stats()
                        async_times = np.asarray(
                            self.recurrent_async_inference_times,
                            dtype=np.float64,
                        )
                        async_mean_ms = (
                            float(np.mean(async_times))
                            if async_times.size
                            else 0.0
                        )
                        async_max_ms = (
                            float(np.max(async_times))
                            if async_times.size
                            else 0.0
                        )
                        recurrent_status += (
                            " recurrent_async=1"
                            " recurrent_async_process=1"
                            " recurrent_async_submitted="
                            f"{async_stats['submitted']}"
                            " recurrent_async_completed="
                            f"{async_stats['completed']}"
                            " recurrent_async_dropped="
                            f"{async_stats['dropped']}"
                            " recurrent_async_stale="
                            f"{async_stats['stale']}"
                            " recurrent_async_errors="
                            f"{async_stats['errors']}"
                            " recurrent_async_sample_status="
                            f"{self.core.last_recurrent_shadow_sample_status}"
                            " recurrent_async_mean_ms="
                            f"{async_mean_ms:.2f}"
                            " recurrent_async_max_ms="
                            f"{async_max_ms:.2f}"
                        )

                self.get_logger().info(
                    f"E2E_STATUS scans={self.scan_count} stale={int(self.sensor_stale)} "
                    f"scan_hz={scan_hz:.2f} "
                    f"avg_inference_ms={avg_time:.2f} "
                    f"max_inference_ms={max_time:.2f} "
                    f"inference_capacity_hz={inference_capacity_hz:.2f}"
                    f"{teacher_status}"
                    f"{safety_status}"
                    f"{speed_governor_status}"
                    f"{residual_status}"
                    f"{shadow_status}"
                    f"{recurrent_status}"
                )
                self.inference_times.clear()
                self.last_log_scan_count = self.scan_count
                self.last_log_gap_teacher_active_count = (
                    self.gap_teacher_active_count
                )
                self.last_log_longitudinal_safety_active_count = (
                    self.longitudinal_safety_active_count
                )
                self.last_log_speed_governor_active_count = (
                    self.speed_governor_active_count
                )
                self.last_log_shadow_admitted_count = self.shadow_admitted_count
                self.last_log_shadow_skipped_count = self.shadow_skipped_count
                self.last_log_shadow_error_count = self.shadow_error_count
                self.shadow_corrections.clear()
                self.last_log_spatial_authority_applied_count = (
                    self.spatial_authority_applied_count
                )
                self.last_log_spatial_authority_clipped_count = (
                    self.spatial_authority_clipped_count
                )
                self.spatial_authority_corrections.clear()
                self.last_log_recurrent_shadow_admitted_count = (
                    self.recurrent_shadow_admitted_count
                )
                self.last_log_recurrent_shadow_skipped_count = (
                    self.recurrent_shadow_skipped_count
                )
                self.last_log_recurrent_shadow_error_count = (
                    self.recurrent_shadow_error_count
                )
                self.recurrent_shadow_corrections.clear()
                self.last_log_recurrent_authority_applied_count = (
                    self.recurrent_authority_applied_count
                )
                self.last_log_recurrent_authority_clipped_count = (
                    self.recurrent_authority_clipped_count
                )
                self.recurrent_authority_corrections.clear()
                self.recurrent_async_inference_times.clear()

            self.last_log_time = now

    def destroy_node(self):
        if self.recurrent_shadow_executor is not None:
            self.recurrent_shadow_executor.close()
            self.recurrent_shadow_executor = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TinyLidarNetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
