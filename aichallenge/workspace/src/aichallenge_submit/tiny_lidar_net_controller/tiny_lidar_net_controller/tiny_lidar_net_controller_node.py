#!/usr/bin/env python3
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from autoware_auto_control_msgs.msg import AckermannControlCommand

from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
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
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('acceleration', 0.1)
        self.declare_parameter('control_mode', 'ai')
        self.declare_parameter('sensor_timeout_sec', 0.25)
        self.declare_parameter('watchdog_period_sec', 0.05)
        self.declare_parameter('startup_grace_sec', 2.0)
        self.declare_parameter('stale_brake_acceleration', -1.0)
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
        self.declare_parameter('debug', False)

        # --- Initialization ---
        input_dim = self.get_parameter('model.input_dim').value
        output_dim = self.get_parameter('model.output_dim').value
        architecture = self.get_parameter('model.architecture').value
        ckpt_path = self.get_parameter('model.ckpt_path').value
        max_range = self.get_parameter('max_range').value
        acceleration = self.get_parameter('acceleration').value
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
        
        self.debug = self.get_parameter('debug').value
        self.log_interval = self.get_parameter('log_interval_sec').value

        positive_parameters = {
            'log_interval_sec': self.log_interval,
            'sensor_timeout_sec': self.sensor_timeout_sec,
            'watchdog_period_sec': watchdog_period_sec,
            'startup_grace_sec': self.startup_grace_sec,
            'max_steering_angle_rad': self.max_steering_angle_rad,
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
                control_mode=control_mode,
                max_range=max_range,
                gap_teacher_config=gap_teacher_config,
            )
            self.get_logger().info(
                f"Core initialized. Arch: {architecture}, Input: {input_dim}, "
                f"MaxRange: {max_range}, "
                f"ValidatedWeights: {self.core.loaded_parameter_count}"
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
        self.last_log_time = self.get_clock().now()
        self.startup_time = self.get_clock().now()
        self.last_scan_time = None
        self.sensor_stale = False
        self.last_error_log_time = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos
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
            accel, steer = self.core.process(ranges)
            gap_decision = self.core.last_gap_teacher_decision
            if gap_decision is not None and gap_decision.active:
                self.gap_teacher_active_count += 1
            safety_decision = self.core.last_longitudinal_safety_decision
            if safety_decision is not None and safety_decision.active:
                self.longitudinal_safety_active_count += 1
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
        except Exception as exc:
            self._log_inference_error(exc)
            self._publish_stop()

        # 4. Operational metrics
        duration_ms = (time.monotonic() - start_time) * 1000.0
        self.inference_times.append(duration_ms)
        self._log_performance_metrics()

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

                safety_status = ""
                safety_decision = self.core.last_longitudinal_safety_decision
                if safety_decision is not None:
                    safety_status = (
                        " longitudinal_safety_active="
                        f"{interval_safety_active}/{interval_scans} "
                        f"front_m={safety_decision.front_distance_m:.2f} "
                        f"safety_reason={safety_decision.reason}"
                    )

                self.get_logger().info(
                    f"E2E_STATUS scans={self.scan_count} stale={int(self.sensor_stale)} "
                    f"scan_hz={scan_hz:.2f} "
                    f"avg_inference_ms={avg_time:.2f} "
                    f"max_inference_ms={max_time:.2f} "
                    f"inference_capacity_hz={inference_capacity_hz:.2f}"
                    f"{teacher_status}"
                    f"{safety_status}"
                )
                self.inference_times.clear()
                self.last_log_scan_count = self.scan_count
                self.last_log_gap_teacher_active_count = (
                    self.gap_teacher_active_count
                )
                self.last_log_longitudinal_safety_active_count = (
                    self.longitudinal_safety_active_count
                )

            self.last_log_time = now


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
