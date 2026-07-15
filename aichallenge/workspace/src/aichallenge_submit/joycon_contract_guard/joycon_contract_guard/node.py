"""ROS 2 boundary adapter for the upstream teleop manager."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Float32MultiArray, String

from .core import BoostGuard


class JoyconContractGuardNode(Node):
    def __init__(self) -> None:
        super().__init__("joycon_contract_guard")
        timeout = float(self.declare_parameter("boost_status_timeout_sec", 0.5).value)
        self._guard = BoostGuard(timeout)
        self._raw_high = False
        self._last_warning: dict[str, float] = {}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._status_sub = self.create_subscription(
            Float32MultiArray, "/awsim/status", self._on_status, qos
        )
        self._state_sub = self.create_subscription(
            String, "/awsim/state", self._on_state, qos
        )
        self._raw_boost_sub = self.create_subscription(
            Float32MultiArray,
            "/participant/joycon/boost_request_raw",
            self._on_raw_boost,
            qos,
        )
        self._reset_sub = self.create_subscription(
            Empty,
            "/participant/joycon/reset_ignored",
            self._on_reset_request,
            qos,
        )
        self._boost_pub = self.create_publisher(Float32MultiArray, "/awsim/cmd", qos)

    def _warn_throttled(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._last_warning.get(key, float("-inf")) >= 1.0:
            self.get_logger().warning(message)
            self._last_warning[key] = now

    def _on_status(self, message: Float32MultiArray) -> None:
        decision = self._guard.on_status(message.data, time.monotonic())
        if not decision.allowed:
            self._warn_throttled("status", f"Boost status rejected: {decision.reason}")

    def _on_state(self, message: String) -> None:
        self._guard.on_state(message.data)

    def _on_raw_boost(self, message: Float32MultiArray) -> None:
        high = bool(message.data) and math.isfinite(float(message.data[0])) and message.data[0] >= 1.0
        rising = high and not self._raw_high
        self._raw_high = high
        if not rising:
            return

        decision = self._guard.try_trigger(time.monotonic())
        if not decision.allowed:
            self._warn_throttled("trigger", f"Boost request blocked: {decision.reason}")
            return
        self._boost_pub.publish(Float32MultiArray(data=[1.0]))
        self._boost_pub.publish(Float32MultiArray(data=[0.0]))
        self.get_logger().info("Forwarded guarded Boost pulse to /awsim/cmd")

    def _on_reset_request(self, _: Empty) -> None:
        self._warn_throttled(
            "reset",
            "Participant reset is disabled: use Domain 0 AWSIM reset, then /set_initial_pose",
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = JoyconContractGuardNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
