#!/usr/bin/env python3
"""Publish explicit empty-world V2X observations for one-vehicle simulation."""

import rclpy
from rclpy.node import Node
from v2x_msgs.msg import V2XVehiclePositionArray


PUBLISH_PERIOD_SEC = 0.05


class SingleVehicleEmptyV2XPublisher(Node):
    """Certify that a single-vehicle scenario has no peer vehicles."""

    def __init__(self) -> None:
        super().__init__("single_vehicle_empty_v2x_publisher")
        self.publisher = self.create_publisher(
            V2XVehiclePositionArray, "/v2x/vehicle_positions", 1)
        self.timer = self.create_timer(
            PUBLISH_PERIOD_SEC, self.publish_observation)
        self.get_logger().info(
            "Publishing explicit empty V2X observations for "
            "a single-vehicle simulation")

    def publish_observation(self) -> None:
        message = V2XVehiclePositionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.vehicles = []
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SingleVehicleEmptyV2XPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
