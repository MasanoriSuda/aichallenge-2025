#!/usr/bin/env python3

import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = "/output/20260825-235153/d1/rosbag2_autoware"
START = 1787669555.4
END = 1787669559.1
TOPICS = {
    "/control/command/control_cmd",
    "/vehicle/status/steering_status",
    "/localization/kinematic_state",
    "/sensing/imu/imu_raw",
}


def stamp_seconds(stamp):
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
    rosbag2_py.ConverterOptions("", ""),
)
types = {
    topic.name: get_message(topic.type)
    for topic in reader.get_all_topics_and_types()
    if topic.name in TOPICS
}
samples = {topic: [] for topic in TOPICS}
while reader.has_next():
    topic, payload, timestamp_ns = reader.read_next()
    record_time = 1.0e-9 * timestamp_ns
    if record_time > END:
        break
    if topic not in TOPICS or record_time < START:
        continue
    message = deserialize_message(payload, types[topic])
    if topic == "/control/command/control_cmd":
        value = (
            stamp_seconds(message.stamp),
            float(message.lateral.steering_tire_angle),
            float(message.lateral.steering_tire_rotation_rate),
            float(message.longitudinal.speed),
            float(message.longitudinal.acceleration),
        )
    elif topic == "/vehicle/status/steering_status":
        value = (stamp_seconds(message.stamp), float(message.steering_tire_angle))
    elif topic == "/localization/kinematic_state":
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        value = (
            stamp_seconds(message.header.stamp),
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            yaw,
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
        )
    else:
        value = (
            stamp_seconds(message.header.stamp),
            float(message.angular_velocity.z),
        )
    samples[topic].append((record_time, value))


def nearest(topic, target):
    return min(samples[topic], key=lambda item: abs(item[0] - target))[1]


print("record_t sim_t cmd_delta cmd_rate cmd_v cmd_a actual_delta odom_v odom_yawrate imu_yawrate x y yaw")
target = START
while target <= END + 1.0e-9:
    command = nearest("/control/command/control_cmd", target)
    steering = nearest("/vehicle/status/steering_status", target)
    odom = nearest("/localization/kinematic_state", target)
    imu = nearest("/sensing/imu/imu_raw", target)
    print(
        f"{target:.3f} {command[0]:.3f} {command[1]:.6f} {command[2]:.6f} "
        f"{command[3]:.6f} {command[4]:.6f} {steering[1]:.6f} "
        f"{odom[4]:.6f} {odom[5]:.6f} {imu[1]:.6f} "
        f"{odom[1]:.6f} {odom[2]:.6f} {odom[3]:.6f}"
    )
    target += 0.1

print("counts", {topic: len(values) for topic, values in samples.items()})
