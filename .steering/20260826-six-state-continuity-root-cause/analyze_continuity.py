#!/usr/bin/env python3

import argparse
import bisect
import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


TOPICS = {
    "/control/command/control_cmd",
    "/vehicle/status/steering_status",
    "/localization/kinematic_state",
}


def stamp_seconds(stamp):
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def read_samples(bag, start, end):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
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
        if record_time > end:
            break
        if topic not in TOPICS or record_time < start:
            continue
        message = deserialize_message(payload, types[topic])
        if topic == "/control/command/control_cmd":
            value = (
                stamp_seconds(message.stamp),
                float(message.lateral.steering_tire_angle),
                float(message.longitudinal.speed),
                float(message.longitudinal.acceleration),
            )
        elif topic == "/vehicle/status/steering_status":
            value = (
                stamp_seconds(message.stamp),
                float(message.steering_tire_angle),
            )
        else:
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
        samples[topic].append((record_time, value))
    return samples


def nearest(values, target):
    times = [item[0] for item in values]
    index = bisect.bisect_left(times, target)
    candidates = values[max(0, index - 1):min(len(values), index + 1)]
    return min(candidates, key=lambda item: abs(item[0] - target))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--end", type=float, required=True)
    parser.add_argument("--step", type=float, default=0.025)
    args = parser.parse_args()
    samples = read_samples(args.bag, args.start, args.end)
    print(
        "record_t command_t command_delta command_v command_a "
        "steering_t measured_delta odom_t odom_v yaw_rate x y yaw"
    )
    target = args.start
    while target <= args.end + 1.0e-9:
        command_record, command = nearest(
            samples["/control/command/control_cmd"], target
        )
        steering_record, steering = nearest(
            samples["/vehicle/status/steering_status"], target
        )
        odom_record, odom = nearest(
            samples["/localization/kinematic_state"], target
        )
        print(
            f"{target:.6f} {command[0]:.6f} {command[1]:.6f} "
            f"{command[2]:.6f} {command[3]:.6f} {steering[0]:.6f} "
            f"{steering[1]:.6f} {odom[0]:.6f} {odom[4]:.6f} "
            f"{odom[5]:.6f} {odom[1]:.6f} {odom[2]:.6f} {odom[3]:.6f}"
        )
        target += args.step
    print("counts", {topic: len(values) for topic, values in samples.items()})


if __name__ == "__main__":
    main()
