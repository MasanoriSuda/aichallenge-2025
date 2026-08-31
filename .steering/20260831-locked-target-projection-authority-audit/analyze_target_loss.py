#!/usr/bin/env python3
"""Print raw d2 geometry at the frozen target-loss epoch."""

import math

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


BAG = "/output/20260831-173350/d1/rosbag2_autoware/rosbag2_autoware_0.mcap"
START_SEC = 1788165276.45
END_SEC = 1788165277.85


reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(uri=BAG, storage_id="mcap"),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    ),
)
types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
odom_type = get_message(types["/localization/kinematic_state"])
v2x_type = get_message(types["/v2x/vehicle_positions"])
trajectory_type = get_message(types["/planning/scenario_planning/trajectory"])
ego = None
course = []
course_progress = []


def update_course(message):
    global course, course_progress
    course = [
        (point.pose.position.x, point.pose.position.y) for point in message.points
    ]
    course_progress = [0.0]
    for index in range(1, len(course)):
        course_progress.append(
            course_progress[-1]
            + math.hypot(
                course[index][0] - course[index - 1][0],
                course[index][1] - course[index - 1][1],
            )
        )


def nearest_course_progress(x, y):
    if not course:
        return math.nan, math.inf, -1
    index, distance = min(
        enumerate(math.hypot(px - x, py - y) for px, py in course),
        key=lambda item: item[1],
    )
    return course_progress[index], distance, index

while reader.has_next():
    topic, data, timestamp_ns = reader.read_next()
    timestamp_sec = timestamp_ns * 1.0e-9
    if timestamp_sec > END_SEC:
        break
    if topic == "/localization/kinematic_state":
        message = deserialize_message(data, odom_type)
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0
            * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0
            - 2.0
            * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        ego = (message.pose.pose.position.x, message.pose.pose.position.y, yaw)
    elif topic == "/planning/scenario_planning/trajectory":
        update_course(deserialize_message(data, trajectory_type))
    elif (
        topic == "/v2x/vehicle_positions"
        and START_SEC <= timestamp_sec <= END_SEC
        and ego is not None
    ):
        message = deserialize_message(data, v2x_type)
        for vehicle in message.vehicles:
            if vehicle.vehicle_id != "d2":
                continue
            dx = vehicle.position.x - ego[0]
            dy = vehicle.position.y - ego[1]
            longitudinal = math.cos(ego[2]) * dx + math.sin(ego[2]) * dy
            lateral = -math.sin(ego[2]) * dx + math.cos(ego[2]) * dy
            ego_progress, ego_course_distance, ego_index = nearest_course_progress(
                ego[0], ego[1]
            )
            target_progress, target_course_distance, target_index = (
                nearest_course_progress(vehicle.position.x, vehicle.position.y)
            )
            forward_progress = target_progress - ego_progress
            if course_progress and forward_progress < 0.0:
                forward_progress += course_progress[-1]
            print(
                f"{timestamp_sec:.6f} "
                f"ego=({ego[0]:.3f},{ego[1]:.3f},{ego[2]:.3f}) "
                f"d2=({vehicle.position.x:.3f},{vehicle.position.y:.3f}) "
                f"euclid={math.hypot(dx, dy):.3f} "
                f"local=({longitudinal:.3f},{lateral:.3f}) "
                f"course=({ego_index}:{ego_course_distance:.3f},"
                f"{target_index}:{target_course_distance:.3f},"
                f"forward:{forward_progress:.3f})"
            )
