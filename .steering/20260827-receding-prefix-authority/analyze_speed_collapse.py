#!/usr/bin/env python3
"""Correlate abrupt speed loss with command, response, IMU, and course pose.

This is an evidence tool for the Slice-6 dynamic gate.  It intentionally reads
the recorded wire topics directly so a controller command and an external
physical impulse are not conflated by nearest-neighbour resampling.
"""

from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any, Callable


@dataclass(frozen=True)
class TimedValue:
    time_sec: float
    value: dict[str, float]


def yaw_from_quaternion(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def nearest(series: list[TimedValue], time_sec: float) -> TimedValue:
    index = bisect_left([sample.time_sec for sample in series], time_sec)
    index = min(max(index, 0), len(series) - 1)
    if index > 0 and (
        abs(series[index - 1].time_sec - time_sec)
        < abs(series[index].time_sec - time_sec)
    ):
        index -= 1
    return series[index]


def trajectory_cross_track(
    state: TimedValue, trajectory: list[tuple[float, float, float]]
) -> tuple[float, float, float]:
    best = (math.inf, math.nan, math.nan)
    for first, second in zip(trajectory, trajectory[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-9:
            continue
        along = (
            (state.value["x"] - first[0]) * dx
            + (state.value["y"] - first[1]) * dy
        ) / length_squared
        along = min(max(along, 0.0), 1.0)
        offset_x = state.value["x"] - (first[0] + along * dx)
        offset_y = state.value["y"] - (first[1] + along * dy)
        distance = math.hypot(offset_x, offset_y)
        if distance < best[0]:
            segment_yaw = math.atan2(dy, dx)
            signed = (dx * offset_y - dy * offset_x) / math.sqrt(length_squared)
            best = (distance, signed, segment_yaw)
    return best


def read_bag(path: Path) -> tuple[dict[str, list[TimedValue]], list[tuple[float, float, float]]]:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    extractors: dict[str, tuple[str, Callable[[Any], dict[str, float]]]] = {
        "/localization/kinematic_state": (
            "state",
            lambda message: {
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": yaw_from_quaternion(message.pose.pose.orientation),
                "speed": float(message.twist.twist.linear.x),
                "yaw_rate": float(message.twist.twist.angular.z),
            },
        ),
        "/vehicle/status/velocity_status": (
            "vehicle",
            lambda message: {
                "speed": float(message.longitudinal_velocity),
                "yaw_rate": float(message.heading_rate),
            },
        ),
        "/vehicle/status/steering_status": (
            "steering",
            lambda message: {"steering": float(message.steering_tire_angle)},
        ),
        "/control/command/control_cmd": (
            "command",
            lambda message: {
                "speed": float(message.longitudinal.speed),
                "acceleration": float(message.longitudinal.acceleration),
                "steering": float(message.lateral.steering_tire_angle),
            },
        ),
        "/localization/acceleration": (
            "acceleration",
            lambda message: {
                "x": float(message.accel.accel.linear.x),
                "y": float(message.accel.accel.linear.y),
            },
        ),
        "/sensing/imu/imu_raw": (
            "imu_raw",
            lambda message: {
                "accel_x": float(message.linear_acceleration.x),
                "accel_y": float(message.linear_acceleration.y),
                "yaw_rate": float(message.angular_velocity.z),
            },
        ),
        "/sensing/imu/imu_data": (
            "imu",
            lambda message: {
                "accel_x": float(message.linear_acceleration.x),
                "accel_y": float(message.linear_acceleration.y),
                "yaw_rate": float(message.angular_velocity.z),
            },
        ),
    }

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(path), storage_id="mcap"),
        ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(topic_types[topic])
        for topic in extractors
        if topic in topic_types
    }
    trajectory_type = get_message(topic_types["/planning/scenario_planning/trajectory"])
    series = {name: [] for name, _ in extractors.values()}
    trajectory: list[tuple[float, float, float]] = []
    first_stamp_ns: int | None = None
    pending: list[tuple[str, int, dict[str, float]]] = []
    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        first_stamp_ns = stamp_ns if first_stamp_ns is None else min(first_stamp_ns, stamp_ns)
        if topic in message_types:
            message = deserialize_message(serialized, message_types[topic])
            name, extractor = extractors[topic]
            pending.append((name, stamp_ns, extractor(message)))
        elif topic == "/planning/scenario_planning/trajectory" and not trajectory:
            message = deserialize_message(serialized, trajectory_type)
            trajectory = [
                (
                    float(point.pose.position.x),
                    float(point.pose.position.y),
                    yaw_from_quaternion(point.pose.orientation),
                )
                for point in message.points
            ]
    origin = float(first_stamp_ns or 0) * 1e-9
    for name, stamp_ns, value in pending:
        series[name].append(TimedValue(float(stamp_ns) * 1e-9 - origin, value))
    return series, trajectory


def main() -> None:
    series, trajectory = read_bag(Path(sys.argv[1]))
    states = series["state"]
    raw_events: list[tuple[TimedValue, float, float]] = []
    for previous, current in zip(states, states[1:]):
        dt = current.time_sec - previous.time_sec
        loss = previous.value["speed"] - current.value["speed"]
        if 0.0 < dt <= 0.060 and loss >= 1.0:
            raw_events.append((current, loss, dt))

    events: list[tuple[TimedValue, float, float]] = []
    for event in raw_events:
        if events and event[0].time_sec - events[-1][0].time_sec < 0.30:
            continue
        events.append(event)

    print(
        "event time loss dt x y yaw speed_before speed_after cmd_age "
        "cmd_v cmd_a cmd_delta vehicle_v loc_ax imu_ax imu_ay cross_track signed yaw_error"
    )
    for index, (state, loss, dt) in enumerate(events, start=1):
        before = nearest(states, state.time_sec - dt)
        command = nearest(series["command"], state.time_sec - 0.030)
        vehicle = nearest(series["vehicle"], state.time_sec)
        acceleration = nearest(series["acceleration"], state.time_sec)
        imu = nearest(series["imu"], state.time_sec)
        cross_track, signed, reference_yaw = trajectory_cross_track(state, trajectory)
        yaw_error = math.atan2(
            math.sin(state.value["yaw"] - reference_yaw),
            math.cos(state.value["yaw"] - reference_yaw),
        )
        print(
            f"{index} {state.time_sec:.3f} {loss:.3f} {dt:.4f} "
            f"{state.value['x']:.3f} {state.value['y']:.3f} {state.value['yaw']:.3f} "
            f"{before.value['speed']:.3f} {state.value['speed']:.3f} "
            f"{state.time_sec - command.time_sec:+.4f} "
            f"{command.value['speed']:.3f} {command.value['acceleration']:.3f} "
            f"{command.value['steering']:.4f} {vehicle.value['speed']:.3f} "
            f"{acceleration.value['x']:.1f} {imu.value['accel_x']:.1f} "
            f"{imu.value['accel_y']:.1f} {cross_track:.3f} {signed:.3f} {yaw_error:.3f}"
        )

    reference_x = events[0][0].value["x"] if events else 89615.344
    reference_y = events[0][0].value["y"] if events else 43164.912
    local_minima: list[TimedValue] = []
    for previous, current, following in zip(states, states[1:], states[2:]):
        previous_distance = math.hypot(
            previous.value["x"] - reference_x, previous.value["y"] - reference_y
        )
        current_distance = math.hypot(
            current.value["x"] - reference_x, current.value["y"] - reference_y
        )
        following_distance = math.hypot(
            following.value["x"] - reference_x, following.value["y"] - reference_y
        )
        if current_distance <= 2.0 and current_distance <= previous_distance and current_distance < following_distance:
            if not local_minima or current.time_sec - local_minima[-1].time_sec >= 20.0:
                local_minima.append(current)

    print("pass time distance speed command_a steering cross_track signed yaw_error")
    for index, state in enumerate(local_minima, start=1):
        command = nearest(series["command"], state.time_sec - 0.030)
        cross_track, signed, reference_yaw = trajectory_cross_track(state, trajectory)
        yaw_error = math.atan2(
            math.sin(state.value["yaw"] - reference_yaw),
            math.cos(state.value["yaw"] - reference_yaw),
        )
        distance = math.hypot(
            state.value["x"] - reference_x, state.value["y"] - reference_y
        )
        print(
            f"{index} {state.time_sec:.3f} {distance:.3f} {state.value['speed']:.3f} "
            f"{command.value['acceleration']:.3f} {command.value['steering']:.4f} "
            f"{cross_track:.3f} {signed:.3f} {yaw_error:.3f}"
        )


if __name__ == "__main__":
    main()
