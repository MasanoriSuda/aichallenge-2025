#!/usr/bin/env python3
"""Compare repeated passages through the WP72--76 physical-event curve."""

from __future__ import annotations

from bisect import bisect_left, bisect_right
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any


@dataclass(frozen=True)
class Sample:
    time_sec: float
    source_time_sec: float
    values: dict[str, float]


def yaw(quaternion: Any) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def roll_pitch(quaternion: Any) -> tuple[float, float]:
    roll = math.atan2(
        2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
        1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y),
    )
    pitch_term = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x)
    pitch = math.asin(min(max(pitch_term, -1.0), 1.0))
    return roll, pitch


def nearest(series: list[Sample], time_sec: float) -> Sample | None:
    if not series:
        return None
    times = [sample.time_sec for sample in series]
    index = min(max(bisect_left(times, time_sec), 0), len(series) - 1)
    if index > 0 and abs(series[index - 1].time_sec - time_sec) < abs(series[index].time_sec - time_sec):
        index -= 1
    return series[index]


def nearest_source(series: list[Sample], source_time_sec: float) -> Sample | None:
    """Find a sample by producer/header time instead of bag arrival time."""
    if not series or not math.isfinite(source_time_sec):
        return None
    candidates = [sample for sample in series if math.isfinite(sample.source_time_sec)]
    if not candidates:
        return None
    return min(candidates, key=lambda sample: abs(sample.source_time_sec - source_time_sec))


def source_stamp(message: Any) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return math.nan
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def previous(series: list[Sample], time_sec: float) -> Sample | None:
    if not series:
        return None
    times = [sample.time_sec for sample in series]
    index = bisect_right(times, time_sec) - 1
    return series[index] if index >= 0 else None


def steering_window_metrics(series: list[Sample], time_sec: float) -> tuple[float, float, float, float]:
    window = [sample for sample in series if time_sec - 1.0 <= sample.time_sec <= time_sec]
    values = [sample.values["steering"] for sample in window]
    if len(values) < 2:
        return (math.nan, math.nan, math.nan, math.nan)
    rates = [
        abs(after.values["steering"] - before.values["steering"])
        / max(after.time_sec - before.time_sec, 1e-9)
        for before, after in zip(window, window[1:])
    ]
    total_variation = sum(
        abs(after - before) for before, after in zip(values, values[1:])
    )
    return min(values), max(values), max(rates, default=0.0), total_variation


def read_bag(path: Path) -> tuple[dict[str, list[Sample]], list[tuple[float, float]]]:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(path), storage_id="mcap"),
        ConverterOptions(input_serialization_format="", output_serialization_format=""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topics = {
        topic: get_message(types[topic])
        for topic in (
            "/localization/kinematic_state",
            "/control/command/control_cmd",
            "/localization/acceleration",
            "/localization/imu_gnss_poser/pose_with_covariance",
            "/sensing/gnss/pose_with_covariance",
            "/vehicle/status/steering_status",
            "/vehicle/status/velocity_status",
            "/sensing/imu/imu_raw",
        )
        if topic in types
    }
    trajectory_type = (
        get_message(types["/planning/scenario_planning/trajectory"])
        if "/planning/scenario_planning/trajectory" in types
        else None
    )
    pending: dict[str, list[tuple[int, float, dict[str, float]]]] = {
        "state": [], "command": [], "acceleration": [], "gnss_pose": [],
        "raw_gnss_pose": [], "steering_report": [], "velocity_report": [], "imu": []
    }
    trajectory: list[tuple[float, float]] = []
    origin_ns: int | None = None
    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        origin_ns = stamp_ns if origin_ns is None else min(origin_ns, stamp_ns)
        if topic not in topics and not (
            topic == "/planning/scenario_planning/trajectory" and trajectory_type is not None
        ):
            continue
        if topic == "/localization/kinematic_state":
            message = deserialize_message(serialized, topics[topic])
            pending["state"].append((stamp_ns, source_stamp(message), {
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": yaw(message.pose.pose.orientation),
                "speed": float(message.twist.twist.linear.x),
                "yaw_rate": float(message.twist.twist.angular.z),
            }))
        elif topic == "/control/command/control_cmd":
            message = deserialize_message(serialized, topics[topic])
            pending["command"].append((stamp_ns, source_stamp(message), {
                "acceleration": float(message.longitudinal.acceleration),
                "steering": float(message.lateral.steering_tire_angle),
            }))
        elif topic == "/localization/acceleration":
            message = deserialize_message(serialized, topics[topic])
            pending["acceleration"].append((stamp_ns, source_stamp(message), {
                "x": float(message.accel.accel.linear.x),
                "y": float(message.accel.accel.linear.y),
            }))
        elif topic == "/localization/imu_gnss_poser/pose_with_covariance":
            message = deserialize_message(serialized, topics[topic])
            pending["gnss_pose"].append((stamp_ns, source_stamp(message), {
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": yaw(message.pose.pose.orientation),
            }))
        elif topic == "/sensing/gnss/pose_with_covariance":
            message = deserialize_message(serialized, topics[topic])
            pending["raw_gnss_pose"].append((stamp_ns, source_stamp(message), {
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": yaw(message.pose.pose.orientation),
            }))
        elif topic == "/vehicle/status/steering_status":
            message = deserialize_message(serialized, topics[topic])
            pending["steering_report"].append((stamp_ns, source_stamp(message), {
                "steering": float(message.steering_tire_angle),
            }))
        elif topic == "/vehicle/status/velocity_status":
            message = deserialize_message(serialized, topics[topic])
            pending["velocity_report"].append((stamp_ns, source_stamp(message), {
                "speed": float(message.longitudinal_velocity),
            }))
        elif topic == "/sensing/imu/imu_raw":
            message = deserialize_message(serialized, topics[topic])
            roll, pitch = roll_pitch(message.orientation)
            pending["imu"].append((stamp_ns, source_stamp(message), {
                "roll": roll,
                "pitch": pitch,
                "wx": float(message.angular_velocity.x),
                "wy": float(message.angular_velocity.y),
                "wz": float(message.angular_velocity.z),
                "ax": float(message.linear_acceleration.x),
                "ay": float(message.linear_acceleration.y),
                "az": float(message.linear_acceleration.z),
            }))
        elif not trajectory:
            message = deserialize_message(serialized, trajectory_type)
            trajectory = [
                (float(point.pose.position.x), float(point.pose.position.y))
                for point in message.points
            ]
    origin = float(origin_ns or 0) * 1e-9
    return {
        name: [
            Sample(stamp_ns * 1e-9 - origin, source_time_sec, sample_values)
            for stamp_ns, source_time_sec, sample_values in values
        ]
        for name, values in pending.items()
    }, trajectory


def segment_projection(
    sample: Sample, path: list[tuple[float, float]]
) -> tuple[float, float, float, int]:
    best = (math.inf, math.nan, math.nan, -1)
    for index, (first, second) in enumerate(zip(path, path[1:])):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-9:
            continue
        along = (
            (sample.values["x"] - first[0]) * dx
            + (sample.values["y"] - first[1]) * dy
        ) / length_squared
        along = min(max(along, 0.0), 1.0)
        offset_x = sample.values["x"] - (first[0] + along * dx)
        offset_y = sample.values["y"] - (first[1] + along * dy)
        distance = math.hypot(offset_x, offset_y)
        if distance < best[0]:
            signed = (dx * offset_y - dy * offset_x) / math.sqrt(length_squared)
            best = (distance, signed, math.atan2(dy, dx), index)
    return best


def passages(states: list[Sample], path: list[tuple[float, float]]) -> list[list[Sample]]:
    groups: list[list[Sample]] = []
    active: list[Sample] = []
    last_near_time = -math.inf
    for state in states:
        distance = segment_projection(state, path)[0]
        if distance <= 3.0:
            if active and state.time_sec - last_near_time > 1.0:
                groups.append(active)
                active = []
            active.append(state)
            last_near_time = state.time_sec
        elif active and state.time_sec - last_near_time > 1.0:
            groups.append(active)
            active = []
    if active:
        groups.append(active)
    return [group for group in groups if len(group) >= 5]


def main() -> None:
    if len(sys.argv) < 3:
        raise SystemExit("usage: compare_curve_passages.py REFERENCE_BAG BAG...")
    reference_series, full_path = read_bag(Path(sys.argv[1]))
    del reference_series
    event_x, event_y = 89615.344, 43164.912
    center = min(
        range(len(full_path)),
        key=lambda index: math.hypot(full_path[index][0] - event_x, full_path[index][1] - event_y),
    )
    begin = max(0, center - 6)
    end = min(len(full_path), center + 12)
    path = full_path[begin:end]
    stations = [0, len(path) // 4, len(path) // 2, 3 * len(path) // 4, len(path) - 1]
    print(f"reference_indices={begin}..{end - 1} points={len(path)}")
    for bag_name in sys.argv[2:]:
        series, _ = read_bag(Path(bag_name))
        print(f"bag={bag_name}")
        print(
            "pass start duration min_v max_v max_abs_ay max_abs_loc_a drop "
            "signed_min signed_max yaw_error_max"
        )
        for pass_index, group in enumerate(passages(series["state"], path), start=1):
            commands = [nearest(series["command"], state.time_sec) for state in group]
            accelerations = [nearest(series["acceleration"], state.time_sec) for state in group]
            projections = [segment_projection(state, path) for state in group]
            speeds = [state.values["speed"] for state in group]
            drops = [before - after for before, after in zip(speeds, speeds[1:])]
            yaw_errors = [
                abs(math.atan2(math.sin(state.values["yaw"] - projection[2]), math.cos(state.values["yaw"] - projection[2])))
                for state, projection in zip(group, projections)
            ]
            lateral_accelerations = [
                abs(state.values["speed"] * state.values["yaw_rate"]) for state in group
            ]
            localization_accelerations = [
                abs(sample.values["x"]) for sample in accelerations if sample is not None
            ]
            print(
                f"{pass_index} {group[0].time_sec:.3f} "
                f"{group[-1].time_sec - group[0].time_sec:.3f} "
                f"{min(speeds):.3f} {max(speeds):.3f} "
                f"{max(lateral_accelerations):.2f} "
                f"{max(localization_accelerations, default=math.nan):.1f} "
                f"{max(drops, default=0.0):.3f} "
                f"{min(value[1] for value in projections):.3f} "
                f"{max(value[1] for value in projections):.3f} "
                f"{max(yaw_errors):.3f}"
            )
            abrupt = []
            for before, after, projection in zip(group, group[1:], projections[1:]):
                speed_loss = before.values["speed"] - after.values["speed"]
                if speed_loss < 1.0:
                    continue
                yaw_error = math.atan2(
                    math.sin(after.values["yaw"] - projection[2]),
                    math.cos(after.values["yaw"] - projection[2]),
                )
                abrupt.append(
                    f"t={after.time_sec:.3f}/loss={speed_loss:.3f}/"
                    f"xy=({after.values['x']:.3f},{after.values['y']:.3f})/"
                    f"segment={begin + projection[3]}/d={projection[1]:+.3f}/"
                    f"ye={yaw_error:+.3f}"
                )
            if abrupt:
                print("  abrupt " + " ".join(abrupt))
            event_state = min(
                group,
                key=lambda item: math.hypot(
                    item.values["x"] - event_x, item.values["y"] - event_y),
            )
            event_projection = segment_projection(event_state, path)
            event_command = previous(series["command"], event_state.time_sec)
            event_gnss = nearest(series["gnss_pose"], event_state.time_sec)
            event_raw_gnss = nearest_source(
                series["raw_gnss_pose"], event_state.source_time_sec)
            event_steering = nearest(series["steering_report"], event_state.time_sec)
            event_velocity = nearest(series["velocity_report"], event_state.time_sec)
            event_imu = nearest(series["imu"], event_state.time_sec)
            command_window = steering_window_metrics(
                series["command"], event_state.time_sec)
            report_window = steering_window_metrics(
                series["steering_report"], event_state.time_sec)
            event_yaw_error = math.atan2(
                math.sin(event_state.values["yaw"] - event_projection[2]),
                math.cos(event_state.values["yaw"] - event_projection[2]),
            )
            print(
                "  event_nearest "
                f"t={event_state.time_sec:.3f}/v={event_state.values['speed']:.3f}/"
                f"distance={math.hypot(event_state.values['x'] - event_x, event_state.values['y'] - event_y):.3f}/"
                f"xy=({event_state.values['x']:.3f},{event_state.values['y']:.3f})/"
                f"segment={begin + event_projection[3]}/d={event_projection[1]:+.3f}/"
                f"yaw={event_state.values['yaw']:+.3f}/ye={event_yaw_error:+.3f}/"
                f"yr={event_state.values['yaw_rate']:+.3f}/"
                f"ay={event_state.values['speed'] * event_state.values['yaw_rate']:+.3f}/"
                f"a={(event_command.values['acceleration'] if event_command else math.nan):+.2f}/"
                f"steer={(event_command.values['steering'] if event_command else math.nan):+.3f}/"
                f"reported_steer={(event_steering.values['steering'] if event_steering else math.nan):+.3f}/"
                f"reported_v={(event_velocity.values['speed'] if event_velocity else math.nan):.3f}/"
                f"gnss_delta={math.hypot(event_gnss.values['x'] - event_state.values['x'], event_gnss.values['y'] - event_state.values['y']) if event_gnss else math.nan:.3f}/"
                f"gnss_xy=({event_gnss.values['x'] if event_gnss else math.nan:.3f},"
                f"{event_gnss.values['y'] if event_gnss else math.nan:.3f})/"
                f"raw_gnss_header_delta={math.hypot(event_raw_gnss.values['x'] - event_state.values['x'], event_raw_gnss.values['y'] - event_state.values['y']) if event_raw_gnss else math.nan:.3f}/"
                f"raw_gnss_header_dt={(event_raw_gnss.source_time_sec - event_state.source_time_sec) if event_raw_gnss else math.nan:+.4f}/"
                f"raw_gnss_xy=({event_raw_gnss.values['x'] if event_raw_gnss else math.nan:.3f},"
                f"{event_raw_gnss.values['y'] if event_raw_gnss else math.nan:.3f})/"
                f"cmd_window=[{command_window[0]:+.3f},{command_window[1]:+.3f}]"
                f"/rate={command_window[2]:.3f}/tv={command_window[3]:.3f}/"
                f"report_window=[{report_window[0]:+.3f},{report_window[1]:+.3f}]"
                f"/rate={report_window[2]:.3f}/tv={report_window[3]:.3f}/"
                f"imu_rp=({event_imu.values['roll'] if event_imu else math.nan:+.3f},"
                f"{event_imu.values['pitch'] if event_imu else math.nan:+.3f})/"
                f"imu_w=({event_imu.values['wx'] if event_imu else math.nan:+.2f},"
                f"{event_imu.values['wy'] if event_imu else math.nan:+.2f},"
                f"{event_imu.values['wz'] if event_imu else math.nan:+.2f})/"
                f"imu_a=({event_imu.values['ax'] if event_imu else math.nan:+.1f},"
                f"{event_imu.values['ay'] if event_imu else math.nan:+.1f},"
                f"{event_imu.values['az'] if event_imu else math.nan:+.1f})"
            )
            details = []
            for station in stations:
                target = path[station]
                state = min(
                    group,
                    key=lambda item: math.hypot(item.values["x"] - target[0], item.values["y"] - target[1]),
                )
                projection = segment_projection(state, path)
                command = nearest(series["command"], state.time_sec)
                details.append(
                    f"s{station}:t={state.time_sec:.2f}/v={state.values['speed']:.2f}/"
                    f"d={projection[1]:+.2f}/ye={math.atan2(math.sin(state.values['yaw'] - projection[2]), math.cos(state.values['yaw'] - projection[2])):+.2f}/"
                    f"a={(command.values['acceleration'] if command else math.nan):+.2f}/"
                    f"steer={(command.values['steering'] if command else math.nan):+.2f}"
                )
            print("  " + " ".join(details))


if __name__ == "__main__":
    main()
