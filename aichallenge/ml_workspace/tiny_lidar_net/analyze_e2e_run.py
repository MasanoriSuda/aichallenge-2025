#!/usr/bin/env python3
"""Summarize E2E motion and detect positive-acceleration stall from a ROS 2 bag."""

import argparse
import json
from pathlib import Path
import sys
from typing import Iterable, Optional, Tuple

import numpy as np


SCHEMA_VERSION = 1


def nearest_values(
    source_times_ns: np.ndarray,
    target_times_ns: np.ndarray,
    target_values: np.ndarray,
) -> np.ndarray:
    """Return the nearest target value for every source timestamp."""
    source = np.asarray(source_times_ns, dtype=np.int64)
    target = np.asarray(target_times_ns, dtype=np.int64)
    values = np.asarray(target_values, dtype=np.float64)
    if target.size == 0 or target.size != values.size:
        raise ValueError("target timestamps and values must be non-empty and aligned")

    insertion = np.clip(np.searchsorted(target, source), 0, target.size - 1)
    previous = np.clip(insertion - 1, 0, target.size - 1)
    use_previous = (
        np.abs(target[previous] - source) < np.abs(target[insertion] - source)
    )
    return values[np.where(use_previous, previous, insertion)]


def longest_true_interval_ns(
    times_ns: np.ndarray,
    mask: np.ndarray,
    max_sample_gap_sec: float = 0.25,
) -> Tuple[Optional[int], Optional[int]]:
    """Return the longest contiguous true interval without bridging bag gaps."""
    times = np.asarray(times_ns, dtype=np.int64)
    selected = np.asarray(mask, dtype=bool)
    if times.ndim != 1 or selected.ndim != 1 or times.size != selected.size:
        raise ValueError("times and mask must be aligned one-dimensional arrays")
    if times.size == 0:
        return None, None
    if np.any(np.diff(times) < 0):
        raise ValueError("timestamps must be monotonic")
    if not np.isfinite(max_sample_gap_sec) or max_sample_gap_sec <= 0.0:
        raise ValueError("max_sample_gap_sec must be finite and positive")

    gap_limit_ns = int(round(max_sample_gap_sec * 1e9))
    longest_start = None
    longest_end = None
    interval_start = None
    previous_time = None

    def retain(start: Optional[int], end: Optional[int]) -> None:
        nonlocal longest_start, longest_end
        if start is None or end is None:
            return
        current_duration = end - start
        best_duration = (
            -1 if longest_start is None else int(longest_end - longest_start)
        )
        if current_duration > best_duration:
            longest_start = int(start)
            longest_end = int(end)

    for timestamp, active in zip(times, selected):
        separated = (
            previous_time is not None and timestamp - previous_time > gap_limit_ns
        )
        if not active or separated:
            retain(interval_start, previous_time)
            interval_start = timestamp if active else None
        elif interval_start is None:
            interval_start = timestamp
        previous_time = timestamp

    retain(interval_start, previous_time)
    return longest_start, longest_end


def longest_true_duration_sec(
    times_ns: np.ndarray,
    mask: np.ndarray,
    max_sample_gap_sec: float = 0.25,
) -> float:
    """Measure the longest contiguous true interval without bridging bag gaps."""
    start, end = longest_true_interval_ns(times_ns, mask, max_sample_gap_sec)
    return 0.0 if start is None else float(end - start) / 1e9


def summarize_motion(
    velocity_times_ns: np.ndarray,
    speeds_mps: np.ndarray,
    control_times_ns: np.ndarray,
    accelerations_mps2: np.ndarray,
    moving_speed_mps: float,
    stall_speed_mps: float,
    positive_accel_mps2: float,
) -> dict:
    """Create motion metrics from independently sampled velocity and command data."""
    times = np.asarray(velocity_times_ns, dtype=np.int64)
    speeds = np.asarray(speeds_mps, dtype=np.float64)
    if times.size < 2 or times.size != speeds.size:
        raise ValueError("at least two aligned velocity samples are required")
    if np.any(~np.isfinite(speeds)) or np.any(np.diff(times) < 0):
        raise ValueError("velocity data must be finite and monotonic")

    acceleration = nearest_values(
        times,
        np.asarray(control_times_ns, dtype=np.int64),
        np.asarray(accelerations_mps2, dtype=np.float64),
    )
    moving = np.abs(speeds) >= moving_speed_mps
    ever_moved = np.maximum.accumulate(moving)
    low_speed = np.abs(speeds) <= stall_speed_mps
    positive_accel_stall = ever_moved & low_speed & (
        acceleration >= positive_accel_mps2
    )
    stall_start, stall_end = longest_true_interval_ns(times, positive_accel_stall)

    delta_sec = np.diff(times).astype(np.float64) / 1e9
    usable_delta = np.where(delta_sec <= 0.25, delta_sec, 0.0)
    distance_m = float(
        np.sum(np.maximum(speeds[:-1], 0.0) * usable_delta)
    )
    return {
        "duration_sec": float(times[-1] - times[0]) / 1e9,
        "distance_m": distance_m,
        "max_speed_mps": float(np.max(speeds)),
        "mean_forward_speed_mps": float(np.mean(np.maximum(speeds, 0.0))),
        "longest_low_speed_sec": longest_true_duration_sec(times, ever_moved & low_speed),
        "longest_positive_accel_stall_sec": longest_true_duration_sec(
            times, positive_accel_stall
        ),
        "positive_accel_stall_start_sec": (
            None if stall_start is None else float(stall_start - times[0]) / 1e9
        ),
        "positive_accel_stall_end_sec": (
            None if stall_end is None else float(stall_end - times[0]) / 1e9
        ),
        "positive_accel_stall_samples": int(np.count_nonzero(positive_accel_stall)),
    }


def scan_sector_minima(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    half_angle_rad: float,
    max_range_m: float,
) -> Tuple[float, float, float]:
    """Return front, left-front and right-front minimum ranges."""
    values = np.asarray(ranges, dtype=np.float64)
    if values.ndim != 1 or values.size == 0:
        raise ValueError("scan ranges must be a non-empty one-dimensional array")
    angles = angle_min + np.arange(values.size, dtype=np.float64) * angle_increment
    cleaned = np.nan_to_num(
        values, nan=0.0, posinf=max_range_m, neginf=0.0
    )
    cleaned = np.clip(cleaned, 0.0, max_range_m)

    def minimum(lower: float, upper: float) -> float:
        sector = cleaned[(angles >= lower) & (angles <= upper)]
        return float(np.min(sector)) if sector.size else float("nan")

    return (
        minimum(-half_angle_rad, half_angle_rad),
        minimum(half_angle_rad, 3.0 * half_angle_rad),
        minimum(-3.0 * half_angle_rad, -half_angle_rad),
    )


def scan_side_clearance(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    max_range_m: float,
) -> Tuple[float, float, float, float]:
    """Return left/right half-plane minima and closest return geometry."""
    values = np.asarray(ranges, dtype=np.float64)
    if values.ndim != 1 or values.size == 0:
        raise ValueError("scan ranges must be a non-empty one-dimensional array")
    angles = angle_min + np.arange(values.size, dtype=np.float64) * angle_increment
    cleaned = np.nan_to_num(
        values, nan=0.0, posinf=max_range_m, neginf=0.0
    )
    cleaned = np.clip(cleaned, 0.0, max_range_m)
    valid = cleaned > 0.0

    def minimum(mask: np.ndarray) -> float:
        sector = cleaned[mask & valid]
        return float(np.min(sector)) if sector.size else float("nan")

    closest_index = int(np.argmin(np.where(valid, cleaned, np.inf)))
    return (
        minimum(angles > 0.0),
        minimum(angles < 0.0),
        float(cleaned[closest_index]),
        float(angles[closest_index]),
    )


def read_bag(bag_path: Path, front_half_angle_rad: float, max_range_m: float) -> dict:
    """Read only the three topics required by the run admission audit."""
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError(
            "rosbags is required; run inside the development container or install "
            "aichallenge/ml_workspace/tiny_lidar_net/requirements.txt"
        ) from exc

    velocity_times = []
    speeds = []
    control_times = []
    accelerations = []
    scan_front = []
    scan_left = []
    scan_right = []
    scan_left_side = []
    scan_right_side = []
    scan_closest = []
    scan_closest_angle = []
    scan_times = []
    steering_commands = []
    pose_times = []
    pose_x = []
    pose_y = []
    required_topics = {
        "/vehicle/status/velocity_status",
        "/control/command/control_cmd",
        "/sensing/lidar/scan",
    }
    optional_topics = {"/localization/kinematic_state"}

    with AnyReader([bag_path]) as reader:
        available = {connection.topic for connection in reader.connections}
        missing = sorted(required_topics - available)
        if missing:
            raise ValueError(f"required topics missing from bag: {missing}")
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in required_topics | optional_topics
        ]
        for connection, timestamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            if connection.topic == "/vehicle/status/velocity_status":
                velocity_times.append(timestamp)
                speeds.append(float(message.longitudinal_velocity))
            elif connection.topic == "/control/command/control_cmd":
                control_times.append(timestamp)
                accelerations.append(float(message.longitudinal.acceleration))
                steering_commands.append(float(message.lateral.steering_tire_angle))
            elif connection.topic == "/sensing/lidar/scan":
                front, left, right = scan_sector_minima(
                    message.ranges,
                    float(message.angle_min),
                    float(message.angle_increment),
                    front_half_angle_rad,
                    max_range_m,
                )
                left_side, right_side, closest, closest_angle = scan_side_clearance(
                    message.ranges,
                    float(message.angle_min),
                    float(message.angle_increment),
                    max_range_m,
                )
                scan_times.append(timestamp)
                scan_front.append(front)
                scan_left.append(left)
                scan_right.append(right)
                scan_left_side.append(left_side)
                scan_right_side.append(right_side)
                scan_closest.append(closest)
                scan_closest_angle.append(closest_angle)
            elif connection.topic == "/localization/kinematic_state":
                pose_times.append(timestamp)
                pose_x.append(float(message.pose.pose.position.x))
                pose_y.append(float(message.pose.pose.position.y))

    return {
        "velocity_times_ns": np.asarray(velocity_times, dtype=np.int64),
        "speeds_mps": np.asarray(speeds, dtype=np.float64),
        "control_times_ns": np.asarray(control_times, dtype=np.int64),
        "accelerations_mps2": np.asarray(accelerations, dtype=np.float64),
        "steering_commands_rad": np.asarray(steering_commands, dtype=np.float64),
        "scan_times_ns": np.asarray(scan_times, dtype=np.int64),
        "scan_front_m": np.asarray(scan_front, dtype=np.float64),
        "scan_left_m": np.asarray(scan_left, dtype=np.float64),
        "scan_right_m": np.asarray(scan_right, dtype=np.float64),
        "scan_left_side_m": np.asarray(scan_left_side, dtype=np.float64),
        "scan_right_side_m": np.asarray(scan_right_side, dtype=np.float64),
        "scan_closest_m": np.asarray(scan_closest, dtype=np.float64),
        "scan_closest_angle_rad": np.asarray(scan_closest_angle, dtype=np.float64),
        "pose_times_ns": np.asarray(pose_times, dtype=np.int64),
        "pose_x_m": np.asarray(pose_x, dtype=np.float64),
        "pose_y_m": np.asarray(pose_y, dtype=np.float64),
    }


def nearest_sample_index(times_ns: np.ndarray, timestamp_ns: int) -> Optional[int]:
    """Return the nearest sample index, or None for an empty series."""
    times = np.asarray(times_ns, dtype=np.int64)
    if times.size == 0:
        return None
    insertion = int(np.clip(np.searchsorted(times, timestamp_ns), 0, times.size - 1))
    previous = max(0, insertion - 1)
    if abs(int(times[previous]) - timestamp_ns) < abs(
        int(times[insertion]) - timestamp_ns
    ):
        return previous
    return insertion


def failure_context(arrays: dict, timestamp_ns: int) -> dict:
    """Capture synchronized physical context at a failure timestamp."""
    context = {}
    scan_index = nearest_sample_index(arrays["scan_times_ns"], timestamp_ns)
    if scan_index is not None:
        context["scan"] = {
            "front_m": float(arrays["scan_front_m"][scan_index]),
            "left_front_m": float(arrays["scan_left_m"][scan_index]),
            "right_front_m": float(arrays["scan_right_m"][scan_index]),
            "left_side_m": float(arrays["scan_left_side_m"][scan_index]),
            "right_side_m": float(arrays["scan_right_side_m"][scan_index]),
            "closest_m": float(arrays["scan_closest_m"][scan_index]),
            "closest_angle_rad": float(
                arrays["scan_closest_angle_rad"][scan_index]
            ),
        }
    control_index = nearest_sample_index(arrays["control_times_ns"], timestamp_ns)
    if control_index is not None:
        context["command"] = {
            "acceleration_mps2": float(arrays["accelerations_mps2"][control_index]),
            "steering_rad": float(arrays["steering_commands_rad"][control_index]),
        }
    pose_index = nearest_sample_index(arrays["pose_times_ns"], timestamp_ns)
    if pose_index is not None:
        context["pose"] = {
            "x_m": float(arrays["pose_x_m"][pose_index]),
            "y_m": float(arrays["pose_y_m"][pose_index]),
        }
    return context


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS 2 bag directory or standalone MCAP")
    parser.add_argument("--output", type=Path, help="Optional JSON output path")
    parser.add_argument("--front-half-angle-rad", type=float, default=0.20)
    parser.add_argument("--max-range-m", type=float, default=30.0)
    parser.add_argument("--moving-speed-mps", type=float, default=1.0)
    parser.add_argument("--stall-speed-mps", type=float, default=0.15)
    parser.add_argument("--positive-accel-mps2", type=float, default=0.2)
    parser.add_argument("--max-positive-accel-stall-sec", type=float, default=5.0)
    parser.add_argument(
        "--fail-on-stall",
        action="store_true",
        help="Return exit status 2 when the positive-acceleration stall limit is exceeded",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    arrays = read_bag(bag, args.front_half_angle_rad, args.max_range_m)
    motion = summarize_motion(
        arrays["velocity_times_ns"],
        arrays["speeds_mps"],
        arrays["control_times_ns"],
        arrays["accelerations_mps2"],
        args.moving_speed_mps,
        args.stall_speed_mps,
        args.positive_accel_mps2,
    )
    scan_front = arrays["scan_front_m"]
    scan = {
        "sample_count": int(scan_front.size),
        "front_min_m": float(np.min(scan_front)),
        "front_p05_m": float(np.percentile(scan_front, 5)),
        "left_front_min_m": float(np.min(arrays["scan_left_m"])),
        "right_front_min_m": float(np.min(arrays["scan_right_m"])),
    }
    stalled = (
        motion["longest_positive_accel_stall_sec"]
        > args.max_positive_accel_stall_sec
    )
    stall_context = None
    stall_start_sec = motion["positive_accel_stall_start_sec"]
    if stall_start_sec is not None:
        stall_start_ns = int(
            arrays["velocity_times_ns"][0] + round(stall_start_sec * 1e9)
        )
        stall_context = {
            "start": failure_context(arrays, stall_start_ns),
            "end": failure_context(
                arrays,
                int(arrays["velocity_times_ns"][-1]),
            ),
        }
    result = {
        "schema_version": SCHEMA_VERSION,
        "source_bag": str(bag),
        "thresholds": {
            "moving_speed_mps": args.moving_speed_mps,
            "stall_speed_mps": args.stall_speed_mps,
            "positive_accel_mps2": args.positive_accel_mps2,
            "max_positive_accel_stall_sec": args.max_positive_accel_stall_sec,
        },
        "motion": motion,
        "scan": scan,
        "positive_accel_stall_context": stall_context,
        "admission": {
            "positive_accel_stall": not stalled,
            "result": "fail" if stalled else "pass",
        },
    }
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 2 if args.fail_on_stall and stalled else 0


if __name__ == "__main__":
    sys.exit(main())
