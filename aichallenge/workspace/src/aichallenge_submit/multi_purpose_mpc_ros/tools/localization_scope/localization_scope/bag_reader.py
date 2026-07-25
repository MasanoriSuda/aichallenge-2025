"""ROS bag reading and message normalization.

ROS imports are intentionally local to :func:`read_bag` so metadata and report
tests can run outside a sourced ROS environment.
"""

from __future__ import annotations

from collections import Counter
import math
from pathlib import Path
from typing import Any

from .models import RunData
from .models import Sample


class BagReadError(RuntimeError):
    """Raised when a rosbag cannot be opened or decoded."""


def _infer_storage(path: Path) -> tuple[str, str, str]:
    if path.is_file():
        suffix = path.suffix.lower()
        if suffix == ".mcap":
            return "mcap", "", ""
        if suffix == ".db3":
            return "sqlite3", "cdr", "cdr"
        if suffix == ".zstd":
            raise BagReadError(
                "file-compressed bags are not supported directly; decompress .mcap.zstd first"
            )
        raise BagReadError(f"unsupported bag file: {path}")

    if not path.is_dir():
        raise BagReadError(f"bag path does not exist: {path}")
    if any(path.glob("*.mcap")):
        return "mcap", "", ""
    if any(path.glob("*.db3")):
        return "sqlite3", "cdr", "cdr"
    raise BagReadError(f"no .mcap or .db3 storage file found in {path}")


def _source_stamp(message: Any) -> float | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    seconds = float(getattr(stamp, "sec", 0))
    nanoseconds = float(getattr(stamp, "nanosec", 0))
    value = seconds + nanoseconds * 1e-9
    return value if value != 0.0 else None


def _yaw(quaternion: Any) -> float:
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _pose_values(pose: Any, covariance: Any = None) -> dict[str, Any]:
    values: dict[str, Any] = {
        "x": float(pose.position.x),
        "y": float(pose.position.y),
        "z": float(pose.position.z),
        "yaw": _yaw(pose.orientation),
    }
    if covariance is not None and len(covariance) >= 36:
        values.update(
            {
                "cov_x": float(covariance[0]),
                "cov_y": float(covariance[7]),
                "cov_yaw": float(covariance[35]),
            }
        )
    return values


def _extract(key: str, message: Any) -> dict[str, Any] | None:
    if key == "ekf_pose":
        values = _pose_values(message.pose.pose, message.pose.covariance)
        values.update(
            {
                "vx": float(message.twist.twist.linear.x),
                "vy": float(message.twist.twist.linear.y),
                "yaw_rate": float(message.twist.twist.angular.z),
            }
        )
        return values
    if key in {"gnss_pose", "ekf_input_pose"}:
        return _pose_values(message.pose.pose, message.pose.covariance)
    if key == "gnss_fix":
        covariance = list(getattr(message, "position_covariance", []))
        return {
            "latitude": float(message.latitude),
            "longitude": float(message.longitude),
            "altitude": float(message.altitude),
            "status": int(message.status.status),
            "cov_x": float(covariance[0]) if len(covariance) > 0 else None,
            "cov_y": float(covariance[4]) if len(covariance) > 4 else None,
            "cov_z": float(covariance[8]) if len(covariance) > 8 else None,
            "covariance_type": int(message.position_covariance_type),
        }
    if key in {"imu_raw", "imu_corrected"}:
        return {
            "yaw_rate": float(message.angular_velocity.z),
            "accel_x": float(message.linear_acceleration.x),
            "accel_y": float(message.linear_acceleration.y),
            "accel_z": float(message.linear_acceleration.z),
        }
    if key == "vehicle_velocity":
        return {
            "vx": float(message.longitudinal_velocity),
            "vy": float(message.lateral_velocity),
            "yaw_rate": float(message.heading_rate),
        }
    if key in {"twist_raw", "twist"}:
        return {
            "vx": float(message.twist.twist.linear.x),
            "vy": float(message.twist.twist.linear.y),
            "yaw_rate": float(message.twist.twist.angular.z),
        }
    if key == "steering":
        return {"steering": float(message.steering_tire_angle)}
    if key == "control":
        return {
            "steering": float(message.lateral.steering_tire_angle),
            "steering_rate": float(message.lateral.steering_tire_rotation_rate),
            "speed": float(message.longitudinal.speed),
            "acceleration": float(message.longitudinal.acceleration),
        }
    if key == "runtime_trajectory":
        points: list[list[float]] = []
        for point in message.points:
            pose = point.pose
            points.append(
                [
                    float(pose.position.x),
                    float(pose.position.y),
                    _yaw(pose.orientation),
                    float(point.longitudinal_velocity_mps),
                ]
            )
        return {"points": points}
    return None


def _trajectory_signature(values: dict[str, Any]) -> tuple[Any, ...]:
    points = values.get("points") or []
    if not points:
        return (0,)
    step = max(1, len(points) // 12)
    sampled = points[::step]
    return (
        len(points),
        *(
            tuple(round(float(value), 3) for value in point[:2])
            for point in sampled
        ),
    )


def read_bag(path: Path, topic_mapping: dict[str, str | None]) -> RunData:
    """Read selected topics and return normalized samples."""

    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions
        from rosbag2_py import SequentialReader
        from rosbag2_py import StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise BagReadError(
            "ROS 2 Python bag dependencies are unavailable; run inside the AI Challenge container"
        ) from error

    storage_id, input_format, output_format = _infer_storage(path)
    reader = SequentialReader()
    try:
        reader.open(
            StorageOptions(uri=str(path), storage_id=storage_id),
            ConverterOptions(
                input_serialization_format=input_format,
                output_serialization_format=output_format,
            ),
        )
    except Exception as error:
        raise BagReadError(f"failed to open rosbag {path}: {error}") from error

    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    reverse_topics = {
        topic: key for key, topic in topic_mapping.items() if isinstance(topic, str)
    }
    selected_types: dict[str, Any] = {}
    warnings: list[str] = []
    for topic, key in reverse_topics.items():
        message_type = topic_types.get(topic)
        if message_type is None:
            warnings.append(f"Missing topic: {topic} ({key})")
            continue
        try:
            selected_types[topic] = get_message(message_type)
        except (AttributeError, ImportError, ModuleNotFoundError, RuntimeError) as error:
            warnings.append(f"Cannot load message type {message_type} for {topic}: {error}")

    absolute_series: dict[str, list[Sample]] = {}
    counts: Counter[str] = Counter()
    first_stamp_ns: int | None = None
    last_stamp_ns: int | None = None
    trajectory_signature: tuple[Any, ...] | None = None

    while reader.has_next():
        topic, serialized, stamp_ns = reader.read_next()
        counts[topic] += 1
        first_stamp_ns = stamp_ns if first_stamp_ns is None else min(first_stamp_ns, stamp_ns)
        last_stamp_ns = stamp_ns if last_stamp_ns is None else max(last_stamp_ns, stamp_ns)
        if topic not in selected_types:
            continue
        key = reverse_topics[topic]
        try:
            message = deserialize_message(serialized, selected_types[topic])
            values = _extract(key, message)
        except Exception as error:
            warnings.append(f"Failed to decode {topic}: {error}")
            selected_types.pop(topic, None)
            continue
        if values is None:
            continue
        if key == "runtime_trajectory":
            signature = _trajectory_signature(values)
            if signature == trajectory_signature:
                continue
            trajectory_signature = signature
            if len(absolute_series.get(key, [])) >= 100:
                continue
        absolute_series.setdefault(key, []).append(
            Sample(
                t=stamp_ns * 1e-9,
                values=values,
                source_stamp=_source_stamp(message),
            )
        )

    origin = (first_stamp_ns or 0) * 1e-9
    normalized_series = {
        key: [
            Sample(
                t=sample.t - origin,
                values=sample.values,
                source_stamp=sample.source_stamp,
            )
            for sample in samples
        ]
        for key, samples in absolute_series.items()
    }
    duration = (
        (last_stamp_ns - first_stamp_ns) * 1e-9
        if first_stamp_ns is not None and last_stamp_ns is not None
        else 0.0
    )
    return RunData(
        bag_path=path,
        series=normalized_series,
        topic_types=topic_types,
        topic_counts=dict(counts),
        warnings=warnings,
        duration_sec=duration,
    )
