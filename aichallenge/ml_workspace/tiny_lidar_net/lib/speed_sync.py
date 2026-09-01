"""Auditable wheel-speed extraction and causal timestamp synchronization."""

from __future__ import annotations

from pathlib import Path
from typing import Tuple

import numpy as np


DEFAULT_SPEED_TOPIC = "/vehicle/status/velocity_status"
DEFAULT_SPEED_MESSAGE_TYPE = "autoware_auto_vehicle_msgs/msg/VelocityReport"
SUPPORTED_SPEED_MESSAGE_TYPES = frozenset(
    {
        DEFAULT_SPEED_MESSAGE_TYPE,
        "nav_msgs/msg/Odometry",
    }
)
CAUSAL_SPEED_SYNC_POLICY = "latest_preceding"


def read_longitudinal_speed(
    bag_path: Path,
    speed_topic: str,
    expected_message_type: str,
) -> Tuple[np.ndarray, np.ndarray]:
    """Read finite non-negative longitudinal speed with bag timestamps."""
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError(
            "rosbags is required inside the development container"
        ) from exc

    if expected_message_type not in SUPPORTED_SPEED_MESSAGE_TYPES:
        raise ValueError(
            "unsupported speed message type: "
            f"{expected_message_type}; "
            f"supported={sorted(SUPPORTED_SPEED_MESSAGE_TYPES)}"
        )
    timestamps = []
    speeds = []
    failures = []
    with AnyReader([bag_path]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic == speed_topic
        ]
        if not connections:
            raise ValueError(f"required speed topic missing: {speed_topic}")
        message_types = sorted({connection.msgtype for connection in connections})
        if message_types != [expected_message_type]:
            raise ValueError(
                f"speed topic type mismatch: expected={expected_message_type}, "
                f"actual={message_types}"
            )
        for connection, timestamp, raw in reader.messages(connections=connections):
            try:
                message = reader.deserialize(raw, connection.msgtype)
                if expected_message_type == "nav_msgs/msg/Odometry":
                    speed = abs(float(message.twist.twist.linear.x))
                else:
                    speed = abs(float(message.longitudinal_velocity))
                if not np.isfinite(speed):
                    raise ValueError("non-finite longitudinal speed")
                timestamps.append(timestamp)
                speeds.append(speed)
            except Exception as exc:  # Partial sensor evidence is not auditable.
                failures.append(f"{timestamp}:{type(exc).__name__}:{exc}")
    if failures:
        raise ValueError(
            f"speed extraction failures={len(failures)}; first={failures[0]}"
        )
    if not timestamps:
        raise ValueError(f"no valid speed samples on {speed_topic}")
    timestamp_array = np.asarray(timestamps, dtype=np.int64)
    speed_array = np.asarray(speeds, dtype=np.float32)
    order = np.argsort(timestamp_array, kind="stable")
    timestamp_array = timestamp_array[order]
    speed_array = speed_array[order]
    if np.any(np.diff(timestamp_array) <= 0):
        raise ValueError("speed timestamps must be strictly increasing")
    return timestamp_array, speed_array


def synchronize_latest_preceding(
    query_timestamps_ns: np.ndarray,
    sample_timestamps_ns: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Match each query to the newest sample that is not from the future.

    Returns matched indices, non-negative ages in nanoseconds and a validity
    mask.  Invalid queries precede the first available sample and receive index
    ``-1`` plus the maximum int64 age so callers cannot accidentally admit
    them with a permissive threshold.
    """
    queries = np.asarray(query_timestamps_ns, dtype=np.int64)
    samples = np.asarray(sample_timestamps_ns, dtype=np.int64)
    if queries.ndim != 1 or samples.ndim != 1:
        raise ValueError("synchronization timestamps must be one-dimensional")
    if queries.size == 0 or samples.size == 0:
        raise ValueError("synchronization timestamps must not be empty")
    if np.any(np.diff(queries) < 0):
        raise ValueError("query timestamps must be monotonic")
    if np.any(np.diff(samples) <= 0):
        raise ValueError("sample timestamps must be strictly increasing")

    indices = np.searchsorted(samples, queries, side="right") - 1
    valid = indices >= 0
    safe_indices = np.maximum(indices, 0)
    ages = np.full(queries.shape, np.iinfo(np.int64).max, dtype=np.int64)
    ages[valid] = queries[valid] - samples[safe_indices[valid]]
    if np.any(ages[valid] < 0):
        raise RuntimeError("causal synchronization produced a future sample")
    return indices.astype(np.int64, copy=False), ages, valid
