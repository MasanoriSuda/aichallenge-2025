#!/usr/bin/env python3
"""Relabel pre-contact student LiDAR states with the runtime gap teacher."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from typing import Optional

import numpy as np

from extract_data_from_bag import clean_scan_array, make_sequence_id
from lib.checkpoint import sha256_file


def first_confirmed_breach(
    minima_m: np.ndarray,
    threshold_m: float,
    confirmation_samples: int,
) -> Optional[int]:
    """Return the first index starting a confirmed low-clearance interval."""
    minima = np.asarray(minima_m, dtype=np.float64)
    if minima.ndim != 1 or not np.all(np.isfinite(minima)):
        raise ValueError("minima must be a finite one-dimensional array")
    if not np.isfinite(threshold_m) or threshold_m <= 0.0:
        raise ValueError("threshold_m must be finite and positive")
    if confirmation_samples <= 0:
        raise ValueError("confirmation_samples must be positive")
    breached = minima < threshold_m
    if breached.size < confirmation_samples:
        return None
    window = np.convolve(
        breached.astype(np.int32),
        np.ones(confirmation_samples, dtype=np.int32),
        mode="valid",
    )
    confirmed = np.flatnonzero(window == confirmation_samples)
    return None if confirmed.size == 0 else int(confirmed[0])


def cutoff_before_margin(
    timestamps_ns: np.ndarray,
    breach_index: Optional[int],
    margin_sec: float,
) -> int:
    """Return an exclusive cutoff before a breach, preserving monotonic time."""
    timestamps = np.asarray(timestamps_ns, dtype=np.int64)
    if timestamps.ndim != 1 or timestamps.size == 0:
        raise ValueError("timestamps must be a non-empty one-dimensional array")
    if np.any(np.diff(timestamps) < 0):
        raise ValueError("timestamps must be monotonic")
    if not np.isfinite(margin_sec) or margin_sec < 0.0:
        raise ValueError("margin_sec must be finite and non-negative")
    if breach_index is None:
        return int(timestamps.size)
    if not 0 <= breach_index < timestamps.size:
        raise ValueError("breach_index is outside timestamps")
    cutoff_time = int(timestamps[breach_index] - round(margin_sec * 1e9))
    return int(np.searchsorted(timestamps, cutoff_time, side="left"))


def minimum_observed_ranges(scans: np.ndarray) -> np.ndarray:
    """Return per-scan positive minima, treating an empty scan as blocked."""
    scan_array = np.asarray(scans, dtype=np.float32)
    if scan_array.ndim != 2:
        raise ValueError("scans must be a two-dimensional array")
    positive_ranges = np.where(scan_array > 0.0, scan_array, np.inf)
    minima = np.min(positive_ranges, axis=1)
    return np.where(np.isfinite(minima), minima, 0.0)


def read_scans(bag_path: Path, topic: str, max_range_m: float) -> tuple:
    try:
        from rosbags.highlevel import AnyReader
    except ImportError as exc:
        raise RuntimeError("rosbags is required inside the development container") from exc

    timestamps = []
    scans = []
    with AnyReader([bag_path]) as reader:
        matching = [connection for connection in reader.connections if connection.topic == topic]
        if not matching:
            raise ValueError(f"required topic missing: {topic}")
        actual_types = sorted({connection.msgtype for connection in matching})
        if actual_types != ["sensor_msgs/msg/LaserScan"]:
            raise ValueError(f"unexpected scan topic types: {actual_types}")
        for connection, timestamp, raw in reader.messages(connections=matching):
            message = reader.deserialize(raw, connection.msgtype)
            cleaned = clean_scan_array(np.asarray(message.ranges), max_range_m)
            if cleaned.shape != (750,):
                raise ValueError(f"expected 750-point LiDAR scan, got {cleaned.shape}")
            timestamps.append(timestamp)
            scans.append(cleaned)
    if not scans:
        raise ValueError("bag contains no valid LiDAR scans")
    return np.asarray(timestamps, dtype=np.int64), np.asarray(scans, dtype=np.float32)


def relabel(args: argparse.Namespace) -> dict:
    from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
    from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore

    bag = args.bag.expanduser().resolve()
    checkpoint = args.checkpoint.expanduser().resolve()
    if not (bag / "metadata.yaml").is_file():
        raise ValueError(f"not a ROS 2 bag directory: {bag}")
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")

    timestamps, scans = read_scans(bag, args.scan_topic, args.max_scan_range)
    minima = minimum_observed_ranges(scans)
    breach_index = first_confirmed_breach(
        minima, args.contact_clearance_m, args.contact_confirmation_samples
    )
    cutoff = cutoff_before_margin(timestamps, breach_index, args.pre_contact_margin_sec)
    if cutoff <= 0:
        raise ValueError("contact cutoff removed the complete sequence")

    teacher_config = GapTeacherConfig()
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(checkpoint),
        acceleration=args.fixed_acceleration,
        control_mode="gap_teacher",
        max_range=args.max_scan_range,
        gap_teacher_config=teacher_config,
    )
    accepted_scans = []
    accepted_steering = []
    accepted_acceleration = []
    accepted_timestamps = []
    reason_counts = {}
    for timestamp, scan in zip(timestamps[:cutoff], scans[:cutoff]):
        acceleration, steering = core.process(scan)
        decision = core.last_gap_teacher_decision
        if decision is None:
            raise RuntimeError("gap teacher did not produce an auditable decision")
        reason_counts[decision.reason] = reason_counts.get(decision.reason, 0) + 1
        if args.active_only and not decision.active:
            continue
        accepted_scans.append(scan)
        accepted_steering.append(steering)
        accepted_acceleration.append(acceleration)
        accepted_timestamps.append(timestamp)
    if not accepted_scans:
        raise ValueError("no teacher correction samples survived admission")

    checkpoint_sha = sha256_file(checkpoint)
    identity = f"{bag}:dagger:{checkpoint_sha}:{timestamps[cutoff - 1]}"
    sequence_id = make_sequence_id(identity)
    output_dir = args.outdir.expanduser().resolve() / "train" / sequence_id
    if output_dir.exists():
        raise FileExistsError(f"output already exists: {output_dir}")
    output_dir.mkdir(parents=True)

    scan_array = np.asarray(accepted_scans, dtype=np.float32)
    steering_array = np.asarray(accepted_steering, dtype=np.float32)
    acceleration_array = np.asarray(accepted_acceleration, dtype=np.float32)
    timestamp_array = np.asarray(accepted_timestamps, dtype=np.int64)
    np.save(output_dir / "scans.npy", scan_array)
    np.save(output_dir / "steers.npy", steering_array)
    np.save(output_dir / "accelerations.npy", acceleration_array)
    np.save(output_dir / "delta_times.npy", np.zeros(len(scan_array), dtype=np.float64))
    np.save(output_dir / "scan_timestamps_ns.npy", timestamp_array)
    np.save(output_dir / "control_timestamps_ns.npy", timestamp_array)

    metadata = {
        "schema_version": 1,
        "sequence_id": sequence_id,
        "split": "train",
        "source_bag": str(bag),
        "topics": {"scan": args.scan_topic, "control": "offline_gap_teacher"},
        "message_types": {
            "scan": "sensor_msgs/msg/LaserScan",
            "control": "generated/tiny_lidar_gap_teacher",
        },
        "scan_shape": [750],
        "max_scan_range_m": args.max_scan_range,
        "max_sync_delta_sec": 0.0,
        "label_source": "lidar_gap_teacher_dagger",
        "counts": {
            "raw_scans": int(len(scans)),
            "pre_contact_scans": int(cutoff),
            "accepted_samples": int(len(scan_array)),
            "rejected_sync_samples": 0,
            "message_failures": 0,
        },
        "sync_delta_sec": {"mean": 0.0, "p95": 0.0, "max": 0.0},
        "timestamp_ns": {
            "first_scan": int(timestamp_array[0]),
            "last_scan": int(timestamp_array[-1]),
        },
        "relabeling": {
            "student_checkpoint": str(checkpoint),
            "student_checkpoint_sha256": checkpoint_sha,
            "teacher": "LidarGapTeacher",
            "teacher_config": asdict(teacher_config),
            "active_only": args.active_only,
            "decision_reason_counts_before_filter": reason_counts,
            "contact_clearance_m": args.contact_clearance_m,
            "contact_confirmation_samples": args.contact_confirmation_samples,
            "pre_contact_margin_sec": args.pre_contact_margin_sec,
            "breach_index": breach_index,
            "exclusive_cutoff_index": cutoff,
        },
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--scan-topic", default="/sensing/lidar/scan")
    parser.add_argument("--max-scan-range", type=float, default=30.0)
    parser.add_argument("--fixed-acceleration", type=float, default=0.6)
    parser.add_argument("--contact-clearance-m", type=float, default=0.5)
    parser.add_argument("--contact-confirmation-samples", type=int, default=3)
    parser.add_argument("--pre-contact-margin-sec", type=float, default=1.0)
    parser.add_argument(
        "--active-only", action=argparse.BooleanOptionalAction, default=True
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    metadata = relabel(args)
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
