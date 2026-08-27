#!/usr/bin/env python3
"""Summarize V2X source-sample continuity for Slice 6 evidence."""

from __future__ import annotations

import argparse
import math
from collections import Counter

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


def stamp_seconds(stamp: object) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topic", default="/v2x/vehicle_positions")
    parser.add_argument("--jump-min", type=float, default=5.0)
    parser.add_argument("--speed-limit", type=float, default=30.0)
    parser.add_argument("--sample-from", type=float)
    parser.add_argument("--sample-to", type=float)
    args = parser.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_type = get_message(topic_types[args.topic])

    previous_array_stamp: float | None = None
    previous_samples: dict[str, tuple[float, float, float]] = {}
    counts: Counter[str] = Counter()
    examples: dict[str, list[str]] = {}

    def record(kind: str, detail: str) -> None:
        counts[kind] += 1
        if len(examples.setdefault(kind, [])) < 8:
            examples[kind].append(detail)

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic != args.topic:
            continue
        message = deserialize_message(serialized, message_type)
        counts["messages"] += 1
        array_stamp = stamp_seconds(message.header.stamp)
        if previous_array_stamp is not None:
            array_dt = array_stamp - previous_array_stamp
            if array_dt <= 1e-9:
                record("nonadvancing_array_stamp", f"stamp={array_stamp:.9f} dt={array_dt:.9f}")
            elif array_dt > 0.1:
                record("array_gap_over_100ms", f"stamp={array_stamp:.9f} dt={array_dt:.9f}")
        previous_array_stamp = array_stamp
        if not message.vehicles:
            counts["empty_messages"] += 1
        seen: set[str] = set()
        for vehicle in message.vehicles:
            counts["vehicle_samples"] += 1
            vehicle_id = vehicle.vehicle_id
            if vehicle_id in seen:
                record("duplicate_id", f"stamp={array_stamp:.9f} id={vehicle_id}")
            seen.add(vehicle_id)
            sample_stamp = stamp_seconds(vehicle.header.stamp)
            source_age = array_stamp - sample_stamp
            if source_age < -1e-6 or source_age > 1.0 + 1e-6:
                record(
                    "sample_source_age_invalid",
                    f"array={array_stamp:.9f} sample={sample_stamp:.9f} age={source_age:.6f}",
                )
            x = float(vehicle.position.x)
            y = float(vehicle.position.y)
            if (
                args.sample_from is not None
                and args.sample_to is not None
                and args.sample_from <= array_stamp <= args.sample_to
            ):
                print(
                    "sample "
                    f"array={array_stamp:.6f} id={vehicle_id} "
                    f"source={sample_stamp:.6f} "
                    f"position=({x:.3f},{y:.3f})"
                )
            if not all(math.isfinite(value) for value in (array_stamp, sample_stamp, x, y)):
                record("nonfinite", f"stamp={array_stamp} id={vehicle_id} x={x} y={y}")
                continue
            previous = previous_samples.get(vehicle_id)
            if previous is not None:
                dt = sample_stamp - previous[0]
                jump = math.hypot(x - previous[1], y - previous[2])
                if dt <= 1e-9:
                    record(
                        "nonadvancing_vehicle_stamp",
                        f"id={vehicle_id} stamp={sample_stamp:.9f} dt={dt:.9f} jump={jump:.6f}",
                    )
                else:
                    speed = jump / dt
                    admissible_jump = max(args.jump_min, args.speed_limit * dt)
                    if jump > admissible_jump:
                        record(
                            "position_jump",
                            f"id={vehicle_id} stamp={sample_stamp:.9f} dt={dt:.6f} jump={jump:.3f} limit={admissible_jump:.3f}",
                        )
                    if speed > args.speed_limit:
                        record(
                            "speed_over_limit",
                            f"id={vehicle_id} stamp={sample_stamp:.9f} dt={dt:.6f} speed={speed:.3f}",
                        )
            previous_samples[vehicle_id] = (sample_stamp, x, y)

    for key in sorted(counts):
        print(f"{key}={counts[key]}")
        for detail in examples.get(key, []):
            print(f"  {detail}")


if __name__ == "__main__":
    main()
