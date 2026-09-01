#!/usr/bin/env python3
"""Synchronize speed onto immutable production-normal LiDAR sequences."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from build_recurrent_dataset import (
    DEFAULT_SPEED_MESSAGE_TYPE,
    DEFAULT_SPEED_TOPIC,
    iter_source_sequences,
    load_physical_source_scans,
    longest_true_run,
    read_odometry_speed,
)
from extract_data_from_bag import synchronize_data
from lib.data import ScanControlSequenceDataset
from lib.normal_anchor import (
    NORMAL_ANCHOR_LABEL_SOURCE,
    NORMAL_ANCHOR_SCHEMA_VERSION,
)


def normal_anchor_sequence_id(
    source_sequence_id: str,
    speed_topic: str,
    max_sync_delta_sec: float,
) -> str:
    contract = (
        f"schema={NORMAL_ANCHOR_SCHEMA_VERSION}|source={source_sequence_id}|"
        f"label={NORMAL_ANCHOR_LABEL_SOURCE}|speed={speed_topic}|"
        f"max_delta={max_sync_delta_sec:.9f}|scan_unit=m"
    )
    digest = hashlib.sha256(contract.encode("utf-8")).hexdigest()[:12]
    return f"{source_sequence_id[-72:]}-normal-recurrent-{digest}"


def build_sequence(
    source_dir: Path,
    source_root: Path,
    split: str,
    output_root: Path,
    speed_topic: str,
    speed_message_type: str,
    max_speed_sync_delta_sec: float,
    minimum_contiguous_samples: int,
) -> dict:
    source = ScanControlSequenceDataset(source_dir, expected_split=split)
    source_bag_text = source.metadata.get("source_bag")
    if not isinstance(source_bag_text, str) or not source_bag_text:
        raise ValueError(f"normal source bag missing in {source_dir}")
    source_bag = Path(source_bag_text)
    if not source_bag.is_dir() or not (source_bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"normal source bag unavailable: {source_bag}")
    physical_scans = load_physical_source_scans(
        source_dir, source.scans, source.max_range
    )
    speed_times, raw_speeds = read_odometry_speed(
        source_bag, speed_topic, speed_message_type
    )
    matched_indices, deltas_ns = synchronize_data(
        source.scan_timestamps_ns, speed_times
    )
    max_delta_ns = int(round(max_speed_sync_delta_sec * 1e9))
    accepted = deltas_ns <= max_delta_ns
    start, stop = longest_true_run(accepted)
    if stop - start < minimum_contiguous_samples:
        raise ValueError(
            f"normal synchronized interval too short in {source_dir}: "
            f"samples={stop - start}"
        )
    source_slice = slice(start, stop)
    speed_indices = matched_indices[source_slice]
    deltas_sec = deltas_ns[source_slice].astype(np.float64) / 1e9
    sequence_id = normal_anchor_sequence_id(
        source.sequence_id, speed_topic, max_speed_sync_delta_sec
    )
    output_dir = output_root / split / sequence_id
    if output_dir.exists():
        raise FileExistsError(f"normal-anchor output already exists: {output_dir}")
    output_dir.mkdir(parents=True)
    arrays = {
        "scans.npy": physical_scans[source_slice],
        "speeds.npy": raw_speeds[speed_indices].astype(np.float32, copy=False),
        "scan_timestamps_ns.npy": source.scan_timestamps_ns[source_slice],
        "speed_timestamps_ns.npy": speed_times[speed_indices],
        "speed_sync_deltas_sec.npy": deltas_sec,
    }
    for name, values in arrays.items():
        np.save(output_dir / name, values)
    metadata = {
        "schema_version": NORMAL_ANCHOR_SCHEMA_VERSION,
        "sequence_id": sequence_id,
        "split": split,
        "label_source": NORMAL_ANCHOR_LABEL_SOURCE,
        "target_definition": "frozen_base_steering_correction_equals_zero",
        "stored_source_control_used_as_label": False,
        "scan_unit": "m",
        "scan_shape": list(arrays["scans.npy"].shape[1:]),
        "max_scan_range_m": source.max_range,
        "max_speed_sync_delta_sec": max_speed_sync_delta_sec,
        "source": {
            "dataset_root": str(source_root),
            "sequence_id": source.sequence_id,
            "sequence_dir": str(source_dir),
            "bag": str(source_bag),
            "label_source": source.label_source,
        },
        "topics": {
            "scan": source.metadata["topics"]["scan"],
            "speed": speed_topic,
        },
        "message_types": {
            "scan": source.metadata["message_types"]["scan"],
            "speed": speed_message_type,
        },
        "counts": {
            "source_samples": len(source),
            "accepted_samples": stop - start,
            "rejected_before_interval": start,
            "rejected_after_interval": len(source) - stop,
            "rejected_sync_total": int(np.count_nonzero(~accepted)),
            "raw_speed_samples": len(speed_times),
        },
        "source_interval": {"start_index": start, "stop_index": stop},
        "speed_sync_delta_sec": {
            "mean": float(np.mean(deltas_sec)),
            "p95": float(np.percentile(deltas_sec, 95)),
            "max": float(np.max(deltas_sec)),
        },
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--speed-topic", default=DEFAULT_SPEED_TOPIC)
    parser.add_argument("--speed-message-type", default=DEFAULT_SPEED_MESSAGE_TYPE)
    parser.add_argument("--max-speed-sync-delta-sec", type=float, default=0.05)
    parser.add_argument("--minimum-contiguous-samples", type=int, default=64)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source_root = args.source_root.expanduser().resolve()
    output_root = args.output_root.expanduser().resolve()
    if output_root.exists():
        raise FileExistsError(
            f"normal-anchor root already exists; use an immutable new root: {output_root}"
        )
    if (
        not np.isfinite(args.max_speed_sync_delta_sec)
        or args.max_speed_sync_delta_sec <= 0.0
        or args.minimum_contiguous_samples <= 1
    ):
        raise ValueError("invalid normal-anchor synchronization configuration")
    results = []
    for discovered_root, split, source_dir in iter_source_sequences([source_root]):
        result = build_sequence(
            source_dir,
            discovered_root,
            split,
            output_root,
            args.speed_topic,
            args.speed_message_type,
            args.max_speed_sync_delta_sec,
            args.minimum_contiguous_samples,
        )
        results.append(result)
        print(
            f"built split={split} sequence={result['sequence_id']} "
            f"samples={result['counts']['accepted_samples']}"
        )
    manifest = {
        "schema_version": NORMAL_ANCHOR_SCHEMA_VERSION,
        "label_source": NORMAL_ANCHOR_LABEL_SOURCE,
        "source_dataset_root": str(source_root),
        "speed_topic": args.speed_topic,
        "speed_message_type": args.speed_message_type,
        "max_speed_sync_delta_sec": args.max_speed_sync_delta_sec,
        "minimum_contiguous_samples": args.minimum_contiguous_samples,
        "sequence_ids": [item["sequence_id"] for item in results],
        "summary": {
            "sequences": len(results),
            "train_sequences": sum(item["split"] == "train" for item in results),
            "validation_sequences": sum(
                item["split"] == "val" for item in results
            ),
            "samples": sum(item["counts"]["accepted_samples"] for item in results),
        },
    }
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
