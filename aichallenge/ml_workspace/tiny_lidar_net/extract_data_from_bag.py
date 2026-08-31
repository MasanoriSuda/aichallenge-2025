#!/usr/bin/env python3
"""Extract an auditable, run-level TinyLidarNet dataset from ROS 2 bags."""

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import logging
import multiprocessing
import os
from pathlib import Path
import re
import time
from typing import List, Optional, Tuple

import numpy as np
from rosbags.highlevel import AnyReader


DATASET_SCHEMA_VERSION = 1


@dataclass(frozen=True)
class ExtractionConfig:
    """Configuration shared by all extraction workers."""

    control_topic: str
    scan_topic: str
    max_sync_delta_sec: float
    max_scan_range: float
    label_source: str
    control_msg_type: str = "autoware_auto_control_msgs/msg/AckermannControlCommand"
    scan_msg_type: str = "sensor_msgs/msg/LaserScan"


@dataclass(frozen=True)
class BagTask:
    """One independently auditable source run."""

    bag_path: Path
    sequence_id: str
    split: str


@dataclass
class ExtractionResult:
    """Serializable outcome returned from a worker."""

    sequence_id: str
    split: str
    source_bag: str
    success: bool
    sample_count: int = 0
    rejected_sync_count: int = 0
    error: str = ""


def worker_init(debug_mode: bool) -> None:
    level = logging.DEBUG if debug_mode else logging.INFO
    logging.basicConfig(
        level=level,
        format="[%(levelname)s] [PID:%(process)d] %(message)s",
        force=True,
    )


def setup_logger(debug: bool = False) -> logging.Logger:
    level = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(
        level=level,
        format="[%(levelname)s] [PID:%(process)d] %(message)s",
        handlers=[logging.StreamHandler()],
        force=True,
    )
    return logging.getLogger(__name__)


def clean_scan_array(scan_array: np.ndarray, max_range: float) -> np.ndarray:
    """Replace invalid ranges and clip to the training/runtime range contract."""
    values = np.asarray(scan_array, dtype=np.float32)
    cleaned = np.nan_to_num(values, nan=0.0, posinf=max_range, neginf=0.0)
    return np.clip(cleaned, 0.0, max_range).astype(np.float32, copy=False)


def synchronize_data(
    src_times: np.ndarray, target_times: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Return nearest target index and absolute timestamp delta for each source."""
    src_times = np.asarray(src_times, dtype=np.int64)
    target_times = np.asarray(target_times, dtype=np.int64)
    if target_times.size == 0:
        return np.array([], dtype=np.int64), np.array([], dtype=np.int64)

    insertion = np.searchsorted(target_times, src_times)
    insertion = np.clip(insertion, 0, len(target_times) - 1)
    previous = np.clip(insertion - 1, 0, len(target_times) - 1)
    current_delta = np.abs(target_times[insertion] - src_times)
    previous_delta = np.abs(target_times[previous] - src_times)
    use_previous = previous_delta < current_delta
    return (
        np.where(use_previous, previous, insertion),
        np.where(use_previous, previous_delta, current_delta),
    )


def sync_acceptance_mask(deltas_ns: np.ndarray, max_delta_sec: float) -> np.ndarray:
    """Accept exact-boundary samples and reject only those beyond the contract."""
    if not np.isfinite(max_delta_sec) or max_delta_sec < 0.0:
        raise ValueError("max_delta_sec must be finite and non-negative")
    max_delta_ns = int(round(max_delta_sec * 1e9))
    return np.asarray(deltas_ns, dtype=np.int64) <= max_delta_ns


def make_sequence_id(identity: str) -> str:
    """Create a readable collision-resistant ID without relying on bag basename."""
    normalized = identity.strip().replace("\\", "/").strip("/") or "bag"
    readable = re.sub(r"[^A-Za-z0-9._-]+", "-", normalized).strip("-._")
    readable = (readable or "bag")[-72:]
    digest = hashlib.sha256(normalized.encode("utf-8")).hexdigest()[:10]
    return f"{readable}-{digest}"


def choose_split(sequence_id: str, val_fraction: float, split_seed: int) -> str:
    """Assign an entire sequence to one deterministic dataset split."""
    if not 0.0 <= val_fraction <= 1.0:
        raise ValueError("val_fraction must be within [0.0, 1.0]")
    token = f"{split_seed}:{sequence_id}".encode("utf-8")
    value = int.from_bytes(hashlib.sha256(token).digest()[:8], "big") / float(2**64)
    return "val" if value < val_fraction else "train"


def discover_tasks(
    bags_dir: Optional[Path],
    seq_dirs: Optional[List[Path]],
    val_fraction: float,
    split_seed: int,
) -> List[BagTask]:
    """Discover bags and reject ambiguous sequence identities before workers start."""
    discovered: List[Tuple[Path, str]] = []
    if bags_dir is not None:
        root = bags_dir.expanduser().resolve()
        bag_paths = sorted({path.parent for path in root.rglob("metadata.yaml")})
        if (root / "metadata.yaml").exists():
            bag_paths = [root]
        for path in bag_paths:
            relative = path.relative_to(root)
            identity = str(relative) if str(relative) != "." else root.name
            discovered.append((path, identity))
    else:
        supplied_paths = set()
        for supplied in seq_dirs or []:
            path = supplied.expanduser().resolve()
            if not (path / "metadata.yaml").exists():
                raise ValueError(f"not a ROS 2 bag directory: {path}")
            if path in supplied_paths:
                raise ValueError(f"duplicate ROS 2 bag directory: {path}")
            supplied_paths.add(path)
            identity = str(path)
            discovered.append((path, identity))

    if not discovered:
        raise ValueError("no valid ROS 2 bag directories found")

    tasks: List[BagTask] = []
    identities = {}
    for path, identity in discovered:
        sequence_id = make_sequence_id(identity)
        previous = identities.get(sequence_id)
        if previous is not None and previous != path:
            raise ValueError(
                f"sequence ID collision: {sequence_id}: {previous} and {path}"
            )
        identities[sequence_id] = path
        tasks.append(
            BagTask(
                bag_path=path,
                sequence_id=sequence_id,
                split=choose_split(sequence_id, val_fraction, split_seed),
            )
        )
    return tasks


def _validate_topic_contract(reader: AnyReader, config: ExtractionConfig) -> List:
    expected = {
        config.control_topic: config.control_msg_type,
        config.scan_topic: config.scan_msg_type,
    }
    connections = [connection for connection in reader.connections if connection.topic in expected]
    for topic, message_type in expected.items():
        matching = [connection for connection in connections if connection.topic == topic]
        if not matching:
            raise ValueError(f"required topic missing: {topic}")
        actual_types = sorted({connection.msgtype for connection in matching})
        if actual_types != [message_type]:
            raise ValueError(
                f"topic type mismatch for {topic}: expected={message_type}, "
                f"actual={actual_types}"
            )
    return connections


def process_bag(
    task: BagTask,
    output_root: Path,
    config: ExtractionConfig,
    debug: bool = False,
) -> ExtractionResult:
    """Extract one bag without sharing output identity with any other worker."""
    logger = logging.getLogger(__name__)
    result = ExtractionResult(
        sequence_id=task.sequence_id,
        split=task.split,
        source_bag=str(task.bag_path),
        success=False,
    )
    started = time.perf_counter()

    try:
        output_dir = output_root / task.split / task.sequence_id
        if output_dir.exists():
            raise FileExistsError(
                f"output already exists; use a new output root: {output_dir}"
            )

        command_data: List[List[float]] = []
        command_times: List[int] = []
        scan_data: List[np.ndarray] = []
        scan_times: List[int] = []
        deserialize_failures = 0
        first_deserialize_error = ""
        scan_shape = None

        with AnyReader([task.bag_path]) as reader:
            connections = _validate_topic_contract(reader, config)
            for connection, timestamp, raw in reader.messages(connections=connections):
                try:
                    message = reader.deserialize(raw, connection.msgtype)
                    if connection.topic == config.control_topic:
                        acceleration = float(message.longitudinal.acceleration)
                        steering = float(message.lateral.steering_tire_angle)
                        if not np.isfinite(acceleration) or not np.isfinite(steering):
                            raise ValueError("non-finite control command")
                        command_data.append([steering, acceleration])
                        command_times.append(timestamp)
                    elif connection.topic == config.scan_topic:
                        ranges = np.asarray(message.ranges, dtype=np.float32)
                        if ranges.ndim != 1 or ranges.size == 0:
                            raise ValueError(f"invalid scan shape: {ranges.shape}")
                        if scan_shape is None:
                            scan_shape = ranges.shape
                        elif ranges.shape != scan_shape:
                            raise ValueError(
                                f"scan shape changed: expected={scan_shape}, actual={ranges.shape}"
                            )
                        scan_data.append(clean_scan_array(ranges, config.max_scan_range))
                        scan_times.append(timestamp)
                except Exception as exc:
                    deserialize_failures += 1
                    if not first_deserialize_error:
                        first_deserialize_error = (
                            f"{connection.topic}@{timestamp}: {type(exc).__name__}: {exc}"
                        )

        if not command_data:
            raise ValueError("no valid control commands")
        if not scan_data:
            raise ValueError("no valid LiDAR scans")
        if deserialize_failures:
            raise ValueError(
                f"message extraction failures={deserialize_failures}; "
                f"first={first_deserialize_error}"
            )

        commands = np.asarray(command_data, dtype=np.float32)
        command_timestamps = np.asarray(command_times, dtype=np.int64)
        scans = np.asarray(scan_data, dtype=np.float32)
        scan_timestamps = np.asarray(scan_times, dtype=np.int64)

        command_order = np.argsort(command_timestamps)
        command_timestamps = command_timestamps[command_order]
        commands = commands[command_order]
        scan_order = np.argsort(scan_timestamps)
        scan_timestamps = scan_timestamps[scan_order]
        scans = scans[scan_order]

        matched_indices, deltas_ns = synchronize_data(scan_timestamps, command_timestamps)
        accepted = sync_acceptance_mask(deltas_ns, config.max_sync_delta_sec)
        result.rejected_sync_count = int((~accepted).sum())
        if not np.any(accepted):
            raise ValueError(
                f"all {len(scan_timestamps)} samples exceed sync limit "
                f"{config.max_sync_delta_sec:.6f}s"
            )

        accepted_indices = matched_indices[accepted]
        accepted_deltas_sec = deltas_ns[accepted].astype(np.float64) / 1e9
        accepted_scans = scans[accepted]
        accepted_commands = commands[accepted_indices]
        accepted_scan_times = scan_timestamps[accepted]
        accepted_command_times = command_timestamps[accepted_indices]

        output_dir.mkdir(parents=True, exist_ok=False)
        np.save(output_dir / "scans.npy", accepted_scans)
        np.save(output_dir / "steers.npy", accepted_commands[:, 0])
        np.save(output_dir / "accelerations.npy", accepted_commands[:, 1])
        np.save(output_dir / "delta_times.npy", accepted_deltas_sec)
        np.save(output_dir / "scan_timestamps_ns.npy", accepted_scan_times)
        np.save(output_dir / "control_timestamps_ns.npy", accepted_command_times)

        metadata = {
            "schema_version": DATASET_SCHEMA_VERSION,
            "sequence_id": task.sequence_id,
            "split": task.split,
            "source_bag": str(task.bag_path),
            "topics": {"scan": config.scan_topic, "control": config.control_topic},
            "message_types": {
                "scan": config.scan_msg_type,
                "control": config.control_msg_type,
            },
            "scan_shape": list(accepted_scans.shape[1:]),
            "max_scan_range_m": config.max_scan_range,
            "max_sync_delta_sec": config.max_sync_delta_sec,
            "label_source": config.label_source,
            "counts": {
                "raw_scans": len(scan_timestamps),
                "raw_controls": len(command_timestamps),
                "accepted_samples": len(accepted_scans),
                "rejected_sync_samples": result.rejected_sync_count,
                "message_failures": deserialize_failures,
            },
            "sync_delta_sec": {
                "mean": float(np.mean(accepted_deltas_sec)),
                "p95": float(np.percentile(accepted_deltas_sec, 95)),
                "max": float(np.max(accepted_deltas_sec)),
            },
            "timestamp_ns": {
                "first_scan": int(accepted_scan_times[0]),
                "last_scan": int(accepted_scan_times[-1]),
            },
        }
        (output_dir / "metadata.json").write_text(
            json.dumps(metadata, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )

        result.success = True
        result.sample_count = len(accepted_scans)
        logger.info(
            "Saved %s split=%s samples=%d rejected_sync=%d duration=%.2fs",
            task.sequence_id,
            task.split,
            result.sample_count,
            result.rejected_sync_count,
            time.perf_counter() - started,
        )
        if debug:
            logger.debug("metadata=%s", metadata)
    except Exception as exc:
        result.error = f"{type(exc).__name__}: {exc}"
        logger.error("Failed %s: %s", task.sequence_id, result.error)

    return result


def write_manifest(
    output_root: Path,
    config: ExtractionConfig,
    val_fraction: float,
    split_seed: int,
    results: List[ExtractionResult],
) -> None:
    output_root.mkdir(parents=True, exist_ok=True)
    manifest = {
        "schema_version": DATASET_SCHEMA_VERSION,
        "config": asdict(config),
        "val_fraction": val_fraction,
        "split_seed": split_seed,
        "summary": {
            "total_sequences": len(results),
            "successful_sequences": sum(result.success for result in results),
            "failed_sequences": sum(not result.success for result in results),
            "accepted_samples": sum(result.sample_count for result in results),
            "rejected_sync_samples": sum(result.rejected_sync_count for result in results),
        },
        "sequences": [asdict(result) for result in results],
    }
    (output_root / "dataset-manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Extract synchronized, run-level TinyLidarNet training data.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bags-dir", type=Path, help="Recursively discover ROS 2 bags.")
    source.add_argument("--seq-dirs", type=Path, nargs="+", help="Explicit bag directories.")
    parser.add_argument("--outdir", type=Path, required=True)
    parser.add_argument("--control-topic", default="/control/command/control_cmd")
    parser.add_argument("--scan-topic", default="/sensing/lidar/scan")
    parser.add_argument("--max-sync-delta-sec", type=float, default=0.05)
    parser.add_argument("--max-scan-range", type=float, default=30.0)
    parser.add_argument(
        "--label-source",
        required=True,
        choices=(
            "mpc",
            "mpcc",
            "human",
            "lidar_gap_teacher",
            "student",
            "other",
        ),
        help="Provenance of the synchronized control labels.",
    )
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--split-seed", type=int, default=2026)
    parser.add_argument("--workers", type=int, default=min(os.cpu_count() or 1, 8))
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    logger = setup_logger(args.debug)
    if not np.isfinite(args.max_sync_delta_sec) or args.max_sync_delta_sec < 0.0:
        parser.error("--max-sync-delta-sec must be finite and non-negative")
    if not np.isfinite(args.max_scan_range) or args.max_scan_range <= 0.0:
        parser.error("--max-scan-range must be finite and positive")
    if not 0.0 <= args.val_fraction <= 1.0:
        parser.error("--val-fraction must be within [0.0, 1.0]")

    try:
        tasks = discover_tasks(
            args.bags_dir,
            args.seq_dirs,
            args.val_fraction,
            args.split_seed,
        )
    except ValueError as exc:
        parser.error(str(exc))

    output_root = args.outdir.expanduser().resolve()
    config = ExtractionConfig(
        control_topic=args.control_topic,
        scan_topic=args.scan_topic,
        max_sync_delta_sec=args.max_sync_delta_sec,
        max_scan_range=args.max_scan_range,
        label_source=args.label_source,
    )
    worker_count = min(max(1, args.workers), len(tasks))
    logger.info("Found %d bags; workers=%d", len(tasks), worker_count)

    worker_args = [(task, output_root, config, args.debug) for task in tasks]
    if worker_count == 1:
        results = [process_bag(*worker_args[0])]
    else:
        with multiprocessing.Pool(
            processes=worker_count,
            initializer=worker_init,
            initargs=(args.debug,),
        ) as pool:
            results = pool.starmap(process_bag, worker_args)

    write_manifest(output_root, config, args.val_fraction, args.split_seed, results)
    failures = [result for result in results if not result.success]
    if failures:
        logger.error("Extraction failed for %d/%d sequences", len(failures), len(results))
        raise SystemExit(2)
    logger.info(
        "Extraction complete: sequences=%d samples=%d",
        len(results),
        sum(result.sample_count for result in results),
    )


if __name__ == "__main__":
    multiprocessing.set_start_method("spawn", force=True)
    main()
