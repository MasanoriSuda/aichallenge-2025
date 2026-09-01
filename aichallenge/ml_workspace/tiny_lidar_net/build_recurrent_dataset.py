#!/usr/bin/env python3
"""Derive a speed-synchronized recurrent policy dataset from admitted runs."""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Iterable, Sequence, Tuple

import numpy as np

from extract_data_from_bag import synchronize_data
from lib.recurrent_policy import RECURRENT_DATASET_SCHEMA_VERSION
from lib.residual import SteeringResidualSequenceDataset
from lib.speed_sync import (
    DEFAULT_SPEED_MESSAGE_TYPE,
    DEFAULT_SPEED_TOPIC,
    read_longitudinal_speed,
)
from lib.supervision import validate_executed_teacher_certificate


# End-to-End AI may use wheel odometry, but not the GNSS/IMU-fused localization
# state.  Keep legacy Odometry support only as an explicit audit input; newly
# built datasets default to the AWSIM wheel-speed contract used in production.
RECURRENT_LABEL_SOURCES = {
    "lidar_precontact_teacher_dagger": (
        "lidar_precontact_teacher_recurrent_direct"
    ),
    "lidar_speed_committed_teacher_dagger": (
        "lidar_speed_committed_teacher_recurrent_direct"
    ),
}
EXECUTED_TEACHER_MODES_BY_LABEL_SOURCE = {
    "lidar_precontact_teacher_dagger": "precontact_teacher",
    "lidar_speed_committed_teacher_dagger": "speed_committed_teacher",
}


def longest_true_run(mask: np.ndarray) -> Tuple[int, int]:
    """Return the half-open bounds of the longest contiguous accepted run."""
    accepted = np.asarray(mask, dtype=bool)
    if accepted.ndim != 1:
        raise ValueError("acceptance mask must be one-dimensional")
    best_start = best_stop = run_start = 0
    in_run = False
    for index, value in enumerate(accepted):
        if value and not in_run:
            run_start = index
            in_run = True
        if in_run and (not value or index == len(accepted) - 1):
            run_stop = index if not value else index + 1
            if run_stop - run_start > best_stop - best_start:
                best_start, best_stop = run_start, run_stop
            in_run = False
    return best_start, best_stop


def recurrent_sequence_id(
    source_sequence_id: str,
    speed_topic: str,
    max_sync_delta_sec: float,
    outcome_certificate_sha256: str | None = None,
) -> str:
    """Bind derived identity to the source run and speed synchronization contract."""
    contract = (
        f"schema={RECURRENT_DATASET_SCHEMA_VERSION}|source={source_sequence_id}|"
        f"speed={speed_topic}|max_delta={max_sync_delta_sec:.9f}|scan_unit=m"
        f"|outcome={outcome_certificate_sha256 or 'unproven'}"
    )
    digest = hashlib.sha256(contract.encode("utf-8")).hexdigest()[:12]
    readable = source_sequence_id[-72:]
    return f"{readable}-recurrent-{digest}"


def load_physical_source_scans(
    source_dir: Path,
    normalized_scans: np.ndarray,
    max_scan_range_m: float,
) -> np.ndarray:
    """Recover the immutable metre array and prove its normalized identity."""
    raw_path = source_dir / "scans.npy"
    raw = np.load(raw_path, allow_pickle=False)
    if raw.shape != normalized_scans.shape or raw.ndim != 2:
        raise ValueError(f"source physical scan shape mismatch: {raw_path}")
    if not np.all(np.isfinite(raw)) or np.any(raw < 0.0) or np.any(
        raw > max_scan_range_m
    ):
        raise ValueError(f"invalid source physical scans: {raw_path}")
    expected_normalized = raw / max_scan_range_m
    if not np.allclose(
        expected_normalized,
        normalized_scans,
        rtol=1e-6,
        atol=1e-7,
    ):
        raise ValueError(
            f"source physical/normalized scan identity mismatch: {raw_path}"
        )
    return raw.astype(np.float32, copy=False)


def recurrent_label_source(raw_label_source: str) -> str:
    """Map one immutable raw teacher identity to its recurrent identity."""
    try:
        return RECURRENT_LABEL_SOURCES[raw_label_source]
    except KeyError:
        raise ValueError(
            f"unsupported recurrent source label: {raw_label_source}"
        ) from None


def executed_teacher_mode(raw_label_source: str) -> str:
    """Return the certificate mode required by one raw label source."""
    try:
        return EXECUTED_TEACHER_MODES_BY_LABEL_SOURCE[raw_label_source]
    except KeyError:
        raise ValueError(
            f"unsupported executed teacher source: {raw_label_source}"
        ) from None


def load_embedded_causal_speed(
    source: SteeringResidualSequenceDataset,
    max_speed_sync_delta_sec: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Load and prove the speed sequence used by the stateful raw teacher.

    A stateful teacher must be replayed on every scan in order. Re-reading its
    bag with a different synchronizer would silently create different labels,
    so the recurrent derivative inherits the exact causal speed arrays stored
    by the raw relabeler.
    """
    relabeling = source.metadata.get("relabeling", {})
    if relabeling.get("control_mode") != "speed_committed_teacher":
        raise ValueError("embedded causal speed requires speed-committed source")
    if (
        relabeling.get("active_only") is not False
        or relabeling.get("novel_policy_only") is not False
    ):
        raise ValueError(
            "stateful speed-committed source must retain every temporal sample"
        )
    speed_sync = source.metadata.get("speed_sync")
    if not isinstance(speed_sync, dict) or speed_sync.get("policy") != (
        "latest_preceding"
    ):
        raise ValueError("speed-committed source lacks causal speed provenance")
    recorded_limit = speed_sync.get("max_delta_sec")
    if not isinstance(recorded_limit, (int, float)) or (
        recorded_limit > max_speed_sync_delta_sec + 1e-12
    ):
        raise ValueError("speed-committed source speed contract is too loose")

    required = (
        "speeds.npy",
        "speed_timestamps_ns.npy",
        "speed_sync_deltas_sec.npy",
    )
    missing = [name for name in required if not (source.seq_dir / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"missing embedded causal speed arrays in {source.seq_dir}: {missing}"
        )
    speeds = np.load(source.seq_dir / "speeds.npy", allow_pickle=False)
    timestamps = np.load(
        source.seq_dir / "speed_timestamps_ns.npy", allow_pickle=False
    )
    deltas = np.load(
        source.seq_dir / "speed_sync_deltas_sec.npy", allow_pickle=False
    )
    arrays = (speeds, timestamps, deltas)
    if any(values.ndim != 1 or len(values) != len(source) for values in arrays):
        raise ValueError("embedded causal speed array length mismatch")
    if not np.all(np.isfinite(speeds)) or np.any(speeds < 0.0):
        raise ValueError("embedded causal speeds must be finite and non-negative")
    if not np.issubdtype(timestamps.dtype, np.integer):
        raise ValueError("embedded speed timestamps must be integer nanoseconds")
    if np.any(timestamps > source.scan_timestamps_ns):
        raise ValueError("embedded causal speed contains a future sample")
    expected_deltas = (
        source.scan_timestamps_ns - timestamps
    ).astype(np.float64, copy=False) / 1e9
    if not np.allclose(deltas, expected_deltas, rtol=0.0, atol=1e-12):
        raise ValueError("embedded causal speed age identity mismatch")
    if np.any(deltas < 0.0) or np.any(
        deltas > max_speed_sync_delta_sec + 1e-12
    ):
        raise ValueError("embedded causal speed exceeds freshness contract")
    return (
        speeds.astype(np.float32, copy=False),
        timestamps.astype(np.int64, copy=False),
        deltas.astype(np.float64, copy=False),
    )


def iter_source_sequences(
    source_roots: Sequence[Path],
    excluded_sequence_ids: Sequence[str] = (),
    allow_partial_additional_roots: bool = False,
) -> Iterable[Tuple[Path, str, Path]]:
    """Discover immutable sources and reject run identity reuse up front.

    In partial-root mode, one immutable run root may contain only its assigned
    split, but the complete aggregate must still contain both train and val.
    """
    if not source_roots:
        raise ValueError("at least one source dataset root is required")
    excluded = set(excluded_sequence_ids)
    if len(excluded) != len(excluded_sequence_ids):
        raise ValueError("duplicate excluded source sequence identity")
    matched_exclusions = {sequence_id: 0 for sequence_id in excluded}
    seen_roots = set()
    seen_sequences = {}
    discovered = []
    for source_root in source_roots:
        resolved_root = source_root.expanduser().resolve()
        if resolved_root in seen_roots:
            raise ValueError(f"duplicate source dataset root: {resolved_root}")
        seen_roots.add(resolved_root)
        root_sequence_count = 0
        for split in ("train", "val"):
            split_root = resolved_root / split
            if not split_root.is_dir():
                if allow_partial_additional_roots:
                    continue
                raise FileNotFoundError(f"missing source split: {split_root}")
            sequence_dirs = sorted(
                path for path in split_root.iterdir() if path.is_dir()
            )
            if not sequence_dirs:
                if allow_partial_additional_roots:
                    continue
                raise ValueError(f"source split has no sequences: {split_root}")
            for sequence_dir in sequence_dirs:
                root_sequence_count += 1
                try:
                    metadata = json.loads(
                        (sequence_dir / "metadata.json").read_text(encoding="utf-8")
                    )
                except (OSError, json.JSONDecodeError) as exc:
                    raise ValueError(
                        f"failed to read source metadata {sequence_dir}: {exc}"
                    ) from exc
                sequence_id = metadata.get("sequence_id")
                if not isinstance(sequence_id, str) or not sequence_id:
                    raise ValueError(f"missing source sequence identity: {sequence_dir}")
                if sequence_id in excluded:
                    matched_exclusions[sequence_id] += 1
                    if matched_exclusions[sequence_id] > 1:
                        raise ValueError(
                            "excluded source sequence identity is ambiguous: "
                            f"{sequence_id}"
                        )
                    continue
                previous = seen_sequences.get(sequence_id)
                if previous is not None:
                    raise ValueError(
                        f"duplicate source sequence identity {sequence_id}: "
                        f"{previous} and {sequence_dir}"
                    )
                seen_sequences[sequence_id] = sequence_dir
                discovered.append((resolved_root, split, sequence_dir))
        if root_sequence_count == 0:
            raise ValueError(f"source root has no sequences: {resolved_root}")
    if allow_partial_additional_roots:
        discovered_splits = {split for _, split, _ in discovered}
        missing_splits = sorted({"train", "val"} - discovered_splits)
        if missing_splits:
            raise FileNotFoundError(
                "aggregate source dataset is missing splits: "
                f"{missing_splits}"
            )
    missing_exclusions = {
        sequence_id
        for sequence_id, count in matched_exclusions.items()
        if count == 0
    }
    if missing_exclusions:
        raise ValueError(
            "excluded source sequence identity was not found: "
            f"{sorted(missing_exclusions)}"
        )
    return discovered


def build_sequence(
    source_dir: Path,
    output_root: Path,
    split: str,
    source_dataset_root: Path,
    speed_topic: str,
    speed_message_type: str,
    max_speed_sync_delta_sec: float,
    minimum_contiguous_samples: int,
    require_executed_success: bool = False,
) -> dict:
    """Build one sequence, preserving source labels and temporal continuity."""
    source = SteeringResidualSequenceDataset(
        source_dir,
        expected_split=split,
        input_mode="stateless",
    )
    raw_label_source = source.metadata["label_source"]
    derived_label_source = recurrent_label_source(raw_label_source)
    expected_control_mode = executed_teacher_mode(raw_label_source)
    recorded_control_mode = source.metadata.get("relabeling", {}).get(
        "control_mode"
    )
    if recorded_control_mode != expected_control_mode:
        raise ValueError(
            "source teacher mode/label mismatch: "
            f"label={raw_label_source}, mode={recorded_control_mode}"
        )
    source_bag_text = source.metadata.get("source_bag")
    if not isinstance(source_bag_text, str) or not source_bag_text:
        raise ValueError(f"source_bag missing in {source_dir}")
    source_bag = Path(source_bag_text)
    if not source_bag.is_dir() or not (source_bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"source bag unavailable: {source_bag}")
    max_scan_range_m = float(source.metadata["max_scan_range_m"])
    outcome_certificate = source.metadata.get("outcome_certificate")
    recorded_certificate_sha = source.metadata.get(
        "outcome_certificate_sha256"
    )
    outcome_certificate_sha = None
    if outcome_certificate is not None:
        outcome_certificate_sha = validate_executed_teacher_certificate(
            outcome_certificate,
            source_bag=source_bag,
            checkpoint_sha256=source.metadata["relabeling"][
                "student_checkpoint_sha256"
            ],
            expected_control_mode=expected_control_mode,
        )
        if recorded_certificate_sha != outcome_certificate_sha:
            raise ValueError("source outcome certificate digest mismatch")
    elif require_executed_success:
        raise ValueError(
            f"source lacks executed teacher success certificate: {source_dir}"
        )
    physical_scans = load_physical_source_scans(
        source_dir, source.scans, max_scan_range_m
    )

    if expected_control_mode == "speed_committed_teacher":
        if source.metadata.get("topics", {}).get("speed") != speed_topic:
            raise ValueError("speed-committed source speed topic mismatch")
        if source.metadata.get("message_types", {}).get("speed") != (
            speed_message_type
        ):
            raise ValueError("speed-committed source speed message type mismatch")
        raw_speeds, speed_times, accepted_deltas_sec = (
            load_embedded_causal_speed(source, max_speed_sync_delta_sec)
        )
        start, stop = 0, len(source)
        accepted = np.ones(len(source), dtype=bool)
        speed_indices = np.arange(len(source), dtype=np.int64)
        raw_speed_sample_count = len(raw_speeds)
    else:
        speed_times, raw_speeds = read_longitudinal_speed(
            source_bag, speed_topic, speed_message_type
        )
        matched_indices, deltas_ns = synchronize_data(
            source.scan_timestamps_ns, speed_times
        )
        max_delta_ns = int(round(max_speed_sync_delta_sec * 1e9))
        accepted = deltas_ns <= max_delta_ns
        start, stop = longest_true_run(accepted)
        source_slice = slice(start, stop)
        speed_indices = matched_indices[source_slice]
        accepted_deltas_sec = (
            deltas_ns[source_slice].astype(np.float64, copy=False) / 1e9
        )
        raw_speed_sample_count = len(speed_times)
    if stop - start < minimum_contiguous_samples:
        raise ValueError(
            f"longest synchronized interval too short in {source_dir}: "
            f"samples={stop - start}, minimum={minimum_contiguous_samples}"
        )

    source_slice = slice(start, stop)
    sequence_id = recurrent_sequence_id(
        source.sequence_id,
        speed_topic,
        max_speed_sync_delta_sec,
        outcome_certificate_sha,
    )
    output_dir = output_root / split / sequence_id
    if output_dir.exists():
        raise FileExistsError(f"derived output already exists: {output_dir}")
    output_dir.mkdir(parents=True)

    arrays = {
        "scans.npy": physical_scans[source_slice],
        "speeds.npy": raw_speeds[speed_indices].astype(np.float32, copy=False),
        "steers.npy": source.steers[source_slice].astype(np.float32, copy=False),
        "base_steers.npy": source.base_steers[source_slice].astype(
            np.float32, copy=False
        ),
        "scan_timestamps_ns.npy": source.scan_timestamps_ns[source_slice],
        "speed_timestamps_ns.npy": speed_times[speed_indices],
        "speed_sync_deltas_sec.npy": accepted_deltas_sec,
    }
    for filename, values in arrays.items():
        np.save(output_dir / filename, values)

    source_residual = source.metadata["relabeling"]["residual_target"]
    metadata = {
        "schema_version": RECURRENT_DATASET_SCHEMA_VERSION,
        "sequence_id": sequence_id,
        "split": split,
        "label_source": derived_label_source,
        "scan_unit": "m",
        "scan_shape": list(arrays["scans.npy"].shape[1:]),
        "max_scan_range_m": max_scan_range_m,
        "max_speed_sync_delta_sec": max_speed_sync_delta_sec,
        "source": {
            "dataset_root": str(source_dataset_root),
            "sequence_id": source.sequence_id,
            "sequence_dir": str(source_dir),
            "bag": str(source_bag),
            "label_source": source.metadata["label_source"],
            "control_mode": expected_control_mode,
            "successor_teacher": source_residual["successor_teacher"],
            "base_policy": source_residual["base_policy"],
            "student_checkpoint": source.metadata["relabeling"][
                "student_checkpoint"
            ],
            "student_checkpoint_sha256": source.metadata["relabeling"][
                "student_checkpoint_sha256"
            ],
            "outcome_certificate_sha256": outcome_certificate_sha,
        },
        "outcome_certificate": outcome_certificate,
        "outcome_certificate_sha256": outcome_certificate_sha,
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
            "raw_speed_samples": raw_speed_sample_count,
        },
        "source_interval": {"start_index": start, "stop_index": stop},
        "speed_sync_delta_sec": {
            "mean": float(np.mean(accepted_deltas_sec)),
            "p95": float(np.percentile(accepted_deltas_sec, 95)),
            "max": float(np.max(accepted_deltas_sec)),
        },
        "timestamp_ns": {
            "first_scan": int(arrays["scan_timestamps_ns.npy"][0]),
            "last_scan": int(arrays["scan_timestamps_ns.npy"][-1]),
        },
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument(
        "--additional-source-root",
        type=Path,
        action="append",
        default=[],
        help=(
            "Additional immutable source dataset; may be repeated. Sequence "
            "identities must remain unique across every root."
        ),
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--speed-topic", default=DEFAULT_SPEED_TOPIC)
    parser.add_argument("--speed-message-type", default=DEFAULT_SPEED_MESSAGE_TYPE)
    parser.add_argument(
        "--exclude-source-sequence-id",
        action="append",
        default=[],
        help=(
            "Explicit immutable source sequence to exclude; may be repeated. "
            "Every requested identity must exist exactly once."
        ),
    )
    parser.add_argument("--max-speed-sync-delta-sec", type=float, default=0.05)
    parser.add_argument("--minimum-contiguous-samples", type=int, default=64)
    parser.add_argument(
        "--require-executed-success",
        action="store_true",
        help=(
            "Reject every source sequence that lacks a valid embedded "
            "executed_teacher_success certificate."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source_roots = [
        args.source_root.expanduser().resolve(),
        *(path.expanduser().resolve() for path in args.additional_source_root),
    ]
    output_root = args.output_root.expanduser().resolve()
    if output_root.exists():
        raise FileExistsError(
            f"output root already exists; use a new immutable root: {output_root}"
        )
    if (
        not np.isfinite(args.max_speed_sync_delta_sec)
        or args.max_speed_sync_delta_sec <= 0.0
        or args.minimum_contiguous_samples <= 1
    ):
        raise ValueError("invalid recurrent synchronization configuration")

    results = []
    for source_root, split, source_dir in iter_source_sequences(
        source_roots,
        args.exclude_source_sequence_id,
        allow_partial_additional_roots=True,
    ):
        metadata = build_sequence(
            source_dir=source_dir,
            output_root=output_root,
            split=split,
            source_dataset_root=source_root,
            speed_topic=args.speed_topic,
            speed_message_type=args.speed_message_type,
            max_speed_sync_delta_sec=args.max_speed_sync_delta_sec,
            minimum_contiguous_samples=args.minimum_contiguous_samples,
            require_executed_success=args.require_executed_success,
        )
        results.append(metadata)
        print(
            f"built split={split} sequence={metadata['sequence_id']} "
            f"samples={metadata['counts']['accepted_samples']}"
        )

    manifest = {
        "schema_version": RECURRENT_DATASET_SCHEMA_VERSION,
        "source_dataset_root": str(source_roots[0]),
        "additional_source_dataset_roots": [
            str(path) for path in source_roots[1:]
        ],
        "speed_topic": args.speed_topic,
        "speed_message_type": args.speed_message_type,
        "max_speed_sync_delta_sec": args.max_speed_sync_delta_sec,
        "minimum_contiguous_samples": args.minimum_contiguous_samples,
        "require_executed_success": args.require_executed_success,
        "excluded_source_sequence_ids": sorted(
            args.exclude_source_sequence_id
        ),
        "sequence_ids": [metadata["sequence_id"] for metadata in results],
        "summary": {
            "sequences": len(results),
            "train_sequences": sum(item["split"] == "train" for item in results),
            "val_sequences": sum(item["split"] == "val" for item in results),
            "samples": sum(item["counts"]["accepted_samples"] for item in results),
        },
    }
    (output_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
