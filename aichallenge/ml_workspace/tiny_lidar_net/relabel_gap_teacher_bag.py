#!/usr/bin/env python3
"""Relabel admitted LiDAR states with an explicitly identified runtime teacher."""

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import sys
from typing import Optional

import numpy as np

from extract_data_from_bag import clean_scan_array, make_sequence_id
from lib.checkpoint import sha256_file
from lib.supervision import (
    admit_successful_run,
    source_domain_and_run,
    validate_executed_teacher_certificate,
)


@dataclass(frozen=True)
class TeacherIdentity:
    """Fields that jointly define the provenance of an offline teacher."""

    control_mode: str
    label_source: str
    teacher_class: str
    generated_control_type: str


TEACHER_IDENTITIES = {
    "gap_teacher": TeacherIdentity(
        control_mode="gap_teacher",
        label_source="lidar_gap_teacher_dagger",
        teacher_class="LidarGapTeacher",
        generated_control_type="generated/tiny_lidar_gap_teacher",
    ),
    "precontact_teacher": TeacherIdentity(
        control_mode="precontact_teacher",
        label_source="lidar_precontact_teacher_dagger",
        teacher_class="LidarPrecontactTeacher",
        generated_control_type="generated/tiny_lidar_precontact_teacher",
    ),
}


def teacher_identity(mode: str) -> TeacherIdentity:
    """Resolve all provenance fields from one validated control mode."""
    try:
        return TEACHER_IDENTITIES[mode]
    except KeyError:
        raise ValueError(f"unsupported teacher mode: {mode}") from None


def is_novel_policy_sample(
    teacher_steering_rad: float,
    reference_steering_rad: float,
    minimum_delta_rad: float,
) -> bool:
    """Return whether a successor teacher materially differs from its reference."""
    values = np.asarray(
        [teacher_steering_rad, reference_steering_rad, minimum_delta_rad],
        dtype=np.float64,
    )
    if not np.all(np.isfinite(values)) or minimum_delta_rad < 0.0:
        raise ValueError("novel-policy steering values must be finite and non-negative")
    return abs(teacher_steering_rad - reference_steering_rad) >= minimum_delta_rad


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


def cutoff_before_duration(
    timestamps_ns: np.ndarray,
    max_duration_sec: Optional[float],
) -> int:
    """Return an exclusive cutoff measured from the first admitted scan.

    Closed-loop DAgger bags may contain a terminal stalled state that is a
    consequence of an earlier policy error.  A caller-supplied duration keeps
    only the causal pre-failure prefix without changing the source bag.  The
    boundary is explicit metadata, not an inferred runtime heuristic.
    """
    timestamps = np.asarray(timestamps_ns, dtype=np.int64)
    if timestamps.ndim != 1 or timestamps.size == 0:
        raise ValueError("timestamps must be a non-empty one-dimensional array")
    if np.any(np.diff(timestamps) < 0):
        raise ValueError("timestamps must be monotonic")
    if max_duration_sec is None:
        return int(timestamps.size)
    if not np.isfinite(max_duration_sec) or max_duration_sec <= 0.0:
        raise ValueError("max_duration_sec must be finite and positive")
    cutoff_time = int(timestamps[0] + round(max_duration_sec * 1e9))
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
    from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig, LidarGapTeacher
    from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore

    bag = args.bag.expanduser().resolve()
    checkpoint = args.checkpoint.expanduser().resolve()
    identity = teacher_identity(args.teacher_mode)
    if args.novel_policy_only and args.teacher_mode != "precontact_teacher":
        raise ValueError(
            "--novel-policy-only requires --teacher-mode precontact_teacher"
        )
    if (
        not np.isfinite(args.minimum_novel_steering_delta_rad)
        or args.minimum_novel_steering_delta_rad < 0.0
    ):
        raise ValueError(
            "minimum novel steering delta must be finite and non-negative"
        )
    if not (bag / "metadata.yaml").is_file():
        raise ValueError(f"not a ROS 2 bag directory: {bag}")
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")

    checkpoint_sha = sha256_file(checkpoint)
    competition_analysis = getattr(args, "competition_analysis", None)
    require_executed_success = getattr(args, "require_executed_success", False)
    if require_executed_success and competition_analysis is None:
        raise ValueError(
            "--require-executed-success requires --competition-analysis"
        )
    outcome_certificate = None
    outcome_certificate_sha = None
    if competition_analysis is not None:
        domain, run_dir = source_domain_and_run(bag)
        admitted = admit_successful_run(
            run_dir,
            domain,
            identity.control_mode,
            checkpoint_sha,
            report_path=competition_analysis,
        )
        outcome_certificate = admitted["outcome_certificate"]
        outcome_certificate_sha = validate_executed_teacher_certificate(
            outcome_certificate,
            source_bag=bag,
            checkpoint_sha256=checkpoint_sha,
        )
        if outcome_certificate_sha != admitted["outcome_certificate_sha256"]:
            raise RuntimeError("outcome certificate identity is inconsistent")

    timestamps, scans = read_scans(bag, args.scan_topic, args.max_scan_range)
    minima = minimum_observed_ranges(scans)
    breach_index = first_confirmed_breach(
        minima, args.contact_clearance_m, args.contact_confirmation_samples
    )
    contact_cutoff = cutoff_before_margin(
        timestamps, breach_index, args.pre_contact_margin_sec
    )
    duration_cutoff = cutoff_before_duration(timestamps, args.max_duration_sec)
    cutoff = min(contact_cutoff, duration_cutoff)
    if cutoff <= 0:
        raise ValueError("admission cutoff removed the complete sequence")

    teacher_config = GapTeacherConfig()
    # A successor teacher is only meaningful relative to the historical
    # teacher evaluated on the exact same state and base steering.  Persist the
    # paired result for every precontact sample, not only when a positives-only
    # novelty filter happens to be requested.
    reference_teacher = (
        LidarGapTeacher(teacher_config)
        if args.teacher_mode == "precontact_teacher"
        else None
    )
    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(checkpoint),
        acceleration=args.fixed_acceleration,
        control_mode=identity.control_mode,
        max_range=args.max_scan_range,
        gap_teacher_config=teacher_config,
    )
    accepted_scans = []
    accepted_steering = []
    accepted_acceleration = []
    accepted_base_steering = []
    accepted_reference_steering = []
    accepted_steering_delta = []
    accepted_reference_steering_delta = []
    accepted_successor_upgrade_delta = []
    accepted_timestamps = []
    reason_counts = {}
    rejected_non_novel_samples = 0
    for timestamp, scan in zip(timestamps[:cutoff], scans[:cutoff]):
        acceleration, steering = core.process(scan)
        decision = core.last_gap_teacher_decision
        if decision is None:
            raise RuntimeError("gap teacher did not produce an auditable decision")
        reason_counts[decision.reason] = reason_counts.get(decision.reason, 0) + 1
        if args.active_only and not decision.active:
            continue
        reference_decision = None
        if reference_teacher is not None:
            reference_decision = reference_teacher.decide(
                scan,
                decision.base_steering_rad,
                args.fixed_acceleration,
            )
            if args.novel_policy_only and not is_novel_policy_sample(
                decision.steering_rad,
                reference_decision.steering_rad,
                args.minimum_novel_steering_delta_rad,
            ):
                rejected_non_novel_samples += 1
                continue
        accepted_scans.append(scan)
        accepted_steering.append(steering)
        accepted_acceleration.append(acceleration)
        accepted_base_steering.append(decision.base_steering_rad)
        if reference_decision is not None:
            accepted_reference_steering.append(reference_decision.steering_rad)
            accepted_steering_delta.append(
                decision.steering_rad - decision.base_steering_rad
            )
            accepted_reference_steering_delta.append(
                reference_decision.steering_rad - decision.base_steering_rad
            )
            accepted_successor_upgrade_delta.append(
                decision.steering_rad - reference_decision.steering_rad
            )
        accepted_timestamps.append(timestamp)
    if not accepted_scans:
        raise ValueError("no teacher correction samples survived admission")

    sequence_identity = (
        f"{bag}:dagger:{identity.control_mode}:{checkpoint_sha}:"
        f"outcome={outcome_certificate_sha or 'unproven'}:"
        f"{timestamps[cutoff - 1]}:active={args.active_only}:"
        f"novel={args.novel_policy_only}:"
        f"novel_delta={args.minimum_novel_steering_delta_rad}:"
        f"max_duration={args.max_duration_sec}:split={args.split}"
    )
    sequence_id = make_sequence_id(sequence_identity)
    output_dir = args.outdir.expanduser().resolve() / args.split / sequence_id
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
    residual_statistics = None
    if reference_teacher is not None:
        base_array = np.asarray(accepted_base_steering, dtype=np.float32)
        reference_array = np.asarray(
            accepted_reference_steering, dtype=np.float32
        )
        residual_array = np.asarray(accepted_steering_delta, dtype=np.float32)
        reference_residual_array = np.asarray(
            accepted_reference_steering_delta, dtype=np.float32
        )
        successor_upgrade_array = np.asarray(
            accepted_successor_upgrade_delta, dtype=np.float32
        )
        paired_arrays = {
            "base": base_array,
            "reference": reference_array,
            "residual": residual_array,
            "reference_residual": reference_residual_array,
            "successor_upgrade": successor_upgrade_array,
        }
        for name, values in paired_arrays.items():
            if values.shape != steering_array.shape:
                raise RuntimeError(
                    f"paired residual {name} steering shape mismatch"
                )
        if not np.allclose(
            steering_array - base_array,
            residual_array,
            rtol=1e-6,
            atol=1e-7,
        ):
            raise RuntimeError("runtime residual target identity mismatch")
        if not np.allclose(
            reference_array - base_array,
            reference_residual_array,
            rtol=1e-6,
            atol=1e-7,
        ):
            raise RuntimeError("reference teacher residual identity mismatch")
        if not np.allclose(
            steering_array - reference_array,
            successor_upgrade_array,
            rtol=1e-6,
            atol=1e-7,
        ):
            raise RuntimeError("successor teacher upgrade identity mismatch")
        np.save(output_dir / "base_steers.npy", base_array)
        np.save(output_dir / "reference_steers.npy", reference_array)
        np.save(output_dir / "steering_deltas.npy", residual_array)
        np.save(
            output_dir / "reference_steering_deltas.npy",
            reference_residual_array,
        )
        np.save(
            output_dir / "successor_upgrade_deltas.npy",
            successor_upgrade_array,
        )
        material = np.abs(residual_array) >= args.minimum_novel_steering_delta_rad
        upgrade_material = (
            np.abs(successor_upgrade_array)
            >= args.minimum_novel_steering_delta_rad
        )
        residual_statistics = {
            "target_definition": "successor_steering_minus_base_steering",
            "runtime_composition": "base_steering_plus_learned_residual",
            "successor_teacher": identity.teacher_class,
            "base_policy": "frozen_production_tiny_lidar_net",
            "material_delta_rad": args.minimum_novel_steering_delta_rad,
            "material_samples": int(np.count_nonzero(material)),
            "anchor_samples": int(len(residual_array) - np.count_nonzero(material)),
            "mean_abs_delta_rad": float(np.mean(np.abs(residual_array))),
            "max_abs_delta_rad": float(np.max(np.abs(residual_array))),
            "diagnostic_reference_teacher": "LidarGapTeacher",
            "reference_teacher_material_samples": int(
                np.count_nonzero(upgrade_material)
            ),
            "reference_teacher_mean_abs_upgrade_rad": float(
                np.mean(np.abs(successor_upgrade_array))
            ),
        }

    metadata = {
        "schema_version": 1,
        "sequence_id": sequence_id,
        "split": args.split,
        "source_bag": str(bag),
        "topics": {
            "scan": args.scan_topic,
            "control": f"offline_{identity.control_mode}",
        },
        "message_types": {
            "scan": "sensor_msgs/msg/LaserScan",
            "control": identity.generated_control_type,
        },
        "scan_shape": [750],
        "max_scan_range_m": args.max_scan_range,
        "max_sync_delta_sec": 0.0,
        "label_source": identity.label_source,
        "outcome_certificate": outcome_certificate,
        "outcome_certificate_sha256": outcome_certificate_sha,
        "counts": {
            "raw_scans": int(len(scans)),
            "pre_contact_scans": int(cutoff),
            "accepted_samples": int(len(scan_array)),
            "rejected_sync_samples": 0,
            "message_failures": 0,
            "rejected_non_novel_samples": rejected_non_novel_samples,
        },
        "sync_delta_sec": {"mean": 0.0, "p95": 0.0, "max": 0.0},
        "timestamp_ns": {
            "first_scan": int(timestamp_array[0]),
            "last_scan": int(timestamp_array[-1]),
        },
        "relabeling": {
            "student_checkpoint": str(checkpoint),
            "student_checkpoint_sha256": checkpoint_sha,
            "teacher": identity.teacher_class,
            "control_mode": identity.control_mode,
            "teacher_config": asdict(teacher_config),
            "active_only": args.active_only,
            "novel_policy_only": args.novel_policy_only,
            "minimum_novel_steering_delta_rad": (
                args.minimum_novel_steering_delta_rad
            ),
            "reference_teacher": (
                "LidarGapTeacher" if reference_teacher is not None else None
            ),
            "residual_target": residual_statistics,
            "decision_reason_counts_before_filter": reason_counts,
            "contact_clearance_m": args.contact_clearance_m,
            "contact_confirmation_samples": args.contact_confirmation_samples,
            "pre_contact_margin_sec": args.pre_contact_margin_sec,
            "breach_index": breach_index,
            "contact_cutoff_index": contact_cutoff,
            "max_duration_sec": args.max_duration_sec,
            "duration_cutoff_index": duration_cutoff,
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
    parser.add_argument(
        "--split",
        choices=("train", "val"),
        default="train",
        help="Run-level dataset split; source bags must not cross splits.",
    )
    parser.add_argument(
        "--teacher-mode",
        choices=tuple(TEACHER_IDENTITIES),
        default="gap_teacher",
        help="Offline teacher policy and its immutable dataset provenance.",
    )
    parser.add_argument("--contact-clearance-m", type=float, default=0.5)
    parser.add_argument("--contact-confirmation-samples", type=int, default=3)
    parser.add_argument("--pre-contact-margin-sec", type=float, default=1.0)
    parser.add_argument(
        "--max-duration-sec",
        type=float,
        help=(
            "Optional exclusive source-bag duration for a causal pre-failure "
            "DAgger prefix; combined with the contact cutoff by taking the "
            "earlier boundary."
        ),
    )
    parser.add_argument(
        "--active-only", action=argparse.BooleanOptionalAction, default=True
    )
    parser.add_argument(
        "--novel-policy-only",
        action="store_true",
        help=(
            "Keep only precontact-teacher labels that materially differ from "
            "the historical LidarGapTeacher on the same state."
        ),
    )
    parser.add_argument(
        "--minimum-novel-steering-delta-rad",
        type=float,
        default=0.02,
        help="Minimum steering difference for --novel-policy-only.",
    )
    parser.add_argument(
        "--competition-analysis",
        type=Path,
        help=(
            "Strict run-level outcome evidence. When supplied it must belong "
            "to the source bag and match the teacher mode, checkpoint, race "
            "and motion artifacts."
        ),
    )
    parser.add_argument(
        "--require-executed-success",
        action="store_true",
        help=(
            "Reject relabeling unless --competition-analysis proves that the "
            "exact precontact teacher executed and completed the run."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    metadata = relabel(args)
    print(json.dumps(metadata, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
