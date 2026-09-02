#!/usr/bin/env python3
"""Audit a short-horizon swept-footprint alternative to polar gap labels.

The replay is diagnostic only.  It sequentially reconstructs the rejected
speed-committed teacher state, then evaluates a stateless bank of complete
shift/counter-shift/Stop candidates from selected current LiDAR frames.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
from typing import Iterable

import numpy as np

from audit_spatial_candidate_replay import read_replay_inputs
from lib.checkpoint import sha256_file
from lib.maneuver_rollout import (
    ManeuverCandidate,
    ManeuverRolloutConfig,
    evaluate_candidates,
    select_best_candidate,
)
from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore


DEFAULT_OFFSETS_RAD = (-0.32, -0.24, -0.16, 0.0, 0.16, 0.24, 0.32)


def latest_preceding_indices(
    query_times_ns: np.ndarray,
    sample_times_ns: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    query = np.asarray(query_times_ns, dtype=np.int64)
    samples = np.asarray(sample_times_ns, dtype=np.int64)
    if query.ndim != 1 or samples.ndim != 1 or samples.size == 0:
        raise ValueError("timestamps must be one-dimensional and samples non-empty")
    if np.any(np.diff(query) < 0) or np.any(np.diff(samples) <= 0):
        raise ValueError("timestamps must be monotonic")
    indices = np.searchsorted(samples, query, side="right") - 1
    valid = indices >= 0
    safe_indices = np.clip(indices, 0, samples.size - 1)
    return safe_indices, query - samples[safe_indices], valid


def _distribution(values: Iterable[float | None]) -> dict:
    raw_values = list(values)
    present = [value for value in raw_values if value is not None]
    array = np.asarray(present, dtype=np.float64)
    if array.size == 0:
        return {"samples": len(raw_values), "finite_samples": 0}
    finite = array[np.isfinite(array)]
    if finite.size == 0:
        return {"samples": int(array.size), "finite_samples": 0}
    return {
        "samples": int(array.size),
        "finite_samples": int(finite.size),
        "minimum": float(np.min(finite)),
        "p05": float(np.quantile(finite, 0.05)),
        "p50": float(np.quantile(finite, 0.50)),
        "p95": float(np.quantile(finite, 0.95)),
        "maximum": float(np.max(finite)),
    }


def _fraction(mask: np.ndarray) -> float:
    values = np.asarray(mask, dtype=bool)
    return float(np.mean(values)) if values.size else 0.0


def summarize_maneuver_evidence(records: list[dict]) -> dict:
    """Summarize candidate availability without inferring closed-loop success."""
    if not records:
        return {"evaluated_samples": 0}
    selected_available = np.asarray(
        [record["selected_side_available"] for record in records], dtype=bool
    )
    opposite_available = np.asarray(
        [record["opposite_side_available"] for record in records], dtype=bool
    )
    any_available = np.asarray(
        [record["any_candidate_available"] for record in records], dtype=bool
    )
    forward = np.asarray([record["teacher_forward"] for record in records], dtype=bool)
    selected_failed_opposite = (~selected_available) & opposite_available
    unproven_forward = forward & ~selected_available
    return {
        "evaluated_samples": len(records),
        "selected_side_available_fraction": _fraction(selected_available),
        "opposite_side_available_fraction": _fraction(opposite_available),
        "any_candidate_available_fraction": _fraction(any_available),
        "both_sides_available_fraction": _fraction(
            selected_available & opposite_available
        ),
        "no_candidate_available_fraction": _fraction(~any_available),
        "selected_infeasible_opposite_feasible": {
            "samples": int(np.count_nonzero(selected_failed_opposite)),
            "fraction": _fraction(selected_failed_opposite),
        },
        "teacher_forward_without_selected_certificate": {
            "samples": int(np.count_nonzero(unproven_forward)),
            "fraction": _fraction(unproven_forward),
        },
        "best_side_counts": dict(
            sorted(Counter(str(record["best_side_sign"]) for record in records).items())
        ),
        "teacher_side_counts": dict(
            sorted(Counter(str(record["teacher_side_sign"]) for record in records).items())
        ),
        "best_minimum_clearance_m": _distribution(
            record["best_minimum_clearance_m"] for record in records
        ),
        "speed_mps": _distribution(record["speed_mps"] for record in records),
        "front_distance_m": _distribution(
            record["front_distance_m"] for record in records
        ),
    }


def _select_evaluation_indices(
    relative_time_sec: np.ndarray,
    relevant: np.ndarray,
    failure_start_sec: float | None,
    failure_window_sec: float,
    maximum_samples: int,
) -> np.ndarray:
    mask = np.asarray(relevant, dtype=bool).copy()
    if failure_start_sec is not None:
        mask &= relative_time_sec >= failure_start_sec - failure_window_sec
        mask &= relative_time_sec < failure_start_sec
    indices = np.flatnonzero(mask)
    if len(indices) <= maximum_samples:
        return indices
    selection = np.linspace(0, len(indices) - 1, maximum_samples, dtype=int)
    return indices[selection]


def _side_available(candidates: list[ManeuverCandidate], side_sign: int) -> bool:
    return bool(
        side_sign != 0
        and any(candidate.feasible and candidate.side_sign == side_sign for candidate in candidates)
    )


def replay_case(
    bag_path: Path,
    checkpoint: Path,
    fixed_acceleration_mps2: float,
    maximum_speed_age_sec: float,
    maximum_samples: int,
    failure_start_sec: float | None,
    failure_window_sec: float,
    offsets_rad: tuple[float, ...],
    rollout_config: ManeuverRolloutConfig,
) -> dict:
    arrays = read_replay_inputs(bag_path)
    scan_times = arrays["scan_times_ns"]
    speed_indices, speed_ages_ns, preceding = latest_preceding_indices(
        scan_times, arrays["wheel_times_ns"]
    )
    admitted = preceding & (
        speed_ages_ns <= int(round(maximum_speed_age_sec * 1e9))
    )

    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(checkpoint),
        acceleration=fixed_acceleration_mps2,
        control_mode="speed_committed_teacher",
        max_range=30.0,
        gap_teacher_config=GapTeacherConfig(),
    )

    teacher_decisions = [None] * len(scan_times)
    relevant = np.zeros(len(scan_times), dtype=bool)
    for index, scan in enumerate(arrays["scans_m"]):
        if not admitted[index]:
            continue
        speed = float(arrays["wheel_speeds_mps"][speed_indices[index]])
        acceleration, _ = core.process(scan, speed_mps=speed)
        decision = core.last_gap_teacher_decision
        if decision is None:
            raise RuntimeError("teacher replay produced no decision")
        teacher_decisions[index] = (decision, float(acceleration), speed)
        relevant[index] = bool(
            decision.active
            and speed >= core.gap_teacher.config.minimum_commit_speed_mps
            and decision.front_distance_m <= decision.dynamic_trigger_distance_m
        )

    evaluation_indices = _select_evaluation_indices(
        arrays["relative_time_sec"],
        relevant,
        failure_start_sec,
        failure_window_sec,
        maximum_samples,
    )
    records = []
    for index in evaluation_indices:
        decision, acceleration, speed = teacher_decisions[index]
        candidates = evaluate_candidates(
            arrays["scans_m"][index],
            speed,
            float(arrays["published_steering_at_scan_rad"][index]),
            offsets_rad,
            rollout_config,
        )
        best = select_best_candidate(candidates)
        teacher_side = int(
            decision.committed_side_sign
            if decision.committed_side_sign != 0
            else decision.proposed_side_sign
        )
        opposite_side = -teacher_side
        records.append(
            {
                "relative_time_sec": float(arrays["relative_time_sec"][index]),
                "speed_mps": speed,
                "front_distance_m": float(decision.front_distance_m),
                "teacher_side_sign": teacher_side,
                "teacher_forward": acceleration > 1e-6,
                "selected_side_available": _side_available(candidates, teacher_side),
                "opposite_side_available": _side_available(candidates, opposite_side),
                "any_candidate_available": best is not None,
                "best_side_sign": 0 if best is None else best.side_sign,
                "best_steering_offset_rad": (
                    None if best is None else best.steering_offset_rad
                ),
                "best_minimum_clearance_m": (
                    None
                    if best is None or not np.isfinite(best.minimum_clearance_m)
                    else best.minimum_clearance_m
                ),
                "candidate_clearance_m": {
                    f"{candidate.steering_offset_rad:+.2f}": (
                        candidate.minimum_clearance_m
                        if np.isfinite(candidate.minimum_clearance_m)
                        else None
                    )
                    for candidate in candidates
                },
            }
        )

    summary = summarize_maneuver_evidence(records)
    summary.update(
        {
            "source_bag": str(bag_path),
            "admitted_speed_samples": int(np.count_nonzero(admitted)),
            "relevant_teacher_samples": int(np.count_nonzero(relevant)),
            "evaluation_window": {
                "failure_start_sec": failure_start_sec,
                "failure_window_sec": failure_window_sec if failure_start_sec else None,
                "maximum_samples": maximum_samples,
            },
            "records": records,
        }
    )
    return summary


def parse_case(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("case must be LABEL=/path/to/bag")
    label, path = value.split("=", 1)
    if not label or not path:
        raise argparse.ArgumentTypeError("case label and path must be non-empty")
    return label, Path(path)


def parse_offsets(value: str) -> tuple[float, ...]:
    try:
        offsets = tuple(float(item) for item in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("offsets must be comma-separated numbers") from exc
    if not offsets or not np.all(np.isfinite(offsets)):
        raise argparse.ArgumentTypeError("offsets must be finite and non-empty")
    return offsets


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", type=parse_case, required=True)
    parser.add_argument("--failed-label", required=True)
    parser.add_argument("--failure-start-sec", type=float, required=True)
    parser.add_argument("--failure-window-sec", type=float, default=20.0)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixed-acceleration-mps2", type=float, default=0.8)
    parser.add_argument("--maximum-speed-age-sec", type=float, default=0.1)
    parser.add_argument("--maximum-samples-per-case", type=int, default=300)
    parser.add_argument(
        "--steering-offsets-rad",
        type=parse_offsets,
        default=DEFAULT_OFFSETS_RAD,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cases = dict(args.case)
    if len(cases) != len(args.case):
        raise ValueError("case labels must be unique")
    if args.failed_label not in cases:
        raise ValueError("failed label is not present in cases")
    if args.failure_start_sec <= 0.0 or args.failure_window_sec <= 0.0:
        raise ValueError("failure timing must be positive")
    if args.maximum_samples_per_case <= 0:
        raise ValueError("maximum samples must be positive")
    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")

    rollout_config = ManeuverRolloutConfig()
    reports = {}
    for label, bag in cases.items():
        bag_path = bag.expanduser().resolve()
        if not (bag_path / "metadata.yaml").is_file():
            raise ValueError(f"not a ROS 2 bag directory: {bag_path}")
        reports[label] = replay_case(
            bag_path=bag_path,
            checkpoint=checkpoint,
            fixed_acceleration_mps2=args.fixed_acceleration_mps2,
            maximum_speed_age_sec=args.maximum_speed_age_sec,
            maximum_samples=args.maximum_samples_per_case,
            failure_start_sec=(
                args.failure_start_sec if label == args.failed_label else None
            ),
            failure_window_sec=args.failure_window_sec,
            offsets_rad=tuple(args.steering_offsets_rad),
            rollout_config=rollout_config,
        )

    failed = reports[args.failed_label]
    success = [report for label, report in reports.items() if label != args.failed_label]
    failed_opposite_fraction = failed.get(
        "selected_infeasible_opposite_feasible", {}
    ).get("fraction", 0.0)
    success_opposite_fractions = [
        report.get("selected_infeasible_opposite_feasible", {}).get("fraction", 0.0)
        for report in success
    ]
    discriminates = bool(
        failed_opposite_fraction > 0.05
        and (
            not success_opposite_fractions
            or failed_opposite_fraction > max(success_opposite_fractions) + 0.05
        )
    )
    result = {
        "schema_version": 1,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": sha256_file(checkpoint),
        "failed_label": args.failed_label,
        "failure_start_sec": args.failure_start_sec,
        "steering_offsets_rad": list(args.steering_offsets_rad),
        "rollout_config": rollout_config.__dict__,
        "cases": reports,
        "comparison": {
            "selected_infeasible_opposite_feasible_discriminates_failure": discriminates,
            "classification": (
                "instantaneous-side-selection-defect"
                if discriminates
                else "current-scan-swept-rollout-does-not-isolate-failure"
            ),
            "certificate_scope": {
                "current_scan": True,
                "kinematic_trajectory": True,
                "swept_vehicle_footprint": True,
                "terminal_stop_suffix": True,
                "future_peer_prediction": False,
                "occluded_wall_proof": False,
                "runtime_authority": False,
            },
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
