#!/usr/bin/env python3
"""Audit what the speed-committed teacher proves before forward escape.

This is a counterfactual, sequential replay tool.  It does not certify a run,
generate labels, or propose a runtime threshold.  Its purpose is to compare
successful and failed closed-loop bags using the exact same teacher state and
to distinguish a stopping-envelope violation from a missing trajectory proof.
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
from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore


def latest_preceding_indices(
    query_times_ns: np.ndarray,
    sample_times_ns: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return causal sample indices, ages and a preceding-sample mask."""
    query = np.asarray(query_times_ns, dtype=np.int64)
    samples = np.asarray(sample_times_ns, dtype=np.int64)
    if query.ndim != 1 or samples.ndim != 1 or samples.size == 0:
        raise ValueError("timestamps must be one-dimensional and samples non-empty")
    if np.any(np.diff(query) < 0) or np.any(np.diff(samples) <= 0):
        raise ValueError("timestamps must be monotonic")
    indices = np.searchsorted(samples, query, side="right") - 1
    valid = indices >= 0
    safe_indices = np.clip(indices, 0, samples.size - 1)
    ages = query - samples[safe_indices]
    return safe_indices, ages, valid


def longest_episode_sec(
    times_sec: np.ndarray,
    mask: np.ndarray,
    maximum_gap_sec: float = 0.25,
) -> float:
    """Return longest contiguous true interval without joining recorder gaps."""
    times = np.asarray(times_sec, dtype=np.float64)
    selected = np.asarray(mask, dtype=bool)
    if times.shape != selected.shape or times.ndim != 1:
        raise ValueError("times and mask must be aligned vectors")
    longest = 0.0
    start = None
    previous = None
    for timestamp, active in zip(times, selected):
        separated = previous is not None and timestamp - previous > maximum_gap_sec
        if active and (start is None or separated):
            if start is not None:
                longest = max(longest, previous - start)
            start = timestamp
        elif not active and start is not None:
            longest = max(longest, previous - start)
            start = None
        previous = timestamp
    if start is not None and previous is not None:
        longest = max(longest, previous - start)
    return float(longest)


def _distribution(values: Iterable[float]) -> dict:
    array = np.asarray(list(values), dtype=np.float64)
    if array.size == 0:
        return {"samples": 0}
    return {
        "samples": int(array.size),
        "minimum": float(np.min(array)),
        "p05": float(np.quantile(array, 0.05)),
        "p50": float(np.quantile(array, 0.50)),
        "p95": float(np.quantile(array, 0.95)),
        "maximum": float(np.max(array)),
    }


def summarize_replay(
    *,
    times_sec: np.ndarray,
    admitted_mask: np.ndarray,
    inside_stop_envelope: np.ndarray,
    committed_gap: np.ndarray,
    forward_command: np.ndarray,
    braking_command: np.ndarray,
    speeds_mps: np.ndarray,
    front_distances_m: np.ndarray,
    required_stop_distances_m: np.ndarray,
    reasons: list[str],
    supervisor_reasons: list[str],
    replay_acceleration_mps2: np.ndarray,
    published_acceleration_mps2: np.ndarray,
    failure_start_sec: float | None,
) -> dict:
    """Summarize aligned replay arrays without assigning causal success."""
    arrays = [
        admitted_mask,
        inside_stop_envelope,
        committed_gap,
        forward_command,
        braking_command,
        speeds_mps,
        front_distances_m,
        required_stop_distances_m,
        replay_acceleration_mps2,
        published_acceleration_mps2,
    ]
    if any(np.asarray(value).shape != times_sec.shape for value in arrays):
        raise ValueError("replay arrays must align")
    admitted = np.asarray(admitted_mask, dtype=bool)
    exposed = admitted & inside_stop_envelope
    unproven_forward = exposed & committed_gap & forward_command
    brake_exposed = exposed & braking_command
    parity_error = np.abs(
        replay_acceleration_mps2[admitted]
        - published_acceleration_mps2[admitted]
    )
    report = {
        "samples": int(times_sec.size),
        "admitted_samples": int(np.count_nonzero(admitted)),
        "rejected_speed_samples": int(np.count_nonzero(~admitted)),
        "inside_dynamic_stop_envelope": {
            "samples": int(np.count_nonzero(exposed)),
            "fraction_of_admitted": float(np.mean(inside_stop_envelope[admitted])),
            "longest_sec": longest_episode_sec(times_sec, exposed),
        },
        "committed_gap_forward_inside_dynamic_stop_envelope": {
            "samples": int(np.count_nonzero(unproven_forward)),
            "fraction_of_admitted": float(np.mean(unproven_forward[admitted])),
            "longest_sec": longest_episode_sec(times_sec, unproven_forward),
            "speed_mps": _distribution(speeds_mps[unproven_forward]),
            "front_distance_m": _distribution(front_distances_m[unproven_forward]),
            "required_stop_distance_m": _distribution(
                required_stop_distances_m[unproven_forward]
            ),
        },
        "braking_inside_dynamic_stop_envelope": {
            "samples": int(np.count_nonzero(brake_exposed)),
            "longest_sec": longest_episode_sec(times_sec, brake_exposed),
        },
        "decision_reason_counts": dict(sorted(Counter(reasons).items())),
        "supervisor_reason_counts": dict(
            sorted(Counter(supervisor_reasons).items())
        ),
        "published_acceleration_parity": {
            "mae_mps2": float(np.mean(parity_error)),
            "p95_abs_mps2": float(np.quantile(parity_error, 0.95)),
            "maximum_abs_mps2": float(np.max(parity_error)),
        },
    }
    if failure_start_sec is not None:
        prefix = admitted & (times_sec >= max(0.0, failure_start_sec - 20.0)) & (
            times_sec < failure_start_sec
        )
        report["pre_failure_20s"] = {
            "samples": int(np.count_nonzero(prefix)),
            "inside_dynamic_stop_envelope_fraction": (
                float(np.mean(inside_stop_envelope[prefix]))
                if np.any(prefix)
                else None
            ),
            "committed_gap_forward_samples": int(
                np.count_nonzero(prefix & unproven_forward)
            ),
            "committed_gap_forward_longest_sec": longest_episode_sec(
                times_sec, prefix & unproven_forward
            ),
        }
    return report


def replay_case(
    bag_path: Path,
    checkpoint: Path,
    fixed_acceleration_mps2: float,
    maximum_speed_age_sec: float,
    failure_start_sec: float | None,
) -> dict:
    arrays = read_replay_inputs(bag_path)
    scan_times = arrays["scan_times_ns"]
    speed_indices, speed_ages_ns, preceding = latest_preceding_indices(
        scan_times, arrays["wheel_times_ns"]
    )
    admitted = preceding & (
        speed_ages_ns <= int(round(maximum_speed_age_sec * 1e9))
    )
    relative = (scan_times - arrays["wheel_times_ns"][0]).astype(np.float64) / 1e9

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

    size = len(scan_times)
    inside = np.zeros(size, dtype=bool)
    committed_gap = np.zeros(size, dtype=bool)
    forward = np.zeros(size, dtype=bool)
    braking = np.zeros(size, dtype=bool)
    speed_values = np.full(size, np.nan, dtype=np.float64)
    front_values = np.full(size, np.nan, dtype=np.float64)
    stop_values = np.full(size, np.nan, dtype=np.float64)
    replay_acceleration = np.full(size, np.nan, dtype=np.float64)
    reasons = []
    supervisor_reasons = []

    for index, scan in enumerate(arrays["scans_m"]):
        if not admitted[index]:
            continue
        speed = float(arrays["wheel_speeds_mps"][speed_indices[index]])
        acceleration, _ = core.process(scan, speed_mps=speed)
        decision = core.last_gap_teacher_decision
        if decision is None:
            raise RuntimeError("teacher replay produced no auditable decision")
        speed_values[index] = speed
        front_values[index] = decision.front_distance_m
        stop_values[index] = decision.required_stop_distance_m
        replay_acceleration[index] = acceleration
        inside[index] = (
            decision.front_distance_m <= decision.required_stop_distance_m
        )
        committed_gap[index] = bool(
            decision.proposed_side_sign != 0
            and decision.proposed_side_sign == decision.committed_side_sign
            and decision.supervisor_reason
            in {"side-acquired", "side-maintained", "side-switch-confirmed"}
        )
        forward[index] = acceleration > 1e-6
        braking[index] = acceleration < -1e-6
        reasons.append(decision.reason)
        supervisor_reasons.append(decision.supervisor_reason)

    summary = summarize_replay(
        times_sec=relative,
        admitted_mask=admitted,
        inside_stop_envelope=inside,
        committed_gap=committed_gap,
        forward_command=forward,
        braking_command=braking,
        speeds_mps=speed_values,
        front_distances_m=front_values,
        required_stop_distances_m=stop_values,
        reasons=reasons,
        supervisor_reasons=supervisor_reasons,
        replay_acceleration_mps2=replay_acceleration,
        published_acceleration_mps2=arrays[
            "published_acceleration_at_scan_mps2"
        ].astype(np.float64),
        failure_start_sec=failure_start_sec,
    )
    summary["source_bag"] = str(bag_path)
    return summary


def parse_case(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("case must be LABEL=/path/to/bag")
    label, path = value.split("=", 1)
    if not label or not path:
        raise argparse.ArgumentTypeError("case label and path must be non-empty")
    return label, Path(path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", type=parse_case, required=True)
    parser.add_argument("--failed-label", required=True)
    parser.add_argument("--failure-start-sec", type=float, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixed-acceleration-mps2", type=float, default=0.8)
    parser.add_argument("--maximum-speed-age-sec", type=float, default=0.1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cases = dict(args.case)
    if len(cases) != len(args.case):
        raise ValueError("case labels must be unique")
    if args.failed_label not in cases:
        raise ValueError("failed label is not present in cases")
    if args.failure_start_sec <= 0.0:
        raise ValueError("failure start must be positive")
    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")

    reports = {}
    for label, bag in cases.items():
        bag_path = bag.expanduser().resolve()
        if not (bag_path / "metadata.yaml").is_file():
            raise ValueError(f"not a ROS 2 bag directory: {bag_path}")
        reports[label] = replay_case(
            bag_path,
            checkpoint,
            args.fixed_acceleration_mps2,
            args.maximum_speed_age_sec,
            args.failure_start_sec if label == args.failed_label else None,
        )

    success_labels = [label for label in reports if label != args.failed_label]
    failure_fraction = reports[args.failed_label][
        "committed_gap_forward_inside_dynamic_stop_envelope"
    ]["fraction_of_admitted"]
    success_fractions = [
        reports[label]["committed_gap_forward_inside_dynamic_stop_envelope"][
            "fraction_of_admitted"
        ]
        for label in success_labels
    ]
    dynamic_stop_predicate_discriminates = bool(
        success_fractions and max(success_fractions) < 1e-6 and failure_fraction > 0.0
    )
    result = {
        "schema_version": 1,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": sha256_file(checkpoint),
        "failed_label": args.failed_label,
        "failure_start_sec": args.failure_start_sec,
        "maximum_speed_age_sec": args.maximum_speed_age_sec,
        "cases": reports,
        "comparison": {
            "dynamic_stop_predicate_discriminates_failure": (
                dynamic_stop_predicate_discriminates
            ),
            "classification": (
                "dynamic-stop-envelope-defect"
                if dynamic_stop_predicate_discriminates
                else "instantaneous-gap-lacks-escape-certificate"
            ),
            "teacher_proof_fields": {
                "instantaneous_gap_angle": True,
                "committed_side_identity": True,
                "speed_dependent_stop_distance": True,
                "time_indexed_trajectory": False,
                "swept_vehicle_footprint": False,
                "dynamic_obstacle_prediction": False,
                "terminal_successor_viability": False,
            },
            "authority_change_permitted": False,
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
