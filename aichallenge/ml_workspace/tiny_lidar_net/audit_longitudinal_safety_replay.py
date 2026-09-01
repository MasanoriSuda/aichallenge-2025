#!/usr/bin/env python3
"""Replay bounded longitudinal-safety policies on one immutable E2E bag.

The report is counterfactual and must not be used as closed-loop admission.
It compares the current distance-only policy with continuous safe-speed
envelopes while preserving the recorded LiDAR, speed and production pace
request.  Its purpose is to reject unsafe or globally over-conservative
formulations before changing runtime authority.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from audit_spatial_candidate_replay import read_replay_inputs
from tiny_lidar_net_controller.gap_teacher import (
    GapTeacherConfig,
    LidarLongitudinalSafety,
)


def safe_speed_from_clearance(
    front_distance_m: np.ndarray,
    minimum_clearance_m: float,
    effective_deceleration_mps2: float,
    reaction_time_sec: float,
) -> np.ndarray:
    """Invert a reaction-plus-braking-distance envelope.

    ``distance = v * reaction + v^2 / (2 * deceleration)`` is solved for the
    largest non-negative speed that preserves ``minimum_clearance_m``.
    """
    distance = np.asarray(front_distance_m, dtype=np.float64)
    values = (
        minimum_clearance_m,
        effective_deceleration_mps2,
        reaction_time_sec,
    )
    if not np.all(np.isfinite(distance)) or np.any(distance < 0.0):
        raise ValueError("front distance must be finite and non-negative")
    if any(not np.isfinite(value) for value in values):
        raise ValueError("safe-speed parameters must be finite")
    if minimum_clearance_m <= 0.0 or effective_deceleration_mps2 <= 0.0:
        raise ValueError("clearance and deceleration must be positive")
    if reaction_time_sec < 0.0:
        raise ValueError("reaction time must be non-negative")
    room = np.maximum(distance - minimum_clearance_m, 0.0)
    deceleration = float(effective_deceleration_mps2)
    reaction = float(reaction_time_sec)
    return (
        np.sqrt((deceleration * reaction) ** 2 + 2.0 * deceleration * room)
        - deceleration * reaction
    )


def production_pace_request(
    speed_mps: np.ndarray,
    requested_acceleration_mps2: float,
    maximum_forward_speed_mps: float,
) -> np.ndarray:
    """Reconstruct the qualified production governor's positive request."""
    speed = np.asarray(speed_mps, dtype=np.float64)
    if not np.all(np.isfinite(speed)) or np.any(speed < 0.0):
        raise ValueError("speed must be finite and non-negative")
    if (
        not np.isfinite(requested_acceleration_mps2)
        or requested_acceleration_mps2 < 0.0
        or not np.isfinite(maximum_forward_speed_mps)
        or maximum_forward_speed_mps <= 0.0
    ):
        raise ValueError("invalid production pace contract")
    return np.minimum(
        requested_acceleration_mps2,
        np.maximum(0.0, maximum_forward_speed_mps - speed),
    )


def safe_speed_acceleration(
    front_distance_m: np.ndarray,
    speed_mps: np.ndarray,
    requested_acceleration_mps2: np.ndarray,
    minimum_clearance_m: float,
    effective_deceleration_mps2: float,
    reaction_time_sec: float,
    brake_acceleration_mps2: float,
    speed_error_gain: float,
    activation_distance_m: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Limit acceleration using the continuous safe-speed envelope."""
    speed = np.asarray(speed_mps, dtype=np.float64)
    requested = np.asarray(requested_acceleration_mps2, dtype=np.float64)
    front = np.asarray(front_distance_m, dtype=np.float64)
    if speed.shape != front.shape or requested.shape != front.shape:
        raise ValueError("front, speed and acceleration arrays must align")
    if (
        not np.isfinite(brake_acceleration_mps2)
        or brake_acceleration_mps2 >= 0.0
        or not np.isfinite(speed_error_gain)
        or speed_error_gain <= 0.0
    ):
        raise ValueError("invalid braking or speed-error gain")
    if activation_distance_m is not None and (
        not np.isfinite(activation_distance_m)
        or activation_distance_m <= minimum_clearance_m
    ):
        raise ValueError("activation distance must exceed minimum clearance")
    safe_speed = safe_speed_from_clearance(
        front,
        minimum_clearance_m,
        effective_deceleration_mps2,
        reaction_time_sec,
    )
    speed_limit_acceleration = np.maximum(
        brake_acceleration_mps2,
        speed_error_gain * (safe_speed - speed),
    )
    acceleration = np.minimum(requested, speed_limit_acceleration)
    acceleration = np.where(
        front <= minimum_clearance_m,
        brake_acceleration_mps2,
        acceleration,
    )
    if activation_distance_m is not None:
        acceleration = np.where(
            front <= activation_distance_m,
            acceleration,
            requested,
        )
    return acceleration.astype(np.float64), safe_speed.astype(np.float64)


def front_distances(scans_m: np.ndarray, config: GapTeacherConfig) -> np.ndarray:
    """Evaluate the exact production percentile-based frontal clearance."""
    safety = LidarLongitudinalSafety(config)
    return np.asarray(
        [safety.front_distance(scan) for scan in scans_m], dtype=np.float64
    )


def longest_duration_sec(
    relative_time_sec: np.ndarray,
    mask: np.ndarray,
    maximum_gap_sec: float = 0.25,
) -> float:
    """Return the longest true duration without joining recorder gaps."""
    times = np.asarray(relative_time_sec, dtype=np.float64)
    selected = np.asarray(mask, dtype=bool)
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


def summarize_policy(
    relative_time_sec: np.ndarray,
    requested_acceleration_mps2: np.ndarray,
    acceleration_mps2: np.ndarray,
    safe_speed_mps: np.ndarray | None,
    mask: np.ndarray,
    failure_start_sec: float | None,
) -> dict:
    """Summarize intervention without claiming a changed closed-loop outcome."""
    times = np.asarray(relative_time_sec, dtype=np.float64)[mask]
    requested = np.asarray(requested_acceleration_mps2, dtype=np.float64)[mask]
    acceleration = np.asarray(acceleration_mps2, dtype=np.float64)[mask]
    if len(times) == 0:
        return {"samples": 0}
    intervention = acceleration < requested - 1e-6
    braking = acceleration < -1e-6
    zero_hold = (np.abs(acceleration) <= 1e-6) & (requested > 1e-6)
    positive_limited = (
        (acceleration > 1e-6)
        & (acceleration < requested - 1e-6)
    )
    first_intervention = (
        float(times[np.flatnonzero(intervention)[0]])
        if np.any(intervention)
        else None
    )
    report = {
        "samples": int(len(times)),
        "intervention_fraction": float(np.mean(intervention)),
        "brake_fraction": float(np.mean(braking)),
        "zero_hold_fraction": float(np.mean(zero_hold)),
        "positive_limited_fraction": float(np.mean(positive_limited)),
        "longest_intervention_sec": longest_duration_sec(
            times, intervention
        ),
        "mean_acceleration_mps2": float(np.mean(acceleration)),
        "minimum_acceleration_mps2": float(np.min(acceleration)),
        "first_intervention_sec": first_intervention,
        "lead_before_failure_sec": (
            None
            if first_intervention is None or failure_start_sec is None
            else float(failure_start_sec - first_intervention)
        ),
    }
    if safe_speed_mps is not None:
        selected_safe_speed = np.asarray(safe_speed_mps, dtype=np.float64)[mask]
        report["safe_speed_mps"] = {
            "minimum": float(np.min(selected_safe_speed)),
            "p05": float(np.quantile(selected_safe_speed, 0.05)),
            "p50": float(np.quantile(selected_safe_speed, 0.50)),
        }
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--failure-start-sec", type=float)
    parser.add_argument(
        "--effective-deceleration-mps2",
        type=float,
        action="append",
        default=[],
        help="Repeat to compare physical envelope assumptions.",
    )
    parser.add_argument("--reaction-time-sec", type=float, default=0.25)
    parser.add_argument("--minimum-clearance-m", type=float, default=1.5)
    parser.add_argument("--activation-distance-m", type=float, default=3.0)
    parser.add_argument("--brake-acceleration-mps2", type=float, default=-1.0)
    parser.add_argument("--speed-error-gain", type=float, default=1.0)
    parser.add_argument("--requested-acceleration-mps2", type=float, default=0.8)
    parser.add_argument("--maximum-forward-speed-mps", type=float, default=4.6)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    decelerations = args.effective_deceleration_mps2 or [1.0, 2.0, 3.0]
    if args.failure_start_sec is not None and args.failure_start_sec < 0.0:
        raise ValueError("failure start must be non-negative")
    if any(not math.isfinite(value) or value <= 0.0 for value in decelerations):
        raise ValueError("effective deceleration must be finite and positive")

    arrays = read_replay_inputs(args.bag.expanduser().resolve())
    relative = arrays["relative_time_sec"]
    speed = arrays["wheel_at_scan_mps"].astype(np.float64)
    requested = production_pace_request(
        speed,
        args.requested_acceleration_mps2,
        args.maximum_forward_speed_mps,
    )
    gap_config = GapTeacherConfig(
        stop_distance_m=args.minimum_clearance_m,
        brake_acceleration_mps2=args.brake_acceleration_mps2,
    )
    front = front_distances(arrays["scans_m"], gap_config)
    fixed_safety = LidarLongitudinalSafety(gap_config)
    fixed = np.asarray(
        [
            fixed_safety.decide_from_front_distance(distance, request).acceleration_mps2
            for distance, request in zip(front, requested)
        ],
        dtype=np.float64,
    )

    if args.failure_start_sec is None:
        windows = {"all": np.ones(len(relative), dtype=bool)}
    else:
        failure = float(args.failure_start_sec)
        windows = {
            "all": np.ones(len(relative), dtype=bool),
            "pre_failure_20s": (relative >= max(0.0, failure - 20.0))
            & (relative < failure),
            "post_failure_20s": (relative >= failure)
            & (relative < failure + 20.0),
        }

    policies = {
        "fixed_distance": {
            window: summarize_policy(
                relative,
                requested,
                fixed,
                None,
                mask,
                args.failure_start_sec,
            )
            for window, mask in windows.items()
        }
    }
    for deceleration in decelerations:
        acceleration, safe_speed = safe_speed_acceleration(
            front,
            speed,
            requested,
            args.minimum_clearance_m,
            deceleration,
            args.reaction_time_sec,
            args.brake_acceleration_mps2,
            args.speed_error_gain,
        )
        policies[f"safe_speed_decel_{deceleration:g}"] = {
            window: summarize_policy(
                relative,
                requested,
                acceleration,
                safe_speed,
                mask,
                args.failure_start_sec,
            )
            for window, mask in windows.items()
        }
        gated_acceleration, gated_safe_speed = safe_speed_acceleration(
            front,
            speed,
            requested,
            args.minimum_clearance_m,
            deceleration,
            args.reaction_time_sec,
            args.brake_acceleration_mps2,
            args.speed_error_gain,
            args.activation_distance_m,
        )
        policies[f"slow_zone_safe_speed_decel_{deceleration:g}"] = {
            window: summarize_policy(
                relative,
                requested,
                gated_acceleration,
                gated_safe_speed,
                mask,
                args.failure_start_sec,
            )
            for window, mask in windows.items()
        }

    actual_error = fixed - arrays["published_acceleration_at_scan_mps2"]
    report = {
        "schema_version": 1,
        "source_bag": str(args.bag.expanduser().resolve()),
        "failure_start_sec": args.failure_start_sec,
        "samples": int(len(relative)),
        "parameters": {
            "effective_decelerations_mps2": decelerations,
            "reaction_time_sec": args.reaction_time_sec,
            "minimum_clearance_m": args.minimum_clearance_m,
            "activation_distance_m": args.activation_distance_m,
            "brake_acceleration_mps2": args.brake_acceleration_mps2,
            "speed_error_gain": args.speed_error_gain,
            "requested_acceleration_mps2": args.requested_acceleration_mps2,
            "maximum_forward_speed_mps": args.maximum_forward_speed_mps,
        },
        "fixed_runtime_parity": {
            "mae_mps2": float(np.mean(np.abs(actual_error))),
            "p95_abs_mps2": float(np.quantile(np.abs(actual_error), 0.95)),
        },
        "front_distance_m": {
            "minimum": float(np.min(front)),
            "p05": float(np.quantile(front, 0.05)),
            "p50": float(np.quantile(front, 0.50)),
        },
        "policies": policies,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
