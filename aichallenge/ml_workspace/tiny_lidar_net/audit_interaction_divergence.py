#!/usr/bin/env python3
"""Compare pre-contact lateral decisions across immutable E2E run domains."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from audit_spatial_candidate_replay import (
    load_candidate,
    predict,
    predict_base_steering,
    read_replay_inputs,
)
from lib.checkpoint import sha256_file


SCHEMA_VERSION = 1


def parse_case(value: str) -> tuple[str, Path]:
    """Parse NAME=PATH without inferring an identity from the directory name."""
    name, separator, path = value.partition("=")
    if not separator or not name or not path:
        raise argparse.ArgumentTypeError("case must be NAME=PATH")
    return name, Path(path)


def first_sustained_interval(
    times_sec: np.ndarray,
    mask: np.ndarray,
    minimum_duration_sec: float,
    maximum_sample_gap_sec: float = 0.25,
) -> tuple[int, int] | None:
    """Return the first true interval reaching the requested duration."""
    times = np.asarray(times_sec, dtype=np.float64)
    selected = np.asarray(mask, dtype=bool)
    if times.ndim != 1 or selected.shape != times.shape:
        raise ValueError("times and mask must be aligned one-dimensional arrays")
    if np.any(~np.isfinite(times)) or np.any(np.diff(times) < 0.0):
        raise ValueError("times must be finite and monotonic")
    if minimum_duration_sec <= 0.0 or maximum_sample_gap_sec <= 0.0:
        raise ValueError("interval durations must be positive")

    start = None
    previous = None
    for index, active in enumerate(selected):
        separated = (
            previous is not None
            and times[index] - times[previous] > maximum_sample_gap_sec
        )
        if not active or separated:
            start = index if active else None
        elif start is None:
            start = index
        if start is not None and times[index] - times[start] >= minimum_duration_sec:
            end = index
            while (
                end + 1 < len(times)
                and selected[end + 1]
                and times[end + 1] - times[end] <= maximum_sample_gap_sec
            ):
                end += 1
            return start, end
        previous = index
    return None


def contiguous_episode_count(
    times_sec: np.ndarray,
    mask: np.ndarray,
    maximum_sample_gap_sec: float = 0.25,
) -> int:
    """Count non-empty true intervals without joining recorder gaps."""
    times = np.asarray(times_sec, dtype=np.float64)
    selected = np.asarray(mask, dtype=bool)
    if times.ndim != 1 or selected.shape != times.shape:
        raise ValueError("times and mask must be aligned one-dimensional arrays")
    count = 0
    active = False
    previous = None
    for index, value in enumerate(selected):
        separated = (
            previous is not None
            and times[index] - times[previous] > maximum_sample_gap_sec
        )
        if value and (not active or separated):
            count += 1
        active = bool(value)
        previous = index
    return count


def count_sign_flips(
    times_sec: np.ndarray,
    signs: np.ndarray,
    mask: np.ndarray,
    maximum_sample_gap_sec: float = 0.25,
) -> int:
    """Count non-zero sign changes without joining distinct hazard episodes."""
    times = np.asarray(times_sec, dtype=np.float64)
    values = np.asarray(signs, dtype=np.int8)
    selected = np.asarray(mask, dtype=bool)
    if times.ndim != 1 or values.shape != times.shape or selected.shape != times.shape:
        raise ValueError("times, signs and mask must be aligned one-dimensional arrays")
    if np.any(~np.isfinite(times)) or np.any(np.diff(times) < 0.0):
        raise ValueError("times must be finite and monotonic")
    if maximum_sample_gap_sec <= 0.0:
        raise ValueError("maximum sample gap must be positive")

    flips = 0
    previous_index = None
    previous_sign = 0
    for index in np.flatnonzero(selected):
        sign = int(values[index])
        separated = (
            previous_index is not None
            and (
                index != previous_index + 1
                or times[index] - times[previous_index] > maximum_sample_gap_sec
            )
        )
        if separated:
            previous_sign = 0
        if sign != 0:
            if previous_sign != 0 and sign != previous_sign:
                flips += 1
            previous_sign = sign
        previous_index = int(index)
    return flips


def summarize_window(trace: dict, mask: np.ndarray) -> dict:
    """Summarize one causal window without treating the teacher as truth."""
    selected = np.asarray(mask, dtype=bool)
    sample_count = int(np.count_nonzero(selected))
    if sample_count == 0:
        return {"samples": 0}

    side_hazard = selected & trace["side_hazard"]
    hazard_count = int(np.count_nonzero(side_hazard))
    report = {
        "samples": sample_count,
        "speed_mean_mps": float(np.mean(trace["speed_mps"][selected])),
        "published_steering_mean_rad": float(
            np.mean(trace["published_steering_rad"][selected])
        ),
        "teacher_reason_counts": dict(
            sorted(Counter(trace["teacher_reason"][selected]).items())
        ),
        "side_hazard_samples": hazard_count,
        "side_hazard_episodes": contiguous_episode_count(
            trace["time_sec"], side_hazard
        ),
        "escape_side_flips": count_sign_flips(
            trace["time_sec"], trace["escape_sign"], side_hazard
        ),
    }
    if hazard_count:
        teacher_target = trace["teacher_residual_rad"][side_hazard]
        predicted = trace["predicted_residual_rad"][side_hazard]
        published = trace["published_steering_rad"][side_hazard]
        teacher_steering = trace["teacher_steering_rad"][side_hazard]
        escape_sign = trace["escape_sign"][side_hazard]
        report.update(
            {
                "side_hazard_duration_sec": float(
                    trace["time_sec"][np.flatnonzero(side_hazard)[-1]]
                    - trace["time_sec"][np.flatnonzero(side_hazard)[0]]
                ),
                "teacher_residual_mean_rad": float(np.mean(teacher_target)),
                "predicted_residual_mean_rad": float(np.mean(predicted)),
                "teacher_residual_mae_rad": float(
                    np.mean(np.abs(predicted - teacher_target))
                ),
                "toward_obstacle_fraction": float(
                    np.mean(published * escape_sign < -0.02)
                ),
                "teacher_projection_deficit_fraction": float(
                    np.mean(
                        (teacher_steering - published) * escape_sign > 0.05
                    )
                ),
                "left_side_distance_p05_m": float(
                    np.quantile(trace["left_side_m"][side_hazard], 0.05)
                ),
                "right_side_distance_p05_m": float(
                    np.quantile(trace["right_side_m"][side_hazard], 0.05)
                ),
            }
        )
    return report


def build_teacher_trace(scans_m: np.ndarray, base_steering_rad: np.ndarray) -> dict:
    """Replay the immutable diagnostic teacher and retain physical evidence."""
    from tiny_lidar_net_controller.gap_teacher import (
        GapTeacherConfig,
        LidarPrecontactTeacher,
    )

    teacher = LidarPrecontactTeacher(GapTeacherConfig())
    reasons = []
    teacher_steering = []
    left_side = []
    right_side = []
    escape_sign = []
    for scan, base in zip(scans_m, base_steering_rad):
        decision = teacher.decide(scan, float(base), 0.8)
        reasons.append(decision.reason)
        teacher_steering.append(decision.steering_rad)
        left_side.append(decision.left_side_distance_m)
        right_side.append(decision.right_side_distance_m)
        if decision.reason != "side-clearance":
            escape_sign.append(0)
        elif decision.right_side_distance_m < decision.left_side_distance_m:
            escape_sign.append(1)
        else:
            escape_sign.append(-1)
    return {
        "reason": np.asarray(reasons, dtype=str),
        "steering_rad": np.asarray(teacher_steering, dtype=np.float32),
        "left_side_m": np.asarray(left_side, dtype=np.float32),
        "right_side_m": np.asarray(right_side, dtype=np.float32),
        "escape_sign": np.asarray(escape_sign, dtype=np.int8),
    }


def audit_case(
    name: str,
    bag_path: Path,
    model,
    batch_size: int,
    stall_speed_mps: float,
    stall_duration_sec: float,
) -> dict:
    arrays = read_replay_inputs(bag_path)
    scans = arrays["scans_m"]
    speed = arrays["wheel_at_scan_mps"]
    base = predict_base_steering(model, scans, batch_size)
    predicted, _ = predict(model, scans, speed, batch_size)
    teacher = build_teacher_trace(scans, base)
    times = arrays["relative_time_sec"]
    moved = np.maximum.accumulate(speed >= 1.0)
    stall = first_sustained_interval(
        times,
        moved & (speed <= stall_speed_mps),
        stall_duration_sec,
    )
    stall_start = None if stall is None else float(times[stall[0]])
    before_stall = moved.copy()
    if stall_start is not None:
        before_stall &= times < stall_start
        comparison = moved & (times >= max(times[0], stall_start - 10.0))
        comparison &= times < stall_start
        comparison_kind = "pre_stall_10s"
        aftermath = times >= stall_start
        aftermath &= times < stall_start + 10.0
    else:
        comparison = moved & (times < times[np.flatnonzero(moved)[0]] + 60.0)
        comparison_kind = "first_motion_60s"
        aftermath = np.zeros(len(times), dtype=bool)

    side_hazard = teacher["reason"] == "side-clearance"
    trace = {
        "time_sec": times,
        "speed_mps": speed,
        "published_steering_rad": arrays["published_steering_at_scan_rad"],
        "predicted_residual_rad": predicted,
        "teacher_residual_rad": teacher["steering_rad"] - base,
        "teacher_steering_rad": teacher["steering_rad"],
        "teacher_reason": teacher["reason"],
        "left_side_m": teacher["left_side_m"],
        "right_side_m": teacher["right_side_m"],
        "escape_sign": teacher["escape_sign"],
        "side_hazard": side_hazard,
    }
    return {
        "name": name,
        "source_bag": str(bag_path),
        "samples": int(len(times)),
        "first_motion_sec": (
            None if not np.any(moved) else float(times[np.flatnonzero(moved)[0]])
        ),
        "first_sustained_stall_sec": stall_start,
        "recovered_after_stall": bool(
            stall is not None and np.any(speed[stall[1] + 1 :] >= 1.0)
        ),
        "comparison_window_kind": comparison_kind,
        "windows": {
            "before_stall": summarize_window(trace, before_stall),
            "comparison": summarize_window(trace, comparison),
            "aftermath_10s": summarize_window(trace, aftermath),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", action="append", type=parse_case, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--candidate-use-base-steering", action="store_true")
    parser.add_argument("--candidate-max-abs-delta-rad", type=float, default=1.2)
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--stall-speed-mps", type=float, default=0.15)
    parser.add_argument("--stall-duration-sec", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.batch_size <= 0 or args.stall_duration_sec <= 0.0:
        raise ValueError("batch size and stall duration must be positive")
    if args.stall_speed_mps < 0.0:
        raise ValueError("stall speed must be non-negative")

    candidate = args.candidate.expanduser().resolve()
    model = load_candidate(
        candidate,
        use_base_steering=args.candidate_use_base_steering,
        max_abs_delta_rad=args.candidate_max_abs_delta_rad,
    )
    cases = [
        audit_case(
            name,
            path.expanduser().resolve(),
            model,
            args.batch_size,
            args.stall_speed_mps,
            args.stall_duration_sec,
        )
        for name, path in args.case
    ]
    result = {
        "schema_version": SCHEMA_VERSION,
        "purpose": "offline interaction divergence audit",
        "candidate": {
            "path": str(candidate),
            "sha256": sha256_file(candidate),
            "use_base_steering": args.candidate_use_base_steering,
            "max_abs_delta_rad": args.candidate_max_abs_delta_rad,
        },
        "thresholds": {
            "stall_speed_mps": args.stall_speed_mps,
            "stall_duration_sec": args.stall_duration_sec,
            "toward_obstacle_deadband_rad": 0.02,
            "teacher_projection_deficit_rad": 0.05,
        },
        "cases": cases,
    }
    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
