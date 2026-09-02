#!/usr/bin/env python3
"""Privileged future-scan oracle for the frozen E2E manoeuvre candidates.

Future LiDAR and odometry are used only to test whether temporal occupancy is
the missing teacher information.  The tool is not a runtime controller and its
output is not an admissible E2E input.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader

from audit_spatial_candidate_replay import read_replay_inputs
from audit_swept_maneuver_certificate import (
    DEFAULT_OFFSETS_RAD,
    _select_evaluation_indices,
    _side_available,
    latest_preceding_indices,
    parse_case,
    parse_offsets,
    summarize_maneuver_evidence,
)
from lib.checkpoint import sha256_file
from lib.maneuver_rollout import (
    ManeuverRolloutConfig,
    evaluate_candidate_against_time_indexed_points,
    rollout_lane_change_stop,
    scan_points_in_base,
    select_best_candidate,
)
from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
from tiny_lidar_net_controller.tiny_lidar_net_controller_core import TinyLidarNetCore


POSE_TOPIC = "/localization/kinematic_state"
MINIMUM_SUCCESS_CANDIDATE_COVERAGE = 0.5
MINIMUM_FAILURE_SIGNAL_FRACTION = 0.05
MINIMUM_FAILURE_SIGNAL_DELTA = 0.05


def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    values = np.asarray([x, y, z, w], dtype=np.float64)
    if not np.all(np.isfinite(values)) or np.linalg.norm(values) <= 1e-12:
        raise ValueError("pose quaternion must be finite and non-zero")
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def read_pose_stream(bag_path: Path) -> tuple[np.ndarray, np.ndarray]:
    timestamps = []
    poses = []
    with AnyReader([bag_path]) as reader:
        connections = [
            connection
            for connection in reader.connections
            if connection.topic == POSE_TOPIC
        ]
        if not connections:
            raise ValueError(f"required pose topic missing: {POSE_TOPIC}")
        for connection, timestamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            pose = message.pose.pose
            timestamps.append(timestamp)
            poses.append(
                (
                    float(pose.position.x),
                    float(pose.position.y),
                    quaternion_yaw(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                    ),
                )
            )
    times = np.asarray(timestamps, dtype=np.int64)
    pose_array = np.asarray(poses, dtype=np.float64)
    if (
        times.size == 0
        or pose_array.shape != (times.size, 3)
        or np.any(np.diff(times) <= 0)
        or not np.all(np.isfinite(pose_array))
    ):
        raise ValueError("pose stream is empty, misaligned or non-finite")
    return times, pose_array


def transform_points_to_reference(
    points_future_base_m: np.ndarray,
    future_pose_xyyaw: np.ndarray,
    reference_pose_xyyaw: np.ndarray,
) -> np.ndarray:
    """Transform future base-frame points into the reference base frame."""
    points = np.asarray(points_future_base_m, dtype=np.float64)
    future = np.asarray(future_pose_xyyaw, dtype=np.float64)
    reference = np.asarray(reference_pose_xyyaw, dtype=np.float64)
    if points.ndim != 2 or points.shape[1:] != (2,) or not np.all(np.isfinite(points)):
        raise ValueError("points must be a finite Nx2 array")
    if future.shape != (3,) or reference.shape != (3,) or not (
        np.all(np.isfinite(future)) and np.all(np.isfinite(reference))
    ):
        raise ValueError("poses must be finite [x, y, yaw] vectors")
    if len(points) == 0:
        return points.copy()

    future_cos = math.cos(float(future[2]))
    future_sin = math.sin(float(future[2]))
    world_x = future[0] + future_cos * points[:, 0] - future_sin * points[:, 1]
    world_y = future[1] + future_sin * points[:, 0] + future_cos * points[:, 1]
    dx = world_x - reference[0]
    dy = world_y - reference[1]
    reference_cos = math.cos(float(reference[2]))
    reference_sin = math.sin(float(reference[2]))
    return np.column_stack(
        (
            reference_cos * dx + reference_sin * dy,
            -reference_sin * dx + reference_cos * dy,
        )
    )


def future_point_clouds_for_rollout(
    *,
    current_scan_index: int,
    state_count: int,
    scan_times_ns: np.ndarray,
    scans_m: np.ndarray,
    pose_times_ns: np.ndarray,
    poses_xyyaw: np.ndarray,
    config: ManeuverRolloutConfig,
    maximum_pose_age_sec: float,
) -> list[np.ndarray] | None:
    """Build one time-matched point cloud for every rollout state."""
    if state_count <= 0:
        raise ValueError("state_count must be positive")
    current_time = int(scan_times_ns[current_scan_index])
    target_times = current_time + np.rint(
        np.arange(state_count, dtype=np.float64)
        * config.integration_step_sec
        * 1e9
    ).astype(np.int64)
    future_indices = np.searchsorted(scan_times_ns, target_times, side="left")
    if np.any(future_indices >= len(scan_times_ns)):
        return None
    future_indices = np.maximum(future_indices, current_scan_index)
    pose_indices, pose_ages_ns, preceding = latest_preceding_indices(
        scan_times_ns[future_indices], pose_times_ns
    )
    pose_valid = preceding & (
        pose_ages_ns <= int(round(maximum_pose_age_sec * 1e9))
    )
    reference_indices, reference_ages_ns, reference_preceding = latest_preceding_indices(
        np.asarray([current_time], dtype=np.int64), pose_times_ns
    )
    if not bool(reference_preceding[0]) or (
        reference_ages_ns[0] > int(round(maximum_pose_age_sec * 1e9))
    ) or not np.all(pose_valid):
        return None
    reference_pose = poses_xyyaw[reference_indices[0]]
    clouds = []
    for scan_index, pose_index in zip(future_indices, pose_indices):
        future_points = scan_points_in_base(scans_m[scan_index], config)
        clouds.append(
            transform_points_to_reference(
                future_points,
                poses_xyyaw[pose_index],
                reference_pose,
            )
        )
    return clouds


def replay_case(
    *,
    bag_path: Path,
    checkpoint: Path,
    fixed_acceleration_mps2: float,
    maximum_speed_age_sec: float,
    maximum_pose_age_sec: float,
    maximum_samples: int,
    failure_start_sec: float | None,
    failure_window_sec: float,
    offsets_rad: tuple[float, ...],
    rollout_config: ManeuverRolloutConfig,
) -> dict:
    arrays = read_replay_inputs(bag_path)
    pose_times, poses = read_pose_stream(bag_path)
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

    selected_indices = _select_evaluation_indices(
        arrays["relative_time_sec"],
        relevant,
        failure_start_sec,
        failure_window_sec,
        maximum_samples,
    )
    records = []
    skipped_future = 0
    for index in selected_indices:
        decision, acceleration, speed = teacher_decisions[index]
        state_count = len(
            rollout_lane_change_stop(
                speed,
                float(arrays["published_steering_at_scan_rad"][index]),
                0.0,
                rollout_config,
            )
        )
        point_clouds = future_point_clouds_for_rollout(
            current_scan_index=int(index),
            state_count=state_count,
            scan_times_ns=scan_times,
            scans_m=arrays["scans_m"],
            pose_times_ns=pose_times,
            poses_xyyaw=poses,
            config=rollout_config,
            maximum_pose_age_sec=maximum_pose_age_sec,
        )
        if point_clouds is None:
            skipped_future += 1
            continue
        candidates = [
            evaluate_candidate_against_time_indexed_points(
                point_clouds,
                speed,
                float(arrays["published_steering_at_scan_rad"][index]),
                offset,
                rollout_config,
            )
            for offset in offsets_rad
        ]
        best = select_best_candidate(candidates)
        teacher_side = int(
            decision.committed_side_sign
            if decision.committed_side_sign != 0
            else decision.proposed_side_sign
        )
        records.append(
            {
                "relative_time_sec": float(arrays["relative_time_sec"][index]),
                "speed_mps": speed,
                "front_distance_m": float(decision.front_distance_m),
                "teacher_side_sign": teacher_side,
                "teacher_forward": acceleration > 1e-6,
                "selected_side_available": _side_available(candidates, teacher_side),
                "opposite_side_available": _side_available(candidates, -teacher_side),
                "any_candidate_available": best is not None,
                "best_side_sign": 0 if best is None else best.side_sign,
                "best_steering_offset_rad": None if best is None else best.steering_offset_rad,
                "best_minimum_clearance_m": (
                    None
                    if best is None or not np.isfinite(best.minimum_clearance_m)
                    else best.minimum_clearance_m
                ),
            }
        )

    summary = summarize_maneuver_evidence(records)
    summary.update(
        {
            "source_bag": str(bag_path),
            "relevant_teacher_samples": int(np.count_nonzero(relevant)),
            "selected_for_evaluation": int(len(selected_indices)),
            "skipped_without_future_or_pose": skipped_future,
            "evaluation_window": {
                "failure_start_sec": failure_start_sec,
                "failure_window_sec": (
                    failure_window_sec if failure_start_sec is not None else None
                ),
                "maximum_samples": maximum_samples,
            },
            "records": records,
        }
    )
    return summary


def classify_future_occupancy_oracle(
    reports: dict[str, dict],
    failed_label: str,
) -> dict:
    """Classify evidence without mistaking an invalid candidate bank for signal.

    An oracle cannot support manoeuvre labels if the frozen candidate bank fails
    to represent even a majority of successful teacher frames.  In that case a
    higher opposite-side rate in the failed run is only an exploratory temporal
    correlation: track geometry, speed and the straight Stop suffix remain
    confounders.
    """
    if failed_label not in reports:
        raise ValueError("failed label is absent from reports")
    successful = {
        label: report for label, report in reports.items() if label != failed_label
    }
    if not successful:
        raise ValueError("at least one successful reference case is required")

    failed_fraction = float(
        reports[failed_label]
        .get("selected_infeasible_opposite_feasible", {})
        .get("fraction", 0.0)
    )
    success_fractions = {
        label: float(
            report.get("selected_infeasible_opposite_feasible", {}).get(
                "fraction", 0.0
            )
        )
        for label, report in successful.items()
    }
    success_candidate_coverages = {
        label: float(report.get("any_candidate_available_fraction", 0.0))
        for label, report in successful.items()
    }
    maximum_success_signal = max(success_fractions.values())
    minimum_success_coverage = min(success_candidate_coverages.values())
    temporal_signal_observed = bool(
        failed_fraction > MINIMUM_FAILURE_SIGNAL_FRACTION
        and failed_fraction
        > maximum_success_signal + MINIMUM_FAILURE_SIGNAL_DELTA
    )
    candidate_bank_represents_success = bool(
        minimum_success_coverage >= MINIMUM_SUCCESS_CANDIDATE_COVERAGE
    )
    discriminates = bool(
        temporal_signal_observed and candidate_bank_represents_success
    )
    if not candidate_bank_represents_success:
        classification = "inconclusive-candidate-bank-misses-success"
    elif discriminates:
        classification = "temporal-occupancy-supports-privileged-teacher"
    else:
        classification = "future-scan-oracle-does-not-isolate-failure"
    return {
        "future_occupancy_discriminates_failure": discriminates,
        "classification": classification,
        "temporal_signal_observed": temporal_signal_observed,
        "candidate_bank_represents_success": candidate_bank_represents_success,
        "failed_wrong_side_fraction": failed_fraction,
        "maximum_success_wrong_side_fraction": maximum_success_signal,
        "minimum_success_candidate_coverage": minimum_success_coverage,
        "acceptance_thresholds": {
            "minimum_success_candidate_coverage": (
                MINIMUM_SUCCESS_CANDIDATE_COVERAGE
            ),
            "minimum_failure_signal_fraction": MINIMUM_FAILURE_SIGNAL_FRACTION,
            "minimum_failure_signal_delta": MINIMUM_FAILURE_SIGNAL_DELTA,
        },
    }


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
    parser.add_argument("--maximum-pose-age-sec", type=float, default=0.1)
    parser.add_argument("--maximum-samples-per-case", type=int, default=120)
    parser.add_argument(
        "--steering-offsets-rad",
        type=parse_offsets,
        default=DEFAULT_OFFSETS_RAD,
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    cases = dict(args.case)
    if len(cases) != len(args.case) or args.failed_label not in cases:
        raise ValueError("case labels must be unique and include failed-label")
    positive = (
        args.failure_start_sec,
        args.failure_window_sec,
        args.maximum_speed_age_sec,
        args.maximum_pose_age_sec,
    )
    if not all(np.isfinite(value) and value > 0.0 for value in positive):
        raise ValueError("timing parameters must be finite and positive")
    if args.maximum_samples_per_case <= 0:
        raise ValueError("maximum samples must be positive")
    checkpoint = args.checkpoint.expanduser().resolve()
    if not checkpoint.is_file():
        raise FileNotFoundError(f"checkpoint not found: {checkpoint}")
    config = ManeuverRolloutConfig()
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
            maximum_pose_age_sec=args.maximum_pose_age_sec,
            maximum_samples=args.maximum_samples_per_case,
            failure_start_sec=(
                args.failure_start_sec if label == args.failed_label else None
            ),
            failure_window_sec=args.failure_window_sec,
            offsets_rad=tuple(args.steering_offsets_rad),
            rollout_config=config,
        )

    comparison = classify_future_occupancy_oracle(reports, args.failed_label)
    comparison["oracle_scope"] = {
        "future_lidar": True,
        "future_ego_pose": True,
        "counterfactual_sensor_view": False,
        "runtime_input_permitted": False,
        "label_generation_permitted": False,
    }
    result = {
        "schema_version": 1,
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": sha256_file(checkpoint),
        "failed_label": args.failed_label,
        "failure_start_sec": args.failure_start_sec,
        "steering_offsets_rad": list(args.steering_offsets_rad),
        "rollout_config": config.__dict__,
        "cases": reports,
        "comparison": comparison,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    compact = dict(result)
    compact["cases"] = {
        label: {key: value for key, value in report.items() if key != "records"}
        for label, report in reports.items()
    }
    print(json.dumps(compact, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
