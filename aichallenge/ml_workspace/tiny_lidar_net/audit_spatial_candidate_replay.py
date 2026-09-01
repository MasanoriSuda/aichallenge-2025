#!/usr/bin/env python3
"""Replay spatial steering candidates on one immutable E2E bag.

This is a diagnostic, not an admission gate.  It evaluates multiple frozen
candidate files against the exact same LiDAR frames and compares wheel-speed
and legacy fused-speed inputs.  Closed-loop causality is intentionally not
claimed; the report is used to distinguish an input-contract regression from
a changed learned function before another training or runtime experiment.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
import torch
from rosbags.highlevel import AnyReader

from analyze_e2e_run import nearest_values
from lib.checkpoint import load_pretrained_weights, sha256_file
from lib.spatial_adapter import FrozenTinyLidarSpatialResidual


SCAN_TOPIC = "/sensing/lidar/scan"
WHEEL_SPEED_TOPIC = "/vehicle/status/velocity_status"
FUSED_SPEED_TOPIC = "/localization/kinematic_state"


def clean_and_resize_ranges(
    ranges: np.ndarray,
    input_dim: int = 750,
    max_range_m: float = 30.0,
) -> np.ndarray:
    """Apply the production TinyLidarNet scan contract in metres."""
    values = np.asarray(ranges, dtype=np.float32).copy()
    if values.ndim != 1 or values.size == 0:
        raise ValueError("LiDAR ranges must be a non-empty vector")
    values[np.isnan(values)] = 0.0
    values[np.isinf(values)] = max_range_m
    values = np.clip(values, 0.0, max_range_m)
    if len(values) > input_dim:
        indices = np.linspace(0, len(values) - 1, input_dim, dtype=int)
        values = values[indices]
    elif len(values) < input_dim:
        values = np.pad(values, (0, input_dim - len(values)), "constant")
    return values.astype(np.float32, copy=False)


def read_replay_inputs(bag_path: Path) -> dict:
    """Read ordered scans and the two speed streams used by this audit."""
    scan_times = []
    scans = []
    wheel_times = []
    wheel_speeds = []
    fused_times = []
    fused_speeds = []
    required = {SCAN_TOPIC, WHEEL_SPEED_TOPIC, FUSED_SPEED_TOPIC}
    with AnyReader([bag_path]) as reader:
        available = {connection.topic for connection in reader.connections}
        missing = sorted(required - available)
        if missing:
            raise ValueError(f"replay topics missing from bag: {missing}")
        connections = [
            connection
            for connection in reader.connections
            if connection.topic in required
        ]
        for connection, timestamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            if connection.topic == SCAN_TOPIC:
                scan_times.append(timestamp)
                scans.append(clean_and_resize_ranges(message.ranges))
            elif connection.topic == WHEEL_SPEED_TOPIC:
                wheel_times.append(timestamp)
                wheel_speeds.append(abs(float(message.longitudinal_velocity)))
            else:
                fused_times.append(timestamp)
                fused_speeds.append(abs(float(message.twist.twist.linear.x)))
    arrays = {
        "scan_times_ns": np.asarray(scan_times, dtype=np.int64),
        "scans_m": np.asarray(scans, dtype=np.float32),
        "wheel_times_ns": np.asarray(wheel_times, dtype=np.int64),
        "wheel_speeds_mps": np.asarray(wheel_speeds, dtype=np.float32),
        "fused_times_ns": np.asarray(fused_times, dtype=np.int64),
        "fused_speeds_mps": np.asarray(fused_speeds, dtype=np.float32),
    }
    if len(arrays["scan_times_ns"]) == 0:
        raise ValueError("replay bag contains no LiDAR frames")
    for prefix in ("wheel", "fused"):
        times = arrays[f"{prefix}_times_ns"]
        values = arrays[f"{prefix}_speeds_mps"]
        if len(times) == 0 or len(times) != len(values):
            raise ValueError(f"{prefix} speed stream is empty or misaligned")
        if np.any(np.diff(times) <= 0) or not np.all(np.isfinite(values)):
            raise ValueError(f"{prefix} speed stream violates replay contract")
    scan_times_array = arrays["scan_times_ns"]
    arrays["wheel_at_scan_mps"] = nearest_values(
        scan_times_array,
        arrays["wheel_times_ns"],
        arrays["wheel_speeds_mps"],
    ).astype(np.float32)
    arrays["fused_at_scan_mps"] = nearest_values(
        scan_times_array,
        arrays["fused_times_ns"],
        arrays["fused_speeds_mps"],
    ).astype(np.float32)
    arrays["relative_time_sec"] = (
        scan_times_array - arrays["wheel_times_ns"][0]
    ).astype(np.float64) / 1e9
    return arrays


def load_candidate(
    path: Path,
    use_base_steering: bool = False,
    max_abs_delta_rad: float = 1.2,
) -> FrozenTinyLidarSpatialResidual:
    model = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=128,
        max_scan_range_m=30.0,
        max_abs_delta_rad=max_abs_delta_rad,
        use_speed=True,
        use_base_steering=use_base_steering,
        max_speed_mps=12.0,
        spatial_normalization="fixed_train_statistics",
        projection_dim=128,
        projection_seed=2026,
        head_architecture="signed_mixture",
    )
    load_pretrained_weights(model, path)
    model.eval()
    return model


def predict(
    model: FrozenTinyLidarSpatialResidual,
    scans_m: np.ndarray,
    speeds_mps: np.ndarray,
    batch_size: int,
) -> tuple[np.ndarray, np.ndarray]:
    residuals = []
    directions = []
    with torch.no_grad():
        for start in range(0, len(scans_m), batch_size):
            stop = min(start + batch_size, len(scans_m))
            residual, _, _, probabilities = model.forward_components(
                torch.from_numpy(scans_m[start:stop]),
                torch.from_numpy(speeds_mps[start:stop]),
            )
            residuals.append(residual.numpy())
            directions.append(probabilities.numpy())
    return np.concatenate(residuals), np.concatenate(directions)


def longest_true_samples(mask: np.ndarray) -> int:
    longest = current = 0
    for value in np.asarray(mask, dtype=bool):
        current = current + 1 if value else 0
        longest = max(longest, current)
    return longest


def summarize_prediction(
    residual: np.ndarray,
    probabilities: np.ndarray,
    mask: np.ndarray,
    authority_bound_rad: float,
    near_bound_fraction: float = 0.95,
) -> dict:
    selected = residual[mask]
    selected_probabilities = probabilities[mask]
    if len(selected) == 0:
        return {"samples": 0}
    if not 0.0 < near_bound_fraction <= 1.0:
        raise ValueError("near_bound_fraction must be in (0, 1]")
    absolute = np.abs(selected)
    saturated = absolute > authority_bound_rad
    near_bound = absolute >= authority_bound_rad * near_bound_fraction
    return {
        "samples": int(len(selected)),
        "mean_rad": float(np.mean(selected)),
        "mean_abs_rad": float(np.mean(np.abs(selected))),
        "p95_abs_rad": float(np.quantile(np.abs(selected), 0.95)),
        "max_abs_rad": float(np.max(np.abs(selected))),
        "right_fraction": float(np.mean(selected > 0.02)),
        "left_fraction": float(np.mean(selected < -0.02)),
        "neutral_fraction": float(np.mean(np.abs(selected) <= 0.02)),
        "authority_saturation_fraction": float(np.mean(saturated)),
        "longest_authority_saturation_samples": longest_true_samples(saturated),
        "near_authority_bound_threshold_rad": float(
            authority_bound_rad * near_bound_fraction
        ),
        "near_authority_bound_fraction": float(np.mean(near_bound)),
        "longest_near_authority_bound_samples": longest_true_samples(near_bound),
        "mean_direction_probability_lnr": [
            float(value) for value in np.mean(selected_probabilities, axis=0)
        ],
    }


def summarize_teacher_alignment(
    predicted_residual: np.ndarray,
    target_residual: np.ndarray,
    mask: np.ndarray,
    material_delta_rad: float,
) -> dict:
    """Measure one candidate against the admitted teacher on one window."""
    predicted = np.asarray(predicted_residual, dtype=np.float32)[mask]
    target = np.asarray(target_residual, dtype=np.float32)[mask]
    if len(predicted) == 0:
        return {"samples": 0}
    material = np.abs(target) >= material_delta_rad
    material_count = int(np.count_nonzero(material))
    result = {
        "samples": int(len(predicted)),
        "mae_rad": float(np.mean(np.abs(predicted - target))),
        "target_material_fraction": float(np.mean(material)),
        "target_mean_rad": float(np.mean(target)),
        "predicted_mean_rad": float(np.mean(predicted)),
    }
    if material_count:
        predicted_material = predicted[material]
        target_material = target[material]
        result.update(
            {
                "material_samples": material_count,
                "material_mae_rad": float(
                    np.mean(np.abs(predicted_material - target_material))
                ),
                "material_sign_accuracy": float(
                    np.mean(np.sign(predicted_material) == np.sign(target_material))
                ),
                "material_opposite_sign_fraction": float(
                    np.mean(
                        (np.abs(predicted_material) >= material_delta_rad)
                        & (
                            np.sign(predicted_material)
                            != np.sign(target_material)
                        )
                    )
                ),
            }
        )
    else:
        result.update(
            {
                "material_samples": 0,
                "material_mae_rad": None,
                "material_sign_accuracy": None,
                "material_opposite_sign_fraction": None,
            }
        )
    return result


def admitted_teacher_targets(
    checkpoint: Path,
    scans_m: np.ndarray,
) -> tuple[np.ndarray, list[str]]:
    """Return precontact-teacher residual targets for the exact replay scans."""
    from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
    from tiny_lidar_net_controller.tiny_lidar_net_controller_core import (
        TinyLidarNetCore,
    )

    core = TinyLidarNetCore(
        input_dim=750,
        output_dim=2,
        architecture="normal",
        ckpt_path=str(checkpoint),
        acceleration=0.6,
        control_mode="precontact_teacher",
        max_range=30.0,
        gap_teacher_config=GapTeacherConfig(),
    )
    targets = []
    reasons = []
    for scan in scans_m:
        core.process(scan)
        decision = core.last_gap_teacher_decision
        if decision is None:
            raise RuntimeError("precontact teacher produced no replay decision")
        targets.append(decision.steering_rad - decision.base_steering_rad)
        reasons.append(decision.reason)
    return np.asarray(targets, dtype=np.float32), reasons


def parse_candidate(value: str) -> tuple[str, Path]:
    name, separator, path = value.partition("=")
    if not separator or not name or not path:
        raise argparse.ArgumentTypeError("candidate must be NAME=PATH")
    return name, Path(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--candidate", action="append", type=parse_candidate, required=True
    )
    parser.add_argument("--focus-start-sec", type=float, required=True)
    parser.add_argument("--focus-window-sec", type=float, default=20.0)
    parser.add_argument("--authority-bound-rad", type=float, default=0.12)
    parser.add_argument(
        "--candidate-max-abs-delta-rad",
        type=float,
        default=1.2,
        help=(
            "Residual scale used when reconstructing every candidate. This "
            "must match training; it is intentionally separate from the "
            "runtime authority bound."
        ),
    )
    parser.add_argument(
        "--candidate-use-base-steering",
        action="store_true",
        help="Load every candidate with embedded-base steering conditioning.",
    )
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument(
        "--teacher-base-checkpoint",
        type=Path,
        help=(
            "Optional admitted frozen base used to generate precontact-teacher "
            "targets on the exact replay frames."
        ),
    )
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.focus_start_sec < 0.0 or args.focus_window_sec <= 0.0:
        raise ValueError("invalid replay focus window")
    if (
        args.authority_bound_rad <= 0.0
        or args.candidate_max_abs_delta_rad <= 0.0
        or args.material_delta_rad <= 0.0
        or args.batch_size <= 0
    ):
        raise ValueError("invalid replay authority or batch configuration")

    arrays = read_replay_inputs(args.bag.resolve())
    relative = arrays["relative_time_sec"]
    windows = {
        "all": np.ones(len(relative), dtype=bool),
        "pre_focus_20s": (
            (relative >= max(0.0, args.focus_start_sec - 20.0))
            & (relative < args.focus_start_sec)
        ),
        "focus": (
            (relative >= args.focus_start_sec)
            & (relative < args.focus_start_sec + args.focus_window_sec)
        ),
        "post_focus": relative >= args.focus_start_sec,
    }
    predictions = {}
    artifacts = {}
    teacher_target = None
    teacher_reasons = None
    teacher_artifact = None
    if args.teacher_base_checkpoint is not None:
        teacher_path = args.teacher_base_checkpoint.resolve()
        teacher_target, teacher_reasons = admitted_teacher_targets(
            teacher_path, arrays["scans_m"]
        )
        teacher_artifact = {
            "path": str(teacher_path),
            "sha256": sha256_file(teacher_path),
            "material_delta_rad": args.material_delta_rad,
            "windows": {
                window: {
                    "samples": int(np.count_nonzero(mask)),
                    "reason_counts": dict(
                        sorted(
                            Counter(
                                reason
                                for reason, selected in zip(teacher_reasons, mask)
                                if selected
                            ).items()
                        )
                    ),
                }
                for window, mask in windows.items()
            },
        }
    for name, path in args.candidate:
        resolved = path.resolve()
        if name in artifacts:
            raise ValueError(f"duplicate candidate name: {name}")
        model = load_candidate(
            resolved,
            args.candidate_use_base_steering,
            args.candidate_max_abs_delta_rad,
        )
        artifacts[name] = {
            "path": str(resolved),
            "sha256": sha256_file(resolved),
            "max_abs_delta_rad": args.candidate_max_abs_delta_rad,
            "use_base_steering": args.candidate_use_base_steering,
        }
        predictions[name] = {}
        for speed_name in ("wheel", "fused"):
            residual, probabilities = predict(
                model,
                arrays["scans_m"],
                arrays[f"{speed_name}_at_scan_mps"],
                args.batch_size,
            )
            predictions[name][speed_name] = {
                window: summarize_prediction(
                    residual,
                    probabilities,
                    mask,
                    args.authority_bound_rad,
                )
                for window, mask in windows.items()
            }
            if teacher_target is not None:
                predictions[name][speed_name]["teacher_alignment"] = {
                    window: summarize_teacher_alignment(
                        residual,
                        teacher_target,
                        mask,
                        args.material_delta_rad,
                    )
                    for window, mask in windows.items()
                }

    speed_delta = (
        arrays["fused_at_scan_mps"] - arrays["wheel_at_scan_mps"]
    ).astype(np.float64)
    report = {
        "schema_version": 1,
        "source_bag": str(args.bag.resolve()),
        "scan_samples": int(len(relative)),
        "duration_sec": float(relative[-1] - relative[0]),
        "focus": {
            "start_sec": args.focus_start_sec,
            "window_sec": args.focus_window_sec,
        },
        "speed_contract_comparison": {
            "mean_signed_fused_minus_wheel_mps": float(np.mean(speed_delta)),
            "mae_mps": float(np.mean(np.abs(speed_delta))),
            "p95_abs_mps": float(np.quantile(np.abs(speed_delta), 0.95)),
            "max_abs_mps": float(np.max(np.abs(speed_delta))),
        },
        "candidate_artifacts": artifacts,
        "teacher_contract": teacher_artifact,
        "predictions": predictions,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
