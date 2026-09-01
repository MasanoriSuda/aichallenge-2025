#!/usr/bin/env python3
"""Gate one spatial-adapter shadow run without granting control authority."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
TOKEN_RE = re.compile(r"([a-zA-Z0-9_]+)=([^\s]+)")
SHADOW_PATH_RE = re.compile(
    r"tiny_lidar_spatial_shadow_ckpt_path:\s*(\S+)"
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_status_lines(log_text: str) -> list[dict]:
    intervals = []
    for raw_line in log_text.splitlines():
        line = ANSI_RE.sub("", raw_line)
        if "E2E_STATUS" not in line or "spatial_shadow=" not in line:
            continue
        tokens = dict(TOKEN_RE.findall(line))
        admitted_text, scans_text = tokens["spatial_shadow"].split("/", 1)
        probabilities = [
            float(value) for value in tokens["shadow_prob_lnr"].split(",")
        ]
        if len(probabilities) != 3:
            raise ValueError("shadow_prob_lnr must contain left,neutral,right")
        interval = {
            "scans": int(scans_text),
            "admitted": int(admitted_text),
            "skipped": int(tokens["shadow_skipped"]),
            "errors": int(tokens["shadow_errors"]),
            "stale": int(tokens["stale"]),
            "scan_hz": float(tokens["scan_hz"]),
            "avg_inference_ms": float(tokens["avg_inference_ms"]),
            "max_inference_ms": float(tokens["max_inference_ms"]),
            "inference_capacity_hz": float(tokens["inference_capacity_hz"]),
            "mean_abs_correction_rad": float(tokens["shadow_mean_abs_rad"]),
            "p95_abs_correction_rad": float(tokens["shadow_p95_abs_rad"]),
            "last_correction_rad": float(tokens["shadow_last_rad"]),
            "direction_probabilities_lnr": probabilities,
            "status": tokens["shadow_status"],
            "authority_enabled": bool(
                int(tokens.get("spatial_authority_enabled", "0"))
            ),
            "authority_applied": 0,
            "authority_scans": int(scans_text),
            "authority_clipped": int(
                tokens.get("spatial_authority_clipped", "0")
            ),
            "authority_mean_abs_correction_rad": float(
                tokens.get("spatial_authority_mean_abs_rad", "0")
            ),
            "authority_max_abs_correction_rad": float(
                tokens.get("spatial_authority_max_abs_rad", "0")
            ),
        }
        authority_applied = tokens.get("spatial_authority_applied", "0/0")
        applied_text, authority_scans_text = authority_applied.split("/", 1)
        interval["authority_applied"] = int(applied_text)
        interval["authority_scans"] = int(authority_scans_text)
        numeric_values = [
            value
            for key, value in interval.items()
            if key not in {
                "status",
                "direction_probabilities_lnr",
                "authority_enabled",
            }
        ] + probabilities
        if not np.all(np.isfinite(np.asarray(numeric_values, dtype=np.float64))):
            raise ValueError("shadow status contains non-finite metrics")
        intervals.append(interval)
    return intervals


def parse_runtime_config(log_text: str) -> dict | None:
    clean = ANSI_RE.sub("", log_text)
    matches = re.findall(
        r"SpatialShadowConfig:\s*"
        r"hidden=(\d+),projection=(\d+),use_speed=([01]),"
        r"(?:use_base_steering=([01]),)?"
        r"max_speed_mps=([0-9.]+),max_delta_rad=([0-9.]+),"
        r"speed_timeout_sec=([0-9.]+)"
        r"(?:,authority_enabled=([01]),"
        r"authority_max_delta_rad=([0-9.]+))?",
        clean,
    )
    if not matches:
        return None
    unique = set(matches)
    if len(unique) != 1:
        raise ValueError("ambiguous spatial shadow runtime configuration")
    (
        hidden,
        projection,
        use_speed,
        use_base_steering,
        max_speed,
        max_delta,
        timeout,
        authority_enabled,
        authority_max_delta,
    ) = matches[0]
    return {
        "hidden_dim": int(hidden),
        "projection_dim": int(projection),
        "use_speed": bool(int(use_speed)),
        "use_base_steering": bool(int(use_base_steering or "0")),
        "max_speed_mps": float(max_speed),
        "max_abs_delta_rad": float(max_delta),
        "speed_timeout_sec": float(timeout),
        "authority_enabled": bool(int(authority_enabled or "0")),
        "authority_max_abs_delta_rad": (
            None if not authority_max_delta else float(authority_max_delta)
        ),
    }


def load_runtime_log_text(run_dir: Path) -> tuple[str, list[str], bool]:
    """Load one run's spatial evidence without double-counting ROS output.

    ``autoware.log`` is the canonical source.  A Docker stop can, however,
    leave a later launch attempt in that file while the completed controller
    process output remains in ``ROS_LOG_DIR``.  Only when the canonical log no
    longer contains spatial status intervals do we join the per-process ROS
    logs and launch logs from the same run.  This preserves the evidence rather
    than silently treating a log-lifecycle failure as a model failure.
    """

    primary_path = run_dir / "d1" / "autoware.log"
    primary_text = primary_path.read_text(encoding="utf-8", errors="replace")
    if parse_status_lines(primary_text):
        return primary_text, [str(primary_path)], False

    ros_log_root = run_dir / "d1" / "ros" / "log"
    fallback_paths = sorted(ros_log_root.glob("python3_*.log"))
    fallback_paths.extend(sorted(ros_log_root.glob("*/launch.log")))
    fallback_texts = [primary_text]
    used_paths = [primary_path]
    for path in fallback_paths:
        text = path.read_text(encoding="utf-8", errors="replace")
        if "E2E_STATUS" in text or "SpatialShadowConfig" in text or (
            "tiny_lidar_spatial_shadow_ckpt_path" in text
        ):
            fallback_texts.append(text)
            used_paths.append(path)
    return "\n".join(fallback_texts), [str(path) for path in used_paths], True


def summarize_intervals(intervals: list[dict]) -> dict:
    if not intervals:
        raise ValueError("no spatial shadow E2E_STATUS intervals found")
    scans = sum(item["scans"] for item in intervals)
    admitted = sum(item["admitted"] for item in intervals)
    if scans <= 0:
        raise ValueError("shadow interval scan count must be positive")
    weighted_mean_abs = sum(
        item["mean_abs_correction_rad"] * item["admitted"]
        for item in intervals
    ) / max(admitted, 1)
    weighted_inference = sum(
        item["avg_inference_ms"] * item["scans"] for item in intervals
    ) / scans
    return {
        "interval_count": len(intervals),
        "scan_count": scans,
        "admitted_count": admitted,
        "skipped_count": sum(item["skipped"] for item in intervals),
        "error_count": sum(item["errors"] for item in intervals),
        "coverage_fraction": admitted / scans,
        "stale_interval_count": sum(item["stale"] != 0 for item in intervals),
        "min_scan_hz": min(item["scan_hz"] for item in intervals),
        "weighted_avg_inference_ms": weighted_inference,
        "max_inference_ms": max(item["max_inference_ms"] for item in intervals),
        "min_inference_capacity_hz": min(
            item["inference_capacity_hz"] for item in intervals
        ),
        "weighted_mean_abs_correction_rad": weighted_mean_abs,
        "max_interval_p95_abs_correction_rad": max(
            item["p95_abs_correction_rad"] for item in intervals
        ),
        "nonzero_interval_count": sum(
            item["p95_abs_correction_rad"] > 1e-6 for item in intervals
        ),
        "non_ok_interval_count": sum(item["status"] != "ok" for item in intervals),
        "authority_enabled_interval_count": sum(
            item["authority_enabled"] for item in intervals
        ),
        "authority_applied_count": sum(
            item["authority_applied"] for item in intervals
        ),
        "authority_clipped_count": sum(
            item["authority_clipped"] for item in intervals
        ),
        "authority_weighted_mean_abs_correction_rad": sum(
            item["authority_mean_abs_correction_rad"]
            * item["authority_applied"]
            for item in intervals
        ) / max(sum(item["authority_applied"] for item in intervals), 1),
        "authority_max_abs_correction_rad": max(
            item["authority_max_abs_correction_rad"] for item in intervals
        ),
    }


def build_report(args: argparse.Namespace) -> dict:
    run_dir = args.run_dir.resolve()
    result_path = run_dir / "d1-result-details.json"
    competition_path = run_dir / "e2e-competition-analysis.json"
    checkpoint = args.checkpoint_file.resolve()
    log_text, log_sources, used_ros_log_fallback = load_runtime_log_text(run_dir)
    intervals = parse_status_lines(log_text)
    summary = summarize_intervals(intervals)
    runtime_config = parse_runtime_config(log_text)
    expected_authority_enabled = args.expected_authority_enabled == "true"
    expected_use_base_steering = args.expected_use_base_steering == "true"
    path_matches = set(SHADOW_PATH_RE.findall(ANSI_RE.sub("", log_text)))
    race = json.loads(result_path.read_text(encoding="utf-8"))
    competition = json.loads(competition_path.read_text(encoding="utf-8"))
    actual_sha = sha256_file(checkpoint)
    reasons = []
    if actual_sha != args.expected_checkpoint_sha256:
        reasons.append("shadow checkpoint SHA mismatch")
    if path_matches != {args.expected_runtime_checkpoint_path}:
        reasons.append("runtime shadow checkpoint path missing or ambiguous")
    if runtime_config is None:
        reasons.append("runtime shadow configuration missing")
    else:
        if runtime_config["authority_enabled"] != expected_authority_enabled:
            reasons.append("spatial authority mode mismatch")
        if runtime_config["use_base_steering"] != expected_use_base_steering:
            reasons.append("spatial base-steering feature contract mismatch")
        if (
            args.expected_max_abs_delta_rad is not None
            and not np.isclose(
                runtime_config["max_abs_delta_rad"],
                args.expected_max_abs_delta_rad,
                rtol=0.0,
                atol=1e-9,
            )
        ):
            reasons.append("spatial residual scale contract mismatch")
    if summary["coverage_fraction"] < args.min_coverage_fraction:
        reasons.append("shadow coverage below threshold")
    if summary["error_count"] != 0 or summary["non_ok_interval_count"] != 0:
        reasons.append("shadow inference error or non-ok interval")
    if summary["stale_interval_count"] != 0:
        reasons.append("production sensor watchdog became stale")
    if summary["min_scan_hz"] < args.min_scan_hz:
        reasons.append("scan frequency below threshold")
    if summary["nonzero_interval_count"] == 0:
        reasons.append("shadow produced no material diagnostic output")
    if expected_authority_enabled:
        authority_bound = (
            None
            if runtime_config is None
            else runtime_config["authority_max_abs_delta_rad"]
        )
        if summary["authority_enabled_interval_count"] != summary[
            "interval_count"
        ]:
            reasons.append("authority was not enabled in every status interval")
        if summary["authority_applied_count"] == 0:
            reasons.append("spatial authority was never applied")
        if authority_bound is None or summary[
            "authority_max_abs_correction_rad"
        ] > authority_bound + 1e-6:
            reasons.append("applied spatial correction exceeded authority bound")
    elif summary["authority_applied_count"] != 0:
        reasons.append("shadow-only run unexpectedly applied spatial authority")
    if not race.get("finished") or race.get("lap_count", 0) < race.get(
        "required_laps", 0
    ):
        reasons.append("race did not finish required laps")
    if race.get("penalty_count") != 0:
        reasons.append("race contains penalties")
    if competition.get("status") != "pass":
        reasons.append("frozen production competition gate failed")
    return {
        "schema_version": 1,
        "status": "pass" if not reasons else "reject",
        "reasons": reasons,
        "run_dir": str(run_dir),
        "runtime_evidence": {
            "log_sources": log_sources,
            "used_ros_log_fallback": used_ros_log_fallback,
        },
        "shadow_checkpoint": {
            "artifact_path": str(checkpoint),
            "runtime_path": sorted(path_matches),
            "sha256": actual_sha,
        },
        "runtime_config": runtime_config,
        "shadow": summary,
        "race": {
            "finished": race.get("finished"),
            "lap_count": race.get("lap_count"),
            "required_laps": race.get("required_laps"),
            "laps": race.get("laps"),
            "penalty_count": race.get("penalty_count"),
        },
        "production_gate_status": competition.get("status"),
        "thresholds": {
            "min_coverage_fraction": args.min_coverage_fraction,
            "min_scan_hz": args.min_scan_hz,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--checkpoint-file", type=Path, required=True)
    parser.add_argument("--expected-checkpoint-sha256", required=True)
    parser.add_argument("--expected-runtime-checkpoint-path", required=True)
    parser.add_argument(
        "--expected-authority-enabled",
        choices=("true", "false"),
        default="false",
    )
    parser.add_argument(
        "--expected-use-base-steering",
        choices=("true", "false"),
        default="false",
    )
    parser.add_argument("--expected-max-abs-delta-rad", type=float)
    parser.add_argument("--min-coverage-fraction", type=float, default=0.99)
    parser.add_argument("--min-scan-hz", type=float, default=19.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fail-on-rejection", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_report(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return int(args.fail_on_rejection and report["status"] != "pass")


if __name__ == "__main__":
    raise SystemExit(main())
