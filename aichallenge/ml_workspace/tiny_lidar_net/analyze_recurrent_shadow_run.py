#!/usr/bin/env python3
"""Gate one recurrent-adapter shadow run without granting it authority."""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

import numpy as np


ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")
TOKEN_RE = re.compile(r"([a-zA-Z0-9_]+)=([^\s]+)")
CHECKPOINT_PATH_RE = re.compile(
    r"tiny_lidar_recurrent_shadow_ckpt_path:\s*(\S+)"
)
CONFIG_RE = re.compile(
    r"RecurrentShadowConfig:\s*"
    r"hidden=(\d+),projection=(\d+),use_speed=([01]),"
    r"speed_embedding=(\d+),max_speed_mps=([0-9.]+),"
    r"max_correction_rad=([0-9.]+),deadband_rad=([0-9.]+)"
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
        if "E2E_STATUS" not in line or "recurrent_shadow=" not in line:
            continue
        tokens = dict(TOKEN_RE.findall(line))
        admitted_text, scans_text = tokens["recurrent_shadow"].split("/", 1)
        interval = {
            "scans": int(scans_text),
            "admitted": int(admitted_text),
            "skipped": int(tokens["recurrent_skipped"]),
            "errors": int(tokens["recurrent_errors"]),
            "stale": int(tokens["stale"]),
            "scan_hz": float(tokens["scan_hz"]),
            "avg_inference_ms": float(tokens["avg_inference_ms"]),
            "max_inference_ms": float(tokens["max_inference_ms"]),
            "inference_capacity_hz": float(tokens["inference_capacity_hz"]),
            "mean_abs_correction_rad": float(
                tokens["recurrent_mean_abs_rad"]
            ),
            "p95_abs_correction_rad": float(
                tokens["recurrent_p95_abs_rad"]
            ),
            "last_correction_rad": float(tokens["recurrent_last_rad"]),
            "last_raw_correction_rad": float(
                tokens["recurrent_raw_last_rad"]
            ),
            "hidden_norm": float(tokens["recurrent_hidden_norm"]),
            "reset_count": int(tokens["recurrent_resets"]),
            "status": tokens["recurrent_status"],
            "spatial_authority_enabled": bool(
                int(tokens.get("spatial_authority_enabled", "0"))
            ),
        }
        numeric = [
            value
            for key, value in interval.items()
            if key not in {"status", "spatial_authority_enabled"}
        ]
        if not np.all(np.isfinite(np.asarray(numeric, dtype=np.float64))):
            raise ValueError("recurrent shadow status has non-finite metrics")
        intervals.append(interval)
    return intervals


def parse_runtime_config(log_text: str) -> dict | None:
    matches = CONFIG_RE.findall(ANSI_RE.sub("", log_text))
    if not matches:
        return None
    unique = set(matches)
    if len(unique) != 1:
        raise ValueError("ambiguous recurrent shadow runtime configuration")
    hidden, projection, use_speed, speed_embedding, max_speed, cap, deadband = (
        matches[0]
    )
    return {
        "hidden_dim": int(hidden),
        "projection_dim": int(projection),
        "use_speed": bool(int(use_speed)),
        "speed_embedding_dim": int(speed_embedding),
        "max_speed_mps": float(max_speed),
        "max_abs_correction_rad": float(cap),
        "correction_deadband_rad": float(deadband),
    }


def load_runtime_log_text(run_dir: Path) -> tuple[str, list[str], bool]:
    primary_path = run_dir / "d1" / "autoware.log"
    primary_text = primary_path.read_text(encoding="utf-8", errors="replace")
    if parse_status_lines(primary_text):
        return primary_text, [str(primary_path)], False

    ros_log_root = run_dir / "d1" / "ros" / "log"
    fallback_paths = sorted(ros_log_root.glob("python3_*.log"))
    fallback_paths.extend(sorted(ros_log_root.glob("*/launch.log")))
    texts = [primary_text]
    used_paths = [primary_path]
    for path in fallback_paths:
        text = path.read_text(encoding="utf-8", errors="replace")
        if (
            "recurrent_shadow=" in text
            or "RecurrentShadowConfig" in text
            or "tiny_lidar_recurrent_shadow_ckpt_path" in text
        ):
            texts.append(text)
            used_paths.append(path)
    return "\n".join(texts), [str(path) for path in used_paths], True


def summarize(intervals: list[dict]) -> dict:
    if not intervals:
        raise ValueError("no recurrent shadow E2E_STATUS intervals found")
    scans = sum(item["scans"] for item in intervals)
    admitted = sum(item["admitted"] for item in intervals)
    if scans <= 0:
        raise ValueError("recurrent shadow interval scan count must be positive")
    return {
        "interval_count": len(intervals),
        "scan_count": scans,
        "admitted_count": admitted,
        "skipped_count": sum(item["skipped"] for item in intervals),
        "error_count": sum(item["errors"] for item in intervals),
        "coverage_fraction": admitted / scans,
        "stale_interval_count": sum(item["stale"] != 0 for item in intervals),
        "min_scan_hz": min(item["scan_hz"] for item in intervals),
        "weighted_avg_inference_ms": sum(
            item["avg_inference_ms"] * item["scans"] for item in intervals
        ) / scans,
        "max_inference_ms": max(item["max_inference_ms"] for item in intervals),
        "min_inference_capacity_hz": min(
            item["inference_capacity_hz"] for item in intervals
        ),
        "weighted_mean_abs_correction_rad": sum(
            item["mean_abs_correction_rad"] * item["admitted"]
            for item in intervals
        ) / max(admitted, 1),
        "max_interval_p95_abs_correction_rad": max(
            item["p95_abs_correction_rad"] for item in intervals
        ),
        "nonzero_interval_count": sum(
            item["p95_abs_correction_rad"] > 1e-6 for item in intervals
        ),
        "non_ok_interval_count": sum(item["status"] != "ok" for item in intervals),
        "max_reset_count": max(item["reset_count"] for item in intervals),
        "max_hidden_norm": max(item["hidden_norm"] for item in intervals),
        "spatial_authority_enabled_interval_count": sum(
            item["spatial_authority_enabled"] for item in intervals
        ),
    }


def build_report(args: argparse.Namespace) -> dict:
    run_dir = args.run_dir.resolve()
    checkpoint = args.checkpoint_file.resolve()
    log_text, log_sources, used_fallback = load_runtime_log_text(run_dir)
    intervals = parse_status_lines(log_text)
    shadow = summarize(intervals)
    runtime_config = parse_runtime_config(log_text)
    checkpoint_paths = set(
        CHECKPOINT_PATH_RE.findall(ANSI_RE.sub("", log_text))
    )
    race = json.loads(
        (run_dir / "d1-result-details.json").read_text(encoding="utf-8")
    )
    competition = json.loads(
        (run_dir / "e2e-competition-analysis.json").read_text(encoding="utf-8")
    )
    actual_sha = sha256_file(checkpoint)
    reasons = []
    if actual_sha != args.expected_checkpoint_sha256:
        reasons.append("recurrent checkpoint SHA mismatch")
    if checkpoint_paths != {args.expected_runtime_checkpoint_path}:
        reasons.append("runtime recurrent checkpoint path missing or ambiguous")
    if runtime_config is None:
        reasons.append("runtime recurrent configuration missing")
    else:
        expected = {
            "hidden_dim": args.expected_hidden_dim,
            "projection_dim": args.expected_projection_dim,
            "use_speed": args.expected_use_speed == "true",
            "correction_deadband_rad": args.expected_deadband_rad,
        }
        for key, value in expected.items():
            actual = runtime_config[key]
            if isinstance(value, float):
                matches = np.isclose(actual, value, rtol=0.0, atol=1e-9)
            else:
                matches = actual == value
            if not matches:
                reasons.append(f"runtime recurrent {key} contract mismatch")
    if shadow["coverage_fraction"] < args.min_coverage_fraction:
        reasons.append("recurrent shadow coverage below threshold")
    if shadow["error_count"] != 0 or shadow["non_ok_interval_count"] != 0:
        reasons.append("recurrent shadow inference error or non-ok interval")
    if shadow["stale_interval_count"] != 0:
        reasons.append("production sensor watchdog became stale")
    if shadow["min_scan_hz"] < args.min_scan_hz:
        reasons.append("scan frequency below threshold")
    if shadow["nonzero_interval_count"] == 0:
        reasons.append("recurrent shadow produced no material diagnostic output")
    if shadow["max_reset_count"] > args.max_reset_count:
        reasons.append("recurrent hidden state reset count exceeded threshold")
    if shadow["spatial_authority_enabled_interval_count"] != shadow[
        "interval_count"
    ]:
        reasons.append("frozen spatial production authority was not preserved")
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
            "used_ros_log_fallback": used_fallback,
        },
        "checkpoint": {
            "artifact_path": str(checkpoint),
            "runtime_path": sorted(checkpoint_paths),
            "sha256": actual_sha,
        },
        "runtime_config": runtime_config,
        "shadow": shadow,
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
            "max_reset_count": args.max_reset_count,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--checkpoint-file", type=Path, required=True)
    parser.add_argument("--expected-checkpoint-sha256", required=True)
    parser.add_argument("--expected-runtime-checkpoint-path", required=True)
    parser.add_argument("--expected-hidden-dim", type=int, default=64)
    parser.add_argument("--expected-projection-dim", type=int, default=128)
    parser.add_argument(
        "--expected-use-speed", choices=("true", "false"), default="false"
    )
    parser.add_argument("--expected-deadband-rad", type=float, default=0.02)
    parser.add_argument("--min-coverage-fraction", type=float, default=0.99)
    parser.add_argument("--min-scan-hz", type=float, default=19.0)
    parser.add_argument("--max-reset-count", type=int, default=0)
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
