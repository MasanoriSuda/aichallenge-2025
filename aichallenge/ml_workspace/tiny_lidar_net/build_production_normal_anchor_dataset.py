#!/usr/bin/env python3
"""Build zero-residual anchors from admitted frozen-production runs.

Unlike ``build_normal_anchor_recurrent_dataset.py``, this builder does not
inherit states from a teacher-labelled dataset.  Every sequence comes directly
from a completed production run whose controller identity, race result and
motion admission are independently proven.
"""

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from analyze_e2e_competition import load_json, sha256_file
from build_recurrent_dataset import (
    DEFAULT_SPEED_MESSAGE_TYPE,
    DEFAULT_SPEED_TOPIC,
    longest_true_run,
    read_longitudinal_speed,
)
from extract_data_from_bag import synchronize_data
from lib.normal_anchor import (
    NORMAL_ANCHOR_LABEL_SOURCE,
    NORMAL_ANCHOR_SCHEMA_VERSION,
)
from relabel_gap_teacher_bag import read_scans


DEFAULT_SCAN_TOPIC = "/sensing/lidar/scan"
DEFAULT_SCAN_MESSAGE_TYPE = "sensor_msgs/msg/LaserScan"


def parse_run_spec(value: str) -> tuple[str, Path]:
    try:
        split, path_text = value.split(":", 1)
    except ValueError:
        raise ValueError("run must use SPLIT:RUN_DIR syntax") from None
    if split not in {"train", "val"}:
        raise ValueError(f"unsupported run split: {split!r}")
    if not path_text:
        raise ValueError("run directory must not be empty")
    return split, Path(path_text).expanduser().resolve()


def _require_sha(path: Path, expected: Any, label: str) -> str:
    if not path.is_file():
        raise FileNotFoundError(f"{label} missing: {path}")
    actual = sha256_file(path)
    if expected != actual:
        raise ValueError(f"{label} hash mismatch")
    return actual


def admit_production_run(
    run_dir: Path,
    split: str,
    domain: int,
    expected_control_mode: str,
    expected_checkpoint_sha256: str,
) -> dict[str, Any]:
    """Validate one run before any sample can enter the normal corpus."""
    run_dir = run_dir.expanduser().resolve()
    if split not in {"train", "val"}:
        raise ValueError(f"unsupported split: {split!r}")
    if domain <= 0:
        raise ValueError("domain must be positive")
    report_path = run_dir / "e2e-competition-analysis.json"
    report = load_json(report_path)
    if report.get("status") != "pass":
        raise ValueError(f"production run was not admitted: {run_dir}")
    # Reports can be generated at the host path or through the container's
    # /output bind mount.  The immutable run directory name plus local artifact
    # hashes is the portable identity; comparing absolute mount prefixes would
    # reject the same physical run across those two namespaces.
    reported_run = Path(str(report.get("run_dir", "")))
    if not reported_run.is_absolute() or reported_run.name != run_dir.name:
        raise ValueError("competition report run identity mismatch")

    expected_runtime = report.get("expected_runtime")
    checkpoint_artifact = report.get("checkpoint_artifact")
    global_artifacts = report.get("artifacts")
    if not isinstance(expected_runtime, dict) or not isinstance(
        checkpoint_artifact, dict
    ) or not isinstance(global_artifacts, dict):
        raise ValueError("competition report lacks frozen runtime proof")
    expected_hash = expected_checkpoint_sha256.lower()
    if expected_runtime.get("control_mode") != expected_control_mode:
        raise ValueError("competition report control mode mismatch")
    if expected_runtime.get("checkpoint_sha256") != expected_hash:
        raise ValueError("competition report checkpoint expectation mismatch")
    if checkpoint_artifact.get("sha256") != expected_hash:
        raise ValueError("competition report checkpoint artifact mismatch")

    domains = report.get("domains")
    matching = (
        [
            item
            for item in domains
            if isinstance(item, dict) and item.get("domain") == domain
        ]
        if isinstance(domains, list)
        else []
    )
    if len(matching) != 1 or matching[0].get("status") != "pass":
        raise ValueError("requested production domain was not admitted")
    domain_report = matching[0]
    runtime = domain_report.get("runtime")
    race = domain_report.get("race")
    motion = domain_report.get("motion")
    artifacts = domain_report.get("artifacts")
    if not all(
        isinstance(item, dict) for item in (runtime, race, motion, artifacts)
    ):
        raise ValueError("production domain evidence is incomplete")
    if (
        runtime.get("control_mode") != expected_control_mode
        or runtime.get("conflicts")
    ):
        raise ValueError("runtime controller identity is not unique")
    lap_count = race.get("lap_count")
    required_laps = race.get("required_laps")
    if (
        not race.get("finished")
        or not isinstance(lap_count, int)
        or not isinstance(required_laps, int)
        or lap_count < required_laps
    ):
        raise ValueError("production run did not finish required laps")
    if race.get("penalty_count") != 0:
        raise ValueError("penalized run cannot be a production-normal anchor")
    for key in ("longest_low_speed_sec", "longest_positive_accel_stall_sec"):
        value = motion.get(key)
        if (
            not isinstance(value, (int, float))
            or not np.isfinite(value)
            or value > 0.0
        ):
            raise ValueError(f"production-normal motion violation: {key}")

    summary_path = run_dir / "result-summary.json"
    detail_path = run_dir / f"d{domain}-result-details.json"
    motion_path = run_dir / f"d{domain}" / "e2e-run-analysis.json"
    _require_sha(
        summary_path,
        global_artifacts.get("result_summary_sha256"),
        "result summary",
    )
    _require_sha(detail_path, artifacts.get("result_detail_sha256"), "result detail")
    _require_sha(motion_path, artifacts.get("motion_sha256"), "motion analysis")
    motion_document = load_json(motion_path)
    admission = motion_document.get("admission")
    if not isinstance(admission, dict) or admission.get("result") != "pass":
        raise ValueError("motion analysis admission failed")

    bag = run_dir / f"d{domain}" / "rosbag2_autoware"
    if not (bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"finalized production bag missing: {bag}")
    return {
        "split": split,
        "run_dir": run_dir,
        "run_id": run_dir.name,
        "domain": domain,
        "bag": bag.resolve(),
        "report_path": report_path,
        "report_sha256": sha256_file(report_path),
        "result_summary_sha256": sha256_file(summary_path),
        "result_detail_sha256": sha256_file(detail_path),
        "motion_analysis_sha256": sha256_file(motion_path),
        "checkpoint_sha256": expected_hash,
        "control_mode": expected_control_mode,
        "race": race,
        "motion": motion,
    }


def production_normal_sequence_id(
    admitted: dict[str, Any],
    scan_topic: str,
    speed_topic: str,
    max_sync_delta_sec: float,
) -> str:
    contract = (
        f"schema={NORMAL_ANCHOR_SCHEMA_VERSION}|run={admitted['run_dir']}|"
        f"domain={admitted['domain']}|checkpoint={admitted['checkpoint_sha256']}|"
        f"label={NORMAL_ANCHOR_LABEL_SOURCE}|scan={scan_topic}|speed={speed_topic}|"
        f"max_delta={max_sync_delta_sec:.9f}|scan_unit=m"
    )
    digest = hashlib.sha256(contract.encode("utf-8")).hexdigest()[:16]
    return f"{admitted['run_id']}-d{admitted['domain']}-production-normal-{digest}"


def build_sequence(
    admitted: dict[str, Any],
    output_root: Path,
    scan_topic: str,
    speed_topic: str,
    speed_message_type: str,
    max_scan_range_m: float,
    max_speed_sync_delta_sec: float,
    minimum_contiguous_samples: int,
) -> dict[str, Any]:
    scan_times, scans = read_scans(admitted["bag"], scan_topic, max_scan_range_m)
    speed_times, speeds = read_longitudinal_speed(
        admitted["bag"], speed_topic, speed_message_type
    )
    matched_indices, deltas_ns = synchronize_data(scan_times, speed_times)
    maximum_delta_ns = int(round(max_speed_sync_delta_sec * 1e9))
    accepted = deltas_ns <= maximum_delta_ns
    start, stop = longest_true_run(accepted)
    if stop - start < minimum_contiguous_samples:
        raise ValueError(
            f"production-normal synchronized interval too short: {stop - start}"
        )
    source_slice = slice(start, stop)
    speed_indices = matched_indices[source_slice]
    accepted_deltas_sec = deltas_ns[source_slice].astype(np.float64) / 1e9
    sequence_id = production_normal_sequence_id(
        admitted, scan_topic, speed_topic, max_speed_sync_delta_sec
    )
    output_dir = output_root / admitted["split"] / sequence_id
    if output_dir.exists():
        raise FileExistsError(f"production-normal sequence exists: {output_dir}")
    output_dir.mkdir(parents=True)
    arrays = {
        "scans.npy": scans[source_slice].astype(np.float32, copy=False),
        "speeds.npy": speeds[speed_indices].astype(np.float32, copy=False),
        "scan_timestamps_ns.npy": scan_times[source_slice],
        "speed_timestamps_ns.npy": speed_times[speed_indices],
        "speed_sync_deltas_sec.npy": accepted_deltas_sec,
    }
    for filename, values in arrays.items():
        np.save(output_dir / filename, values)
    metadata = {
        "schema_version": NORMAL_ANCHOR_SCHEMA_VERSION,
        "sequence_id": sequence_id,
        "split": admitted["split"],
        "label_source": NORMAL_ANCHOR_LABEL_SOURCE,
        "target_definition": "frozen_base_steering_correction_equals_zero",
        "stored_source_control_used_as_label": False,
        "scan_unit": "m",
        "scan_shape": [int(scans.shape[1])],
        "max_scan_range_m": max_scan_range_m,
        "max_speed_sync_delta_sec": max_speed_sync_delta_sec,
        "source": {
            "run_dir": str(admitted["run_dir"]),
            "run_id": admitted["run_id"],
            "domain": admitted["domain"],
            "bag": str(admitted["bag"]),
            "control_mode": admitted["control_mode"],
            "checkpoint_sha256": admitted["checkpoint_sha256"],
            "competition_analysis": str(admitted["report_path"]),
            "competition_analysis_sha256": admitted["report_sha256"],
            "result_summary_sha256": admitted["result_summary_sha256"],
            "result_detail_sha256": admitted["result_detail_sha256"],
            "motion_analysis_sha256": admitted["motion_analysis_sha256"],
        },
        "race": admitted["race"],
        "motion": admitted["motion"],
        "topics": {"scan": scan_topic, "speed": speed_topic},
        "message_types": {
            "scan": DEFAULT_SCAN_MESSAGE_TYPE,
            "speed": speed_message_type,
        },
        "counts": {
            "source_scan_samples": len(scan_times),
            "accepted_samples": stop - start,
            "rejected_before_interval": start,
            "rejected_after_interval": len(scan_times) - stop,
            "rejected_sync_total": int(np.count_nonzero(~accepted)),
            "raw_speed_samples": len(speed_times),
        },
        "source_interval": {"start_index": start, "stop_index": stop},
        "speed_sync_delta_sec": {
            "mean": float(np.mean(accepted_deltas_sec)),
            "p95": float(np.percentile(accepted_deltas_sec, 95)),
            "max": float(np.max(accepted_deltas_sec)),
        },
    }
    (output_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        action="append",
        required=True,
        help="Admitted source as train:/absolute/run or val:/absolute/run",
    )
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--domain", type=int, default=1)
    parser.add_argument("--expected-control-mode", default="fixed_lidar_brake")
    parser.add_argument("--expected-checkpoint-sha256", required=True)
    parser.add_argument("--scan-topic", default=DEFAULT_SCAN_TOPIC)
    parser.add_argument("--speed-topic", default=DEFAULT_SPEED_TOPIC)
    parser.add_argument("--speed-message-type", default=DEFAULT_SPEED_MESSAGE_TYPE)
    parser.add_argument("--max-scan-range-m", type=float, default=30.0)
    parser.add_argument("--max-speed-sync-delta-sec", type=float, default=0.05)
    parser.add_argument("--minimum-contiguous-samples", type=int, default=64)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_root = args.output_root.expanduser().resolve()
    if output_root.exists():
        raise FileExistsError(f"use a new immutable output root: {output_root}")
    if (
        args.domain <= 0
        or not np.isfinite(args.max_scan_range_m)
        or args.max_scan_range_m <= 0.0
        or not np.isfinite(args.max_speed_sync_delta_sec)
        or args.max_speed_sync_delta_sec <= 0.0
        or args.minimum_contiguous_samples <= 1
    ):
        raise ValueError("invalid production-normal build configuration")
    specifications = [parse_run_spec(value) for value in args.run]
    if {split for split, _ in specifications} != {"train", "val"}:
        raise ValueError("production-normal corpus requires train and val runs")
    run_paths = [run for _, run in specifications]
    if len(set(run_paths)) != len(run_paths):
        raise ValueError("a physical run may appear in only one split")

    admitted_runs = [
        admit_production_run(
            run,
            split,
            args.domain,
            args.expected_control_mode,
            args.expected_checkpoint_sha256,
        )
        for split, run in specifications
    ]
    bags = [item["bag"] for item in admitted_runs]
    if len(set(bags)) != len(bags):
        raise ValueError("a physical bag may appear in only one split")
    results = [
        build_sequence(
            admitted,
            output_root,
            args.scan_topic,
            args.speed_topic,
            args.speed_message_type,
            args.max_scan_range_m,
            args.max_speed_sync_delta_sec,
            args.minimum_contiguous_samples,
        )
        for admitted in admitted_runs
    ]
    manifest = {
        "schema_version": NORMAL_ANCHOR_SCHEMA_VERSION,
        "label_source": NORMAL_ANCHOR_LABEL_SOURCE,
        "expected_control_mode": args.expected_control_mode,
        "expected_checkpoint_sha256": args.expected_checkpoint_sha256.lower(),
        "max_speed_sync_delta_sec": args.max_speed_sync_delta_sec,
        "sequence_ids": [item["sequence_id"] for item in results],
        "summary": {
            "sequences": len(results),
            "train_sequences": sum(item["split"] == "train" for item in results),
            "validation_sequences": sum(item["split"] == "val" for item in results),
            "samples": sum(item["counts"]["accepted_samples"] for item in results),
        },
    }
    output_root.mkdir(parents=True, exist_ok=True)
    (output_root / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    for result in results:
        print(
            f"built split={result['split']} sequence={result['sequence_id']} "
            f"samples={result['counts']['accepted_samples']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
