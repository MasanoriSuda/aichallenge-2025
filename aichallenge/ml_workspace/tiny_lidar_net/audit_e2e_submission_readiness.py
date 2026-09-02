#!/usr/bin/env python3
"""Freeze E2E artifact identity and closed-loop submission readiness."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected a JSON object: {path}")
    return value


def values_match(actual: Any, expected: Any) -> bool:
    if isinstance(expected, float):
        return bool(
            isinstance(actual, (int, float))
            and not isinstance(actual, bool)
            and math.isfinite(float(actual))
            and math.isclose(float(actual), expected, rel_tol=0.0, abs_tol=1e-9)
        )
    return actual == expected


def validate_competition_gate(
    report: dict,
    *,
    expected_domain: int,
    expected_runtime: dict,
    expected_raw_sha256: str,
    expected_spatial_sha256: str,
    expected_motion_sha256: str | None = None,
) -> list[str]:
    """Validate the analyzer contract instead of trusting its status bit."""

    reasons = []
    if report.get("schema_version") != 1:
        reasons.append("competition report schema mismatch")
    if report.get("status") != "pass":
        reasons.append("competition report status is not pass")
    if not isinstance(report.get("run_dir"), str) or not report["run_dir"]:
        reasons.append("competition report run identity missing")
    if report.get("thresholds", {}).get("max_penalty_count") != 0:
        reasons.append("competition report did not require zero penalties")

    report_runtime = report.get("expected_runtime")
    if not isinstance(report_runtime, dict):
        reasons.append("competition expected runtime contract missing")
    else:
        for key, expected in expected_runtime.items():
            if key not in report_runtime:
                reasons.append(f"competition expected runtime missing {key}")
            elif not values_match(report_runtime[key], expected):
                reasons.append(f"competition expected runtime mismatch {key}")

    checkpoint = report.get("checkpoint_artifact")
    if not isinstance(checkpoint, dict) or checkpoint.get("sha256") != (
        expected_raw_sha256
    ):
        reasons.append("competition raw checkpoint identity mismatch")
    spatial = report.get("spatial_checkpoint_artifact")
    if not isinstance(spatial, dict) or spatial.get("sha256") != (
        expected_spatial_sha256
    ):
        reasons.append("competition spatial checkpoint identity mismatch")

    domains = report.get("domains")
    if not isinstance(domains, list) or not domains:
        reasons.append("competition domains missing")
        return reasons
    domain_ids = [
        domain.get("domain") for domain in domains if isinstance(domain, dict)
    ]
    if domain_ids != [expected_domain]:
        reasons.append("competition report domain identity mismatch")

    domain_runtime_keys = {
        "control_mode": "control_mode",
        "checkpoint_path": "checkpoint_path",
        "acceleration_mps2": "acceleration_mps2",
        "maximum_forward_speed_mps": "maximum_forward_speed_mps",
        "residual_checkpoint_path": "residual_checkpoint_path",
        "spatial_checkpoint_path": "spatial_checkpoint_path",
        "spatial_checkpoint_sha256": "spatial_expected_sha256",
        "spatial_use_base_steering": "spatial_use_base_steering",
        "spatial_authority_enabled": "spatial_authority_enabled",
        "spatial_authority_max_abs_delta_rad": (
            "spatial_authority_max_abs_delta_rad"
        ),
        "recurrent_checkpoint_path": "recurrent_checkpoint_path",
        "recurrent_authority_enabled": "recurrent_authority_enabled",
    }
    for domain in domains:
        if not isinstance(domain, dict) or domain.get("status") != "pass":
            reasons.append("competition contains a non-passing domain")
            continue
        runtime = domain.get("runtime")
        if not isinstance(runtime, dict):
            reasons.append("competition domain runtime provenance missing")
            continue
        for expected_key, domain_key in domain_runtime_keys.items():
            expected = expected_runtime[expected_key]
            if domain_key not in runtime:
                reasons.append(f"competition domain runtime missing {domain_key}")
            elif not values_match(runtime[domain_key], expected):
                reasons.append(f"competition domain runtime mismatch {domain_key}")
        if expected_motion_sha256 is not None:
            artifacts = domain.get("artifacts")
            if not isinstance(artifacts, dict) or artifacts.get(
                "motion_sha256"
            ) != expected_motion_sha256:
                reasons.append("competition motion evidence identity mismatch")
    return reasons


def validate_spatial_gate(
    report: dict,
    *,
    expected_domain: int,
    expected_run_dir: str,
    expected_spatial_sha256: str,
    expected_use_base_steering: bool,
    expected_authority_enabled: bool,
    expected_max_abs_delta_rad: float,
    expected_authority_max_abs_delta_rad: float,
    expected_competition_report_sha256: str,
    min_coverage_fraction: float = 0.99,
) -> list[str]:
    """Validate actual spatial execution rather than only launch provenance."""

    reasons = []
    if report.get("schema_version") != 1:
        reasons.append("spatial report schema mismatch")
    if report.get("status") != "pass":
        reasons.append("spatial report status is not pass")
    if report.get("domain") != expected_domain:
        reasons.append("spatial report domain mismatch")
    if report.get("run_dir") != expected_run_dir:
        reasons.append("spatial report run identity mismatch")
    if report.get("production_gate_status") != "pass":
        reasons.append("spatial report production Gate is not pass")
    runtime_evidence = report.get("runtime_evidence")
    if not isinstance(runtime_evidence, dict) or runtime_evidence.get(
        "competition_report_sha256"
    ) != expected_competition_report_sha256:
        reasons.append("spatial competition report identity mismatch")

    checkpoint = report.get("shadow_checkpoint")
    if not isinstance(checkpoint, dict) or checkpoint.get("sha256") != (
        expected_spatial_sha256
    ):
        reasons.append("spatial report checkpoint identity mismatch")

    runtime = report.get("runtime_config")
    expected_config = {
        "use_base_steering": expected_use_base_steering,
        "authority_enabled": expected_authority_enabled,
        "max_abs_delta_rad": expected_max_abs_delta_rad,
        "authority_max_abs_delta_rad": expected_authority_max_abs_delta_rad,
    }
    if not isinstance(runtime, dict):
        reasons.append("spatial runtime configuration missing")
    else:
        for key, expected in expected_config.items():
            if key not in runtime or not values_match(runtime[key], expected):
                reasons.append(f"spatial runtime configuration mismatch {key}")

    shadow = report.get("shadow")
    if not isinstance(shadow, dict):
        reasons.append("spatial execution summary missing")
        return reasons
    coverage = shadow.get("coverage_fraction")
    if not isinstance(coverage, (int, float)) or isinstance(coverage, bool) or (
        not math.isfinite(float(coverage))
        or float(coverage) < min_coverage_fraction
    ):
        reasons.append("spatial execution coverage below threshold")
    for key in ("error_count", "non_ok_interval_count", "stale_interval_count"):
        if shadow.get(key) != 0:
            reasons.append(f"spatial execution {key} is nonzero")
    interval_count = shadow.get("interval_count")
    if not isinstance(interval_count, int) or interval_count <= 0:
        reasons.append("spatial execution intervals missing")
    else:
        expected_enabled_intervals = (
            interval_count if expected_authority_enabled else 0
        )
        if (
            shadow.get("authority_enabled_interval_count")
            != expected_enabled_intervals
        ):
            reasons.append("spatial authority interval state mismatch")
    authority_applied_count = shadow.get("authority_applied_count")
    if expected_authority_enabled:
        if not (
            isinstance(authority_applied_count, int)
            and authority_applied_count > 0
        ):
            reasons.append("spatial authority was never applied")
    elif authority_applied_count != 0:
        reasons.append("spatial authority was unexpectedly applied")
    return reasons


def audit_submission_readiness(
    *,
    raw_checkpoint: Path,
    source_raw_checkpoint: Path,
    expected_raw_sha256: str,
    spatial_adapter: Path,
    source_spatial_adapter: Path,
    expected_spatial_sha256: str,
    expected_runtime: dict,
    expected_spatial_max_abs_delta_rad: float,
    single_competition: dict,
    single_competition_sha256: str,
    single_spatial: dict,
    single_domain: int,
    peer_motion: dict,
    peer_motion_sha256: str,
    peer_competition: dict,
    peer_competition_sha256: str,
    peer_spatial: dict | None,
    peer_domain: int,
    future_oracle: dict,
) -> dict:
    raw_sha = sha256_file(raw_checkpoint)
    source_raw_sha = sha256_file(source_raw_checkpoint)
    spatial_sha = sha256_file(spatial_adapter)
    source_spatial_sha = sha256_file(source_spatial_adapter)
    artifact_identity_pass = bool(
        raw_sha == expected_raw_sha256
        and source_raw_sha == expected_raw_sha256
        and spatial_sha == expected_spatial_sha256
        and source_spatial_sha == expected_spatial_sha256
        and str(raw_checkpoint) == expected_runtime["checkpoint_path"]
        and str(spatial_adapter) == expected_runtime["spatial_checkpoint_path"]
    )

    single_competition_reasons = validate_competition_gate(
        single_competition,
        expected_domain=single_domain,
        expected_runtime=expected_runtime,
        expected_raw_sha256=expected_raw_sha256,
        expected_spatial_sha256=expected_spatial_sha256,
    )
    single_competition_pass = not single_competition_reasons
    single_spatial_reasons = validate_spatial_gate(
        single_spatial,
        expected_domain=single_domain,
        expected_run_dir=single_competition.get("run_dir"),
        expected_spatial_sha256=expected_spatial_sha256,
        expected_use_base_steering=expected_runtime[
            "spatial_use_base_steering"
        ],
        expected_authority_enabled=expected_runtime[
            "spatial_authority_enabled"
        ],
        expected_max_abs_delta_rad=expected_spatial_max_abs_delta_rad,
        expected_authority_max_abs_delta_rad=expected_runtime[
            "spatial_authority_max_abs_delta_rad"
        ],
        expected_competition_report_sha256=single_competition_sha256,
    )
    single_spatial_pass = not single_spatial_reasons
    single_pass = single_competition_pass and single_spatial_pass
    peer_motion_pass = bool(
        peer_motion.get("admission", {}).get("result") == "pass"
    )
    peer_competition_reasons = validate_competition_gate(
        peer_competition,
        expected_domain=peer_domain,
        expected_runtime=expected_runtime,
        expected_raw_sha256=expected_raw_sha256,
        expected_spatial_sha256=expected_spatial_sha256,
        expected_motion_sha256=peer_motion_sha256,
    )
    peer_competition_pass = not peer_competition_reasons
    if peer_spatial is None:
        peer_spatial_reasons = ["mixed-peer spatial execution evidence missing"]
    else:
        peer_spatial_reasons = validate_spatial_gate(
            peer_spatial,
            expected_domain=peer_domain,
            expected_run_dir=peer_competition.get("run_dir"),
            expected_spatial_sha256=expected_spatial_sha256,
            expected_use_base_steering=expected_runtime[
                "spatial_use_base_steering"
            ],
            expected_authority_enabled=expected_runtime[
                "spatial_authority_enabled"
            ],
            expected_max_abs_delta_rad=expected_spatial_max_abs_delta_rad,
            expected_authority_max_abs_delta_rad=expected_runtime[
                "spatial_authority_max_abs_delta_rad"
            ],
            expected_competition_report_sha256=peer_competition_sha256,
        )
    peer_spatial_pass = not peer_spatial_reasons
    peer_pass = (
        peer_motion_pass and peer_competition_pass and peer_spatial_pass
    )
    oracle_comparison = future_oracle.get("comparison", {})
    oracle_discriminates = bool(
        oracle_comparison.get("future_occupancy_discriminates_failure", False)
    )
    oracle_scope = oracle_comparison.get("oracle_scope", {})
    oracle_label_permitted = bool(
        oracle_scope.get("label_generation_permitted", False)
    )
    oracle_runtime_permitted = bool(
        oracle_scope.get("runtime_input_permitted", False)
    )

    reasons = []
    if not artifact_identity_pass:
        reasons.append("production artifact identity mismatch")
    if not single_competition_pass:
        reasons.extend(
            f"single-vehicle competition Gate failed: {reason}"
            for reason in single_competition_reasons
        )
    if not single_spatial_pass:
        reasons.extend(
            f"single-vehicle spatial Gate failed: {reason}"
            for reason in single_spatial_reasons
        )
    if artifact_identity_pass and single_pass and not peer_motion_pass:
        reasons.append("mixed-peer motion Gate failed")
    if artifact_identity_pass and single_pass and not peer_competition_pass:
        reasons.extend(
            f"mixed-peer competition Gate failed: {reason}"
            for reason in peer_competition_reasons
        )
    if artifact_identity_pass and single_pass and not peer_spatial_pass:
        reasons.extend(
            f"mixed-peer spatial Gate failed: {reason}"
            for reason in peer_spatial_reasons
        )
    if not oracle_discriminates:
        reasons.append("privileged future-occupancy oracle is inconclusive")
    if oracle_label_permitted or oracle_runtime_permitted:
        reasons.append("privileged oracle scope unexpectedly permits production use")

    if not artifact_identity_pass or not single_pass:
        classification = "reject"
    elif peer_pass:
        classification = "multi-vehicle-candidate"
    else:
        classification = "single-vehicle-candidate-only"

    return {
        "schema_version": 3,
        "classification": classification,
        "artifact_identity_pass": artifact_identity_pass,
        "single_vehicle_gate_pass": single_pass,
        "single_vehicle_competition_gate_pass": single_competition_pass,
        "single_vehicle_spatial_gate_pass": single_spatial_pass,
        "mixed_peer_gate_pass": peer_pass,
        "mixed_peer_motion_gate_pass": peer_motion_pass,
        "mixed_peer_competition_gate_pass": peer_competition_pass,
        "mixed_peer_spatial_gate_pass": peer_spatial_pass,
        "privileged_oracle": {
            "classification": oracle_comparison.get("classification"),
            "discriminates_failure": oracle_discriminates,
            "label_generation_permitted": oracle_label_permitted,
            "runtime_input_permitted": oracle_runtime_permitted,
        },
        "artifacts": {
            "raw_checkpoint": str(raw_checkpoint),
            "raw_checkpoint_sha256": raw_sha,
            "source_raw_checkpoint": str(source_raw_checkpoint),
            "source_raw_checkpoint_sha256": source_raw_sha,
            "expected_raw_checkpoint_sha256": expected_raw_sha256,
            "spatial_adapter": str(spatial_adapter),
            "spatial_adapter_sha256": spatial_sha,
            "source_spatial_adapter": str(source_spatial_adapter),
            "source_spatial_adapter_sha256": source_spatial_sha,
            "expected_spatial_adapter_sha256": expected_spatial_sha256,
        },
        "expected_runtime": expected_runtime,
        "gate_details": {
            "single_competition_reasons": single_competition_reasons,
            "single_spatial_reasons": single_spatial_reasons,
            "peer_competition_reasons": peer_competition_reasons,
            "peer_spatial_reasons": peer_spatial_reasons,
        },
        "reasons": reasons,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-checkpoint", type=Path, required=True)
    parser.add_argument("--source-raw-checkpoint", type=Path, required=True)
    parser.add_argument("--expected-raw-sha256", required=True)
    parser.add_argument("--spatial-adapter", type=Path, required=True)
    parser.add_argument("--source-spatial-adapter", type=Path, required=True)
    parser.add_argument("--expected-spatial-sha256", required=True)
    parser.add_argument("--expected-control-mode", required=True)
    parser.add_argument("--expected-runtime-raw-checkpoint-path", required=True)
    parser.add_argument("--expected-acceleration-mps2", type=float, required=True)
    parser.add_argument(
        "--expected-maximum-forward-speed-mps", type=float, required=True
    )
    parser.add_argument("--expected-residual-checkpoint-path", required=True)
    parser.add_argument(
        "--expected-runtime-spatial-checkpoint-path", required=True
    )
    parser.add_argument(
        "--expected-spatial-use-base-steering",
        choices=("true", "false"),
        required=True,
    )
    parser.add_argument(
        "--expected-spatial-authority-enabled",
        choices=("true", "false"),
        required=True,
    )
    parser.add_argument(
        "--expected-spatial-max-abs-delta-rad", type=float, required=True
    )
    parser.add_argument(
        "--expected-spatial-authority-max-abs-delta-rad",
        type=float,
        required=True,
    )
    parser.add_argument("--expected-recurrent-checkpoint-path", required=True)
    parser.add_argument(
        "--expected-recurrent-authority-enabled",
        choices=("true", "false"),
        required=True,
    )
    parser.add_argument("--single-competition", type=Path, required=True)
    parser.add_argument("--single-spatial", type=Path, required=True)
    parser.add_argument("--single-domain", type=int, default=1)
    parser.add_argument("--peer-motion", type=Path, required=True)
    parser.add_argument("--peer-competition", type=Path, required=True)
    parser.add_argument("--peer-spatial", type=Path)
    parser.add_argument("--peer-domain", type=int, default=3)
    parser.add_argument("--future-oracle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-multivehicle", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths = (
        args.raw_checkpoint,
        args.source_raw_checkpoint,
        args.spatial_adapter,
        args.source_spatial_adapter,
        args.single_competition,
        args.single_spatial,
        args.peer_motion,
        args.peer_competition,
        args.future_oracle,
    )
    if args.peer_spatial is not None:
        paths += (args.peer_spatial,)
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"required evidence missing: {', '.join(missing)}")
    if args.single_domain <= 0 or args.peer_domain <= 0:
        raise ValueError("evidence domains must be positive")
    expected_runtime = {
        "control_mode": args.expected_control_mode,
        "checkpoint_path": args.expected_runtime_raw_checkpoint_path,
        "checkpoint_sha256": args.expected_raw_sha256.lower(),
        "acceleration_mps2": args.expected_acceleration_mps2,
        "maximum_forward_speed_mps": args.expected_maximum_forward_speed_mps,
        "residual_checkpoint_path": args.expected_residual_checkpoint_path,
        "spatial_checkpoint_path": (
            args.expected_runtime_spatial_checkpoint_path
        ),
        "spatial_checkpoint_sha256": args.expected_spatial_sha256.lower(),
        "spatial_use_base_steering": (
            args.expected_spatial_use_base_steering == "true"
        ),
        "spatial_authority_enabled": (
            args.expected_spatial_authority_enabled == "true"
        ),
        "spatial_authority_max_abs_delta_rad": (
            args.expected_spatial_authority_max_abs_delta_rad
        ),
        "recurrent_checkpoint_path": args.expected_recurrent_checkpoint_path,
        "recurrent_authority_enabled": (
            args.expected_recurrent_authority_enabled == "true"
        ),
    }
    result = audit_submission_readiness(
        raw_checkpoint=args.raw_checkpoint.expanduser().absolute(),
        source_raw_checkpoint=args.source_raw_checkpoint.expanduser().absolute(),
        expected_raw_sha256=args.expected_raw_sha256.lower(),
        spatial_adapter=args.spatial_adapter.expanduser().absolute(),
        source_spatial_adapter=(
            args.source_spatial_adapter.expanduser().absolute()
        ),
        expected_spatial_sha256=args.expected_spatial_sha256.lower(),
        expected_runtime=expected_runtime,
        expected_spatial_max_abs_delta_rad=(
            args.expected_spatial_max_abs_delta_rad
        ),
        single_competition=read_json(args.single_competition),
        single_competition_sha256=sha256_file(args.single_competition),
        single_spatial=read_json(args.single_spatial),
        single_domain=args.single_domain,
        peer_motion=read_json(args.peer_motion),
        peer_motion_sha256=sha256_file(args.peer_motion),
        peer_competition=read_json(args.peer_competition),
        peer_competition_sha256=sha256_file(args.peer_competition),
        peer_spatial=(
            None if args.peer_spatial is None else read_json(args.peer_spatial)
        ),
        peer_domain=args.peer_domain,
        future_oracle=read_json(args.future_oracle),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    if result["classification"] == "reject":
        return 2
    if args.require_multivehicle and result["classification"] != "multi-vehicle-candidate":
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
