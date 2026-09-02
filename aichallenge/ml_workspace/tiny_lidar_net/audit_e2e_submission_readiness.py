#!/usr/bin/env python3
"""Freeze E2E artifact identity and closed-loop submission readiness."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


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


def audit_submission_readiness(
    *,
    raw_checkpoint: Path,
    expected_raw_sha256: str,
    spatial_adapter: Path,
    expected_spatial_sha256: str,
    single_competition: dict,
    peer_motion: dict,
    future_oracle: dict,
) -> dict:
    raw_sha = sha256_file(raw_checkpoint)
    spatial_sha = sha256_file(spatial_adapter)
    artifact_identity_pass = bool(
        raw_sha == expected_raw_sha256
        and spatial_sha == expected_spatial_sha256
    )

    single_domains = single_competition.get("domains", [])
    single_pass = bool(
        single_competition.get("status") == "pass"
        and single_domains
        and all(domain.get("status") == "pass" for domain in single_domains)
    )
    peer_pass = bool(peer_motion.get("admission", {}).get("result") == "pass")
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
    if not single_pass:
        reasons.append("single-vehicle competition Gate failed")
    if artifact_identity_pass and single_pass and not peer_pass:
        reasons.append("mixed-peer motion Gate failed")
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
        "schema_version": 1,
        "classification": classification,
        "artifact_identity_pass": artifact_identity_pass,
        "single_vehicle_gate_pass": single_pass,
        "mixed_peer_gate_pass": peer_pass,
        "privileged_oracle": {
            "classification": oracle_comparison.get("classification"),
            "discriminates_failure": oracle_discriminates,
            "label_generation_permitted": oracle_label_permitted,
            "runtime_input_permitted": oracle_runtime_permitted,
        },
        "artifacts": {
            "raw_checkpoint": str(raw_checkpoint),
            "raw_checkpoint_sha256": raw_sha,
            "expected_raw_checkpoint_sha256": expected_raw_sha256,
            "spatial_adapter": str(spatial_adapter),
            "spatial_adapter_sha256": spatial_sha,
            "expected_spatial_adapter_sha256": expected_spatial_sha256,
        },
        "reasons": reasons,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-checkpoint", type=Path, required=True)
    parser.add_argument("--expected-raw-sha256", required=True)
    parser.add_argument("--spatial-adapter", type=Path, required=True)
    parser.add_argument("--expected-spatial-sha256", required=True)
    parser.add_argument("--single-competition", type=Path, required=True)
    parser.add_argument("--peer-motion", type=Path, required=True)
    parser.add_argument("--future-oracle", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-multivehicle", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths = (
        args.raw_checkpoint,
        args.spatial_adapter,
        args.single_competition,
        args.peer_motion,
        args.future_oracle,
    )
    missing = [str(path) for path in paths if not path.is_file()]
    if missing:
        raise FileNotFoundError(f"required evidence missing: {', '.join(missing)}")
    result = audit_submission_readiness(
        raw_checkpoint=args.raw_checkpoint.resolve(),
        expected_raw_sha256=args.expected_raw_sha256,
        spatial_adapter=args.spatial_adapter.resolve(),
        expected_spatial_sha256=args.expected_spatial_sha256,
        single_competition=read_json(args.single_competition),
        peer_motion=read_json(args.peer_motion),
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
