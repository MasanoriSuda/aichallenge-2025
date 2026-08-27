"""Failure-first tests for the offline MPCC architecture audit platform."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from mpcc_architecture_audit.classification import classify_comparison
from mpcc_architecture_audit.manifest import ManifestError
from mpcc_architecture_audit.manifest import load_snapshot_manifest
from mpcc_architecture_audit.registry import RegistryError
from mpcc_architecture_audit.registry import load_registry


def _write_json(path: Path, value: object) -> Path:
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def test_replay_ready_snapshot_requires_all_immutable_payloads(tmp_path: Path) -> None:
    manifest = {
        "schema_version": 1,
        "snapshot_id": "pass-failure-1",
        "baseline_commit": "b6da7ebb296292bf57929ed9064a9fb789b95df0",
        "run_id": "20260827-214537",
        "domain_id": 1,
        "decision_id": 3048,
        "failure_family": "pass-return",
        "replay_ready": True,
        "payloads": {},
    }

    with pytest.raises(ManifestError, match="missing replay payload"):
        load_snapshot_manifest(_write_json(tmp_path / "snapshot.json", manifest))


def test_incomplete_evidence_must_not_claim_replay_ready(tmp_path: Path) -> None:
    manifest = {
        "schema_version": 1,
        "snapshot_id": "stop-log-only",
        "baseline_commit": "b6da7ebb296292bf57929ed9064a9fb789b95df0",
        "run_id": "20260827-214537",
        "domain_id": 1,
        "decision_id": 3048,
        "failure_family": "stop-wall",
        "replay_ready": False,
        "incomplete_reason": "log lacks exact solver and wall payloads",
        "payloads": {},
    }

    loaded = load_snapshot_manifest(_write_json(tmp_path / "snapshot.json", manifest))
    assert loaded["replay_ready"] is False


def test_all_fail_without_infeasibility_certificate_is_unknown() -> None:
    result = classify_comparison(
        {method: {"solve": "failed", "proof": "not_run"} for method in "ABCD"}
    )
    assert result.classification == "unknown"


def test_only_persistent_mission_failure_is_lifecycle_defect() -> None:
    result = classify_comparison(
        {
            "A": {"solve": "failed", "proof": "not_run"},
            "B": {"solve": "succeeded", "proof": "accepted"},
            "C": {"solve": "not_run", "proof": "not_run"},
            "D": {"solve": "not_run", "proof": "not_run"},
        }
    )
    assert result.classification == "mission_lifecycle_defect"


def test_solve_success_proof_failure_is_model_certificate_mismatch() -> None:
    result = classify_comparison(
        {
            "A": {"solve": "succeeded", "proof": "failed"},
            "B": {"solve": "failed", "proof": "not_run"},
            "C": {"solve": "failed", "proof": "not_run"},
            "D": {"solve": "failed", "proof": "not_run"},
        }
    )
    assert result.classification == "model_certificate_mismatch"


def test_registry_rejects_unknown_snapshot_reference(tmp_path: Path) -> None:
    registry = {
        "schema_version": 1,
        "snapshots": [],
        "experiments": [
            {
                "experiment_id": "exp-1",
                "baseline_commit": "b6da7ebb296292bf57929ed9064a9fb789b95df0",
                "snapshot_ids": ["missing"],
                "hypothesis": "mission lifecycle defect",
                "changed_dimension": "mission_lifecycle",
                "result": "inconclusive",
                "reason": "no complete snapshot",
                "production_impact": "none",
                "deleted_code": [],
                "revisit_condition": "capture snapshot",
            }
        ],
    }

    with pytest.raises(RegistryError, match="unknown snapshot"):
        load_registry(_write_json(tmp_path / "registry.json", registry))
