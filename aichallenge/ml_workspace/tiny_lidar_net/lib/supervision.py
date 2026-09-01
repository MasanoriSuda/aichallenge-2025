"""Fail-closed supervision outcome and artifact provenance contracts."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any, Optional

from lib.checkpoint import sha256_file


OUTCOME_CERTIFICATE_SCHEMA_VERSION = 1
EXECUTED_TEACHER_MODES = frozenset(
    {"precontact_teacher", "speed_committed_teacher"}
)
CONTROL_MODE_PATTERN = re.compile(r"tiny_lidar_control_mode:\s*([^\s]+)")
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}")


def load_json_object(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid JSON object {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return value


def resolve_recorded_path(recorded: str, output_root: Path) -> Path:
    path = Path(recorded)
    if path.is_absolute() and len(path.parts) >= 2 and path.parts[1] == "output":
        return output_root.joinpath(*path.parts[2:])
    return path


def source_domain_and_run(bag: Path) -> tuple[int, Path]:
    domain_name = bag.parent.name
    if not re.fullmatch(r"d[1-9][0-9]*", domain_name):
        raise ValueError(f"source bag does not belong to a domain directory: {bag}")
    return int(domain_name[1:]), bag.parents[1]


def read_control_mode(log_path: Path) -> tuple[Optional[str], str]:
    if not log_path.is_file():
        return None, "missing-autoware-log"
    modes = set(CONTROL_MODE_PATTERN.findall(log_path.read_text(errors="ignore")))
    if not modes:
        return None, "control-mode-not-recorded"
    if len(modes) != 1:
        return None, "ambiguous-control-mode"
    return modes.pop(), "ok"


def read_outcome(detail_path: Path, domain: int) -> dict[str, Any]:
    if not detail_path.is_file():
        return {
            "classification": "outcome_unproven",
            "reason": "missing-result-detail",
        }
    try:
        detail = load_json_object(detail_path)
    except ValueError as exc:
        return {
            "classification": "outcome_unproven",
            "reason": str(exc),
        }
    required = ("vehicle_number", "finished", "lap_count", "required_laps", "penalty_count")
    if detail.get("schema_version") != "v3" or any(key not in detail for key in required):
        return {
            "classification": "outcome_unproven",
            "reason": "invalid-result-detail-contract",
        }
    if detail["vehicle_number"] != domain:
        return {
            "classification": "outcome_unproven",
            "reason": "result-domain-mismatch",
        }
    scalar_contract = (
        isinstance(detail["finished"], bool)
        and isinstance(detail["lap_count"], int)
        and isinstance(detail["required_laps"], int)
        and isinstance(detail["penalty_count"], int)
        and detail["lap_count"] >= 0
        and detail["required_laps"] > 0
        and detail["penalty_count"] >= 0
    )
    if not scalar_contract:
        return {
            "classification": "outcome_unproven",
            "reason": "invalid-result-detail-values",
        }
    passed = (
        detail["finished"]
        and detail["lap_count"] >= detail["required_laps"]
        and detail["penalty_count"] == 0
    )
    return {
        "classification": "certified_success" if passed else "certified_failure",
        "reason": "finish-zero-penalty" if passed else "race-gate-failed",
        "finished": detail["finished"],
        "lap_count": detail["lap_count"],
        "required_laps": detail["required_laps"],
        "penalty_count": detail["penalty_count"],
    }


def evidence_class(control_mode: Optional[str], outcome: str) -> str:
    teacher_executed = control_mode in EXECUTED_TEACHER_MODES
    if outcome == "outcome_unproven":
        return "outcome_unproven"
    if outcome == "certified_success":
        return (
            "executed_teacher_success"
            if teacher_executed
            else "successful_alternative_policy"
        )
    if outcome == "certified_failure":
        return (
            "executed_teacher_failure"
            if teacher_executed
            else "counterfactual_teacher_on_failure"
        )
    raise ValueError(f"unknown outcome classification: {outcome}")


def canonical_json_sha256(value: dict[str, Any]) -> str:
    rendered = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    )
    return hashlib.sha256(rendered.encode("utf-8")).hexdigest()


def _require_sha256(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value.lower()) is None:
        raise ValueError(f"{label} must be a lowercase SHA-256 digest")
    return value.lower()


def _recorded_suffix(recorded: Any, run_name: str) -> tuple[str, ...]:
    if not isinstance(recorded, str) or not Path(recorded).is_absolute():
        raise ValueError("recorded artifact path must be absolute")
    parts = Path(recorded).parts
    matches = [index for index, part in enumerate(parts) if part == run_name]
    if not matches:
        raise ValueError("recorded artifact path does not contain source run")
    return tuple(parts[matches[-1]:])


def _require_artifact(
    path: Path,
    recorded_path: Any,
    expected_sha256: Any,
    run_name: str,
    label: str,
) -> str:
    if not path.is_file():
        raise FileNotFoundError(f"{label} missing: {path}")
    if _recorded_suffix(recorded_path, run_name) != _recorded_suffix(
        str(path.resolve()), run_name
    ):
        raise ValueError(f"{label} path identity mismatch")
    expected = _require_sha256(expected_sha256, f"{label} hash")
    actual = sha256_file(path)
    if actual != expected:
        raise ValueError(f"{label} hash mismatch")
    return actual


def admit_successful_run(
    run_dir: Path,
    domain: int,
    expected_control_mode: str,
    expected_checkpoint_sha256: str,
    *,
    report_path: Optional[Path] = None,
    require_bag: bool = True,
) -> dict[str, Any]:
    """Validate a successful executed policy and return immutable evidence.

    Absolute host and container mount prefixes may differ, but the run name and
    every artifact suffix below it must match.  File hashes then bind the exact
    local artifacts rather than trusting path text in a generated report.
    """
    run_dir = run_dir.expanduser().resolve()
    if domain <= 0:
        raise ValueError("domain must be positive")
    expected_hash = _require_sha256(
        expected_checkpoint_sha256, "expected checkpoint hash"
    )
    report_path = (
        run_dir / "e2e-competition-analysis.json"
        if report_path is None
        else report_path.expanduser().resolve()
    )
    if report_path.parent != run_dir:
        raise ValueError("competition analysis must belong to the source run")
    report = load_json_object(report_path)
    if (
        report.get("schema_version") != 1
        or report.get("status") != "pass"
        or report.get("reasons") != []
    ):
        raise ValueError(f"source run was not admitted: {run_dir}")
    reported_run = Path(str(report.get("run_dir", "")))
    if not reported_run.is_absolute() or reported_run.name != run_dir.name:
        raise ValueError("competition report run identity mismatch")

    expected_runtime = report.get("expected_runtime")
    checkpoint_artifact = report.get("checkpoint_artifact")
    global_artifacts = report.get("artifacts")
    if not all(
        isinstance(item, dict)
        for item in (expected_runtime, checkpoint_artifact, global_artifacts)
    ):
        raise ValueError("competition report lacks frozen runtime proof")
    if expected_runtime.get("control_mode") != expected_control_mode:
        raise ValueError("competition report control mode mismatch")
    if expected_runtime.get("checkpoint_sha256") != expected_hash:
        raise ValueError("competition report checkpoint expectation mismatch")
    if checkpoint_artifact.get("sha256") != expected_hash:
        raise ValueError("competition report checkpoint artifact mismatch")

    domains = report.get("domains")
    matches = (
        [
            item
            for item in domains
            if isinstance(item, dict) and item.get("domain") == domain
        ]
        if isinstance(domains, list)
        else []
    )
    if len(matches) != 1:
        raise ValueError("competition report domain identity is ambiguous")
    domain_report = matches[0]
    if domain_report.get("status") != "pass" or domain_report.get("reasons") != []:
        raise ValueError("requested source domain was not admitted")
    runtime = domain_report.get("runtime")
    race = domain_report.get("race")
    motion = domain_report.get("motion")
    artifacts = domain_report.get("artifacts")
    if not all(isinstance(item, dict) for item in (runtime, race, motion, artifacts)):
        raise ValueError("source domain evidence is incomplete")
    if runtime.get("control_mode") != expected_control_mode or runtime.get("conflicts"):
        raise ValueError("runtime controller identity is not unique")

    lap_count = race.get("lap_count")
    required_laps = race.get("required_laps")
    if (
        not race.get("finished")
        or not isinstance(lap_count, int)
        or not isinstance(required_laps, int)
        or required_laps <= 0
        or lap_count < required_laps
    ):
        raise ValueError("source run did not finish required laps")
    if race.get("penalty_count") != 0:
        raise ValueError("penalized run cannot certify supervision")
    for key in ("longest_low_speed_sec", "longest_positive_accel_stall_sec"):
        value = motion.get(key)
        if (
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value > 0.0
        ):
            raise ValueError(f"source motion violation: {key}")

    summary_path = run_dir / "result-summary.json"
    detail_path = run_dir / f"d{domain}-result-details.json"
    motion_path = run_dir / f"d{domain}" / "e2e-run-analysis.json"
    summary_hash = _require_artifact(
        summary_path,
        global_artifacts.get("result_summary"),
        global_artifacts.get("result_summary_sha256"),
        run_dir.name,
        "result summary",
    )
    detail_hash = _require_artifact(
        detail_path,
        artifacts.get("result_detail"),
        artifacts.get("result_detail_sha256"),
        run_dir.name,
        "result detail",
    )
    motion_hash = _require_artifact(
        motion_path,
        artifacts.get("motion_analysis"),
        artifacts.get("motion_sha256"),
        run_dir.name,
        "motion analysis",
    )
    detail = load_json_object(detail_path)
    outcome = read_outcome(detail_path, domain)
    if outcome.get("classification") != "certified_success":
        raise ValueError("result detail does not prove successful outcome")
    if any(detail.get(key) != race.get(key) for key in (
        "finished", "lap_count", "required_laps", "penalty_count"
    )):
        raise ValueError("competition report race evidence mismatch")
    motion_document = load_json_object(motion_path)
    admission = motion_document.get("admission")
    if not isinstance(admission, dict) or admission.get("result") != "pass":
        raise ValueError("motion analysis admission failed")
    if motion_document.get("motion") != motion:
        raise ValueError("competition report motion evidence mismatch")

    bag = run_dir / f"d{domain}" / "rosbag2_autoware"
    if require_bag and not (bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"finalized source bag missing: {bag}")
    report_hash = sha256_file(report_path)
    certificate = {
        "schema_version": OUTCOME_CERTIFICATE_SCHEMA_VERSION,
        "evidence_class": evidence_class(expected_control_mode, "certified_success"),
        "source_run": str(run_dir),
        "source_run_id": run_dir.name,
        "source_bag": str(bag.resolve()),
        "domain": domain,
        "control_mode": expected_control_mode,
        "checkpoint_sha256": expected_hash,
        "competition_analysis": str(report_path),
        "competition_analysis_sha256": report_hash,
        "result_summary_sha256": summary_hash,
        "result_detail_sha256": detail_hash,
        "motion_analysis_sha256": motion_hash,
        "race": race,
        "motion": motion,
    }
    return {
        "run_dir": run_dir,
        "run_id": run_dir.name,
        "domain": domain,
        "bag": bag.resolve(),
        "report_path": report_path,
        "report_sha256": report_hash,
        "result_summary_sha256": summary_hash,
        "result_detail_sha256": detail_hash,
        "motion_analysis_sha256": motion_hash,
        "checkpoint_sha256": expected_hash,
        "control_mode": expected_control_mode,
        "race": race,
        "motion": motion,
        "outcome_certificate": certificate,
        "outcome_certificate_sha256": canonical_json_sha256(certificate),
    }


def validate_executed_teacher_certificate(
    certificate: Any,
    *,
    source_bag: Optional[Path] = None,
    checkpoint_sha256: Optional[str] = None,
    expected_control_mode: str = "precontact_teacher",
) -> str:
    """Validate an embedded certificate and return its canonical digest."""
    if not isinstance(certificate, dict):
        raise ValueError("executed teacher outcome certificate is missing")
    if certificate.get("schema_version") != OUTCOME_CERTIFICATE_SCHEMA_VERSION:
        raise ValueError("executed teacher certificate schema mismatch")
    if certificate.get("evidence_class") != "executed_teacher_success":
        raise ValueError("supervision is not an executed teacher success")
    if expected_control_mode not in EXECUTED_TEACHER_MODES:
        raise ValueError("unsupported executed teacher mode")
    if certificate.get("control_mode") != expected_control_mode:
        raise ValueError("executed teacher certificate mode mismatch")
    _require_sha256(certificate.get("checkpoint_sha256"), "certificate checkpoint")
    for key in (
        "competition_analysis_sha256",
        "result_summary_sha256",
        "result_detail_sha256",
        "motion_analysis_sha256",
    ):
        _require_sha256(certificate.get(key), f"certificate {key}")
    if checkpoint_sha256 is not None and certificate["checkpoint_sha256"] != (
        checkpoint_sha256.lower()
    ):
        raise ValueError("executed teacher certificate checkpoint mismatch")
    if source_bag is not None:
        recorded_bag = Path(str(certificate.get("source_bag", "")))
        domain, run = source_domain_and_run(source_bag.resolve())
        try:
            recorded_domain, recorded_run = source_domain_and_run(recorded_bag)
        except (IndexError, ValueError) as exc:
            raise ValueError(
                "executed teacher certificate source mismatch"
            ) from exc
        if (
            certificate.get("domain") != domain
            or recorded_domain != domain
            or certificate.get("source_run_id") != run.name
            or recorded_bag.name != source_bag.name
            or recorded_run.name != run.name
        ):
            raise ValueError("executed teacher certificate source mismatch")
    race = certificate.get("race")
    motion = certificate.get("motion")
    if not isinstance(race, dict) or not isinstance(motion, dict):
        raise ValueError("executed teacher certificate outcome is incomplete")
    if (
        race.get("finished") is not True
        or race.get("penalty_count") != 0
        or not isinstance(race.get("lap_count"), int)
        or not isinstance(race.get("required_laps"), int)
        or race["lap_count"] < race["required_laps"]
    ):
        raise ValueError("executed teacher certificate race gate failed")
    if any(motion.get(key) != 0.0 for key in (
        "longest_low_speed_sec", "longest_positive_accel_stall_sec"
    )):
        raise ValueError("executed teacher certificate motion gate failed")
    return canonical_json_sha256(certificate)
