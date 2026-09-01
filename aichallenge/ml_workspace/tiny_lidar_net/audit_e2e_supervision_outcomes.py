#!/usr/bin/env python3
"""Classify E2E steering labels by executed policy and certified run outcome."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import re

import numpy as np


SCHEMA_VERSION = 1
CONTROL_MODE_PATTERN = re.compile(r"tiny_lidar_control_mode:\s*([^\s]+)")


def load_json_object(path: Path) -> dict:
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


def read_control_mode(log_path: Path) -> tuple[str | None, str]:
    if not log_path.is_file():
        return None, "missing-autoware-log"
    modes = set(CONTROL_MODE_PATTERN.findall(log_path.read_text(errors="ignore")))
    if not modes:
        return None, "control-mode-not-recorded"
    if len(modes) != 1:
        return None, "ambiguous-control-mode"
    return modes.pop(), "ok"


def read_outcome(detail_path: Path, domain: int) -> dict:
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


def evidence_class(control_mode: str | None, outcome: str) -> str:
    teacher_executed = control_mode == "precontact_teacher"
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


def correction_summary(sequence_dir: Path, material_delta_rad: float) -> dict:
    steering_path = sequence_dir / "steers.npy"
    base_path = sequence_dir / "base_steers.npy"
    if not steering_path.is_file() or not base_path.is_file():
        count_path = sequence_dir / "scans.npy"
        if not count_path.is_file():
            raise FileNotFoundError(f"missing supervision arrays in {sequence_dir}")
        samples = len(np.load(count_path, allow_pickle=False))
        return {"samples": samples, "material_samples": 0, "material_fraction": 0.0}
    steering = np.load(steering_path, allow_pickle=False)
    base = np.load(base_path, allow_pickle=False)
    if steering.ndim != 1 or steering.shape != base.shape or not len(steering):
        raise ValueError(f"invalid steering arrays in {sequence_dir}")
    correction = steering - base
    material = np.abs(correction) >= material_delta_rad
    return {
        "samples": len(correction),
        "material_samples": int(np.count_nonzero(material)),
        "material_fraction": float(np.mean(material)),
    }


def audit_sequence(
    sequence_dir: Path,
    output_root: Path,
    material_delta_rad: float,
    expected_kind: str,
) -> dict:
    metadata = load_json_object(sequence_dir / "metadata.json")
    source = metadata.get("source")
    if not isinstance(source, dict) or not isinstance(source.get("bag"), str):
        raise ValueError(f"sequence lacks immutable source bag: {sequence_dir}")
    bag = resolve_recorded_path(source["bag"], output_root)
    domain, run = source_domain_and_run(bag)
    control_mode, control_status = read_control_mode(bag.parent / "autoware.log")
    outcome = read_outcome(run / f"d{domain}-result-details.json", domain)
    evidence = evidence_class(control_mode, outcome["classification"])
    summary = correction_summary(sequence_dir, material_delta_rad)
    if expected_kind == "normal":
        summary["material_samples"] = 0
        summary["material_fraction"] = 0.0
    return {
        "sequence_id": metadata.get("sequence_id"),
        "split": metadata.get("split"),
        "kind": expected_kind,
        "source_bag_recorded": source["bag"],
        "source_bag_resolved": str(bag),
        "domain": domain,
        "control_mode": control_mode,
        "control_mode_status": control_status,
        "outcome": outcome,
        "evidence_class": evidence,
        "hard_teacher_label_admissible": (
            expected_kind == "teacher" and evidence == "executed_teacher_success"
        ),
        "demonstrated_zero_action": (
            expected_kind == "normal"
            and evidence == "successful_alternative_policy"
        ),
        "label_summary": summary,
    }


def sequence_dirs(root: Path) -> list[Path]:
    return sorted(
        path.parent
        for path in root.rglob("metadata.json")
        if path.parent.is_dir()
    )


def summarize(records: list[dict]) -> dict:
    classes = Counter(item["evidence_class"] for item in records)
    samples = Counter()
    material = Counter()
    for item in records:
        name = item["evidence_class"]
        samples[name] += item["label_summary"]["samples"]
        material[name] += item["label_summary"]["material_samples"]
    return {
        "sequences": len(records),
        "samples": int(sum(samples.values())),
        "evidence_sequence_counts": dict(sorted(classes.items())),
        "evidence_sample_counts": dict(sorted(samples.items())),
        "evidence_material_sample_counts": dict(sorted(material.items())),
        "hard_teacher_label_admissible_sequences": sum(
            bool(item["hard_teacher_label_admissible"]) for item in records
        ),
        "demonstrated_zero_action_sequences": sum(
            bool(item["demonstrated_zero_action"]) for item in records
        ),
    }


def audit_corpus(
    root: Path,
    output_root: Path,
    material_delta_rad: float,
    kind: str,
) -> dict:
    records = [
        audit_sequence(path, output_root, material_delta_rad, kind)
        for path in sequence_dirs(root)
    ]
    if not records:
        raise RuntimeError(f"no supervision sequences found under {root}")
    return {"summary": summarize(records), "sequences": records}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher-dataset", type=Path, required=True)
    parser.add_argument("--normal-recurrent-root", type=Path, required=True)
    parser.add_argument("--host-output-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.material_delta_rad <= 0.0:
        raise ValueError("material delta must be positive")
    report = {
        "schema_version": SCHEMA_VERSION,
        "purpose": "E2E supervision outcome and execution provenance audit",
        "material_delta_rad": args.material_delta_rad,
        "teacher_dataset": str(args.teacher_dataset.resolve()),
        "normal_recurrent_root": str(args.normal_recurrent_root.resolve()),
        "host_output_root": str(args.host_output_root.resolve()),
        "teacher": audit_corpus(
            args.teacher_dataset,
            args.host_output_root,
            args.material_delta_rad,
            "teacher",
        ),
        "normal": audit_corpus(
            args.normal_recurrent_root,
            args.host_output_root,
            args.material_delta_rad,
            "normal",
        ),
    }
    report["contract"] = {
        "hard_teacher_labels_available": (
            report["teacher"]["summary"]["hard_teacher_label_admissible_sequences"]
            > 0
        ),
        "demonstrated_zero_action_available": (
            report["normal"]["summary"]["demonstrated_zero_action_sequences"] > 0
        ),
        "ready_for_exclusive_action_training": (
            report["teacher"]["summary"]["hard_teacher_label_admissible_sequences"]
            > 0
            and report["normal"]["summary"]["demonstrated_zero_action_sequences"]
            > 0
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "teacher": report["teacher"]["summary"],
        "normal": report["normal"]["summary"],
        "contract": report["contract"],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
