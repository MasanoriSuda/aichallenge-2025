#!/usr/bin/env python3
"""Audit current precontact-teacher semantics on teacher and normal states."""

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from lib.checkpoint import sha256_file
from lib.normal_anchor import MultiSeqNormalAnchorDataset
from lib.recurrent_policy import MultiSeqRecurrentPolicyDataset


def current_teacher_decisions(checkpoint: Path, scans_m: np.ndarray):
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
    base = []
    teacher = []
    reasons = Counter()
    for scan in scans_m:
        core.process(scan)
        decision = core.last_gap_teacher_decision
        if decision is None:
            raise RuntimeError("precontact teacher produced no decision")
        base.append(decision.base_steering_rad)
        teacher.append(decision.steering_rad)
        reasons[decision.reason] += 1
    return (
        np.asarray(base, dtype=np.float32),
        np.asarray(teacher, dtype=np.float32),
        dict(sorted(reasons.items())),
    )


def correction_summary(correction_rad: np.ndarray, material_delta_rad: float) -> dict:
    correction = np.asarray(correction_rad, dtype=np.float64)
    if correction.ndim != 1 or not len(correction):
        raise ValueError("correction audit requires a non-empty vector")
    if not np.all(np.isfinite(correction)) or material_delta_rad <= 0.0:
        raise ValueError("invalid correction audit input")
    left = correction <= -material_delta_rad
    right = correction >= material_delta_rad
    material = left | right
    return {
        "samples": len(correction),
        "mean_abs_rad": float(np.mean(np.abs(correction))),
        "p95_abs_rad": float(np.quantile(np.abs(correction), 0.95)),
        "max_abs_rad": float(np.max(np.abs(correction))),
        "material_samples": int(np.count_nonzero(material)),
        "material_fraction": float(np.mean(material)),
        "left_samples": int(np.count_nonzero(left)),
        "right_samples": int(np.count_nonzero(right)),
    }


def merge_reason_counts(records: list[dict]) -> dict:
    merged = Counter()
    for record in records:
        merged.update(record)
    return dict(sorted(merged.items()))


def audit_teacher_sequences(source, checkpoint: Path, material_delta_rad: float):
    sequence_reports = []
    all_corrections = []
    reason_records = []
    maximum_base_error = 0.0
    maximum_teacher_error = 0.0
    for sequence in source.datasets:
        base, teacher, reasons = current_teacher_decisions(
            checkpoint, sequence.scans
        )
        base_error = float(np.max(np.abs(base - sequence.base_steers)))
        teacher_error = float(np.max(np.abs(teacher - sequence.steers)))
        maximum_base_error = max(maximum_base_error, base_error)
        maximum_teacher_error = max(maximum_teacher_error, teacher_error)
        correction = teacher - base
        all_corrections.append(correction)
        reason_records.append(reasons)
        sequence_reports.append(
            {
                "sequence_id": sequence.sequence_id,
                "source_bag": str(sequence.metadata["source"]["bag"]),
                "stored_base_max_abs_error_rad": base_error,
                "stored_teacher_max_abs_error_rad": teacher_error,
                "current_teacher_correction": correction_summary(
                    correction, material_delta_rad
                ),
                "decision_reasons": reasons,
            }
        )
    return {
        "sequence_count": len(sequence_reports),
        "current_vs_stored": {
            "maximum_base_abs_error_rad": maximum_base_error,
            "maximum_teacher_abs_error_rad": maximum_teacher_error,
            "reproduced_at_1e_6_rad": (
                maximum_base_error <= 1e-6 and maximum_teacher_error <= 1e-6
            ),
        },
        "aggregate_correction": correction_summary(
            np.concatenate(all_corrections), material_delta_rad
        ),
        "decision_reasons": merge_reason_counts(reason_records),
        "sequences": sequence_reports,
    }


def audit_normal_sequences(source, checkpoint: Path, material_delta_rad: float):
    sequence_reports = []
    all_corrections = []
    reason_records = []
    for sequence in source.datasets:
        base, teacher, reasons = current_teacher_decisions(
            checkpoint, sequence.scans
        )
        correction = teacher - base
        all_corrections.append(correction)
        reason_records.append(reasons)
        sequence_reports.append(
            {
                "sequence_id": sequence.sequence_id,
                "source_bag": str(sequence.metadata["source"]["bag"]),
                "declared_target": "zero_residual",
                "current_teacher_correction": correction_summary(
                    correction, material_delta_rad
                ),
                "decision_reasons": reasons,
            }
        )
    aggregate = correction_summary(
        np.concatenate(all_corrections), material_delta_rad
    )
    return {
        "sequence_count": len(sequence_reports),
        "aggregate_current_teacher_correction": aggregate,
        "zero_target_consistent": aggregate["material_samples"] == 0,
        "decision_reasons": merge_reason_counts(reason_records),
        "sequences": sequence_reports,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teacher-dataset", type=Path, required=True)
    parser.add_argument("--normal-recurrent-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.material_delta_rad <= 0.0:
        raise ValueError("material delta must be positive")
    checkpoint = args.checkpoint.expanduser().resolve()
    teacher_root = args.teacher_dataset.expanduser().resolve()
    normal_root = args.normal_recurrent_root.expanduser().resolve()
    teacher = {
        split: audit_teacher_sequences(
            MultiSeqRecurrentPolicyDataset(teacher_root / split, split),
            checkpoint,
            args.material_delta_rad,
        )
        for split in ("train", "val")
    }
    normal = {
        split: audit_normal_sequences(
            MultiSeqNormalAnchorDataset(normal_root / split, split),
            checkpoint,
            args.material_delta_rad,
        )
        for split in ("train", "val")
    }
    report = {
        "schema_version": 1,
        "purpose": "offline teacher/normal label consistency audit",
        "checkpoint": str(checkpoint),
        "checkpoint_sha256": sha256_file(checkpoint),
        "material_delta_rad": args.material_delta_rad,
        "teacher_dataset": str(teacher_root),
        "normal_recurrent_root": str(normal_root),
        "teacher": teacher,
        "normal": normal,
    }
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "teacher_reproduced": {
                    split: report["teacher"][split]["current_vs_stored"]
                    for split in ("train", "val")
                },
                "normal_zero_consistent": {
                    split: report["normal"][split]["zero_target_consistent"]
                    for split in ("train", "val")
                },
                "normal_current_teacher": {
                    split: report["normal"][split][
                        "aggregate_current_teacher_correction"
                    ]
                    for split in ("train", "val")
                },
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
