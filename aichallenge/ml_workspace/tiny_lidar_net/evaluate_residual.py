#!/usr/bin/env python3
"""Evaluate correction learning and normal-state residual leakage."""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader, Subset

from lib.checkpoint import load_pretrained_weights
from lib.data import MultiSeqConcatDataset
from lib.residual import (
    MultiSeqResidualDataset,
    SteeringResidualNet,
    residual_metrics,
    write_json,
)


def predict(model, loader, device, paired: bool) -> tuple[np.ndarray, np.ndarray]:
    model.eval()
    predictions = []
    targets = []
    with torch.no_grad():
        for scans, target in loader:
            output = model(scans.unsqueeze(1).to(device)).cpu().numpy()
            predictions.append(output)
            targets.append(target.numpy() if paired else np.zeros_like(output))
    return np.concatenate(predictions), np.concatenate(targets)


def predict_paired_components(model, loader, device):
    """Expose whether a failure comes from correction capacity or gate collapse."""
    model.eval()
    residuals = []
    corrections = []
    gate_probabilities = []
    targets = []
    with torch.no_grad():
        for scans, target in loader:
            residual, correction, gate_logits = model.forward_components(
                scans.unsqueeze(1).to(device)
            )
            residuals.append(residual.cpu().numpy())
            corrections.append(correction.cpu().numpy())
            gate_probabilities.append(torch.sigmoid(gate_logits).cpu().numpy())
            targets.append(target.numpy())
    return tuple(
        np.concatenate(values)
        for values in (residuals, corrections, gate_probabilities, targets)
    )


def tail_subset(dataset: MultiSeqResidualDataset, tail_sec: float):
    """Select the final time interval from each provenance-checked sequence."""
    if not np.isfinite(tail_sec) or tail_sec <= 0.0:
        raise ValueError("tail_sec must be finite and positive")
    subsets = []
    for sequence in dataset.datasets:
        timestamps = np.asarray(sequence.scan_timestamps_ns, dtype=np.int64)
        cutoff_ns = int(timestamps[-1] - round(tail_sec * 1e9))
        indices = np.flatnonzero(timestamps >= cutoff_ns).tolist()
        if not indices:
            raise RuntimeError(
                f"Tail selection removed sequence {sequence.sequence_id}"
            )
        subsets.append(Subset(sequence, indices))
    return ConcatDataset(subsets)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "val"), default="val")
    parser.add_argument("--normal-dataset-dir", type=Path)
    parser.add_argument("--input-dim", type=int, default=750)
    parser.add_argument("--max-range-m", type=float, default=30.0)
    parser.add_argument("--max-abs-delta-rad", type=float, default=1.28)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument(
        "--include-sequence-id",
        action="append",
        help=(
            "Evaluate only sequence directories whose ID contains one of the "
            "given tokens. May be repeated for a diagnostic subset."
        ),
    )
    parser.add_argument(
        "--tail-sec",
        type=float,
        help="Evaluate only the final N seconds of every selected sequence.",
    )
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--min-material-improvement", type=float, default=0.30)
    parser.add_argument("--max-anchor-mae-rad", type=float, default=0.01)
    parser.add_argument("--max-normal-mae-rad", type=float, default=0.01)
    parser.add_argument("--fail-on-gate", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = SteeringResidualNet(
        input_dim=args.input_dim,
        max_abs_delta_rad=args.max_abs_delta_rad,
    ).to(device)
    provenance = load_pretrained_weights(model, args.checkpoint)
    paired_dataset = MultiSeqResidualDataset(
        args.dataset_dir,
        expected_split=args.split,
        max_range=args.max_range_m,
        expected_input_dim=args.input_dim,
        material_delta_rad=args.material_delta_rad,
        include=args.include_sequence_id,
    )
    evaluated_dataset = (
        paired_dataset
        if args.tail_sec is None
        else tail_subset(paired_dataset, args.tail_sec)
    )
    paired_loader = DataLoader(
        evaluated_dataset, batch_size=args.batch_size, shuffle=False
    )
    predictions, corrections, gate_probabilities, targets = (
        predict_paired_components(model, paired_loader, device)
    )
    paired_metrics = residual_metrics(
        predictions, targets, args.material_delta_rad
    )
    correction_metrics = residual_metrics(
        corrections, targets, args.material_delta_rad
    )
    material_mask = np.abs(targets) >= args.material_delta_rad
    component_diagnostics = {
        "ungated_correction_metrics": correction_metrics,
        "gate_probability": {
            "material_mean": (
                float(np.mean(gate_probabilities[material_mask]))
                if np.any(material_mask)
                else None
            ),
            "anchor_mean": (
                float(np.mean(gate_probabilities[~material_mask]))
                if np.any(~material_mask)
                else None
            ),
        },
    }

    normal_metrics = None
    if args.normal_dataset_dir is not None:
        normal_dataset = MultiSeqConcatDataset(
            args.normal_dataset_dir,
            expected_split="val",
            max_range=args.max_range_m,
            expected_input_dim=args.input_dim,
        )
        normal_loader = DataLoader(
            normal_dataset, batch_size=args.batch_size, shuffle=False
        )
        normal_predictions, normal_targets = predict(
            model, normal_loader, device, paired=False
        )
        normal_metrics = residual_metrics(
            normal_predictions, normal_targets, args.material_delta_rad
        )["all"]

    improvement = paired_metrics["material_mae_improvement_fraction"]
    anchor_mae = paired_metrics["anchor"]["mae_rad"]
    normal_mae = None if normal_metrics is None else normal_metrics["mae_rad"]
    gates = {
        "material_improvement": (
            improvement is not None and improvement >= args.min_material_improvement
        ),
        "anchor_leakage": (
            anchor_mae is not None and anchor_mae <= args.max_anchor_mae_rad
        ),
        "independent_normal_leakage": (
            normal_mae is None or normal_mae <= args.max_normal_mae_rad
        ),
    }
    result = {
        "schema_version": 1,
        "checkpoint": provenance,
        "paired_sequence_ids": paired_dataset.sequence_ids,
        "tail_sec": args.tail_sec,
        "paired_metrics": paired_metrics,
        "component_diagnostics": component_diagnostics,
        "independent_normal_metrics": normal_metrics,
        "thresholds": {
            "min_material_improvement": args.min_material_improvement,
            "max_anchor_mae_rad": args.max_anchor_mae_rad,
            "max_normal_mae_rad": args.max_normal_mae_rad,
        },
        "gates": gates,
        "result": "pass" if all(gates.values()) else "fail",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 2 if args.fail_on_gate and result["result"] != "pass" else 0


if __name__ == "__main__":
    raise SystemExit(main())
