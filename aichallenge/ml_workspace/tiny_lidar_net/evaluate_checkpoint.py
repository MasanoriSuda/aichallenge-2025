#!/usr/bin/env python3
"""Evaluate a TinyLidarNet checkpoint against an auditable run-level split."""

import argparse
import json
from pathlib import Path
import sys
from typing import Dict, Optional, Tuple

import numpy as np
import torch
from torch.utils.data import DataLoader

from lib.checkpoint import load_pretrained_weights
from lib.data import MultiSeqConcatDataset
from lib.model import TinyLidarNet


def summarize_errors(predictions: np.ndarray, targets: np.ndarray) -> Dict[str, float]:
    """Return steering error metrics without hiding non-finite model output."""
    predictions = np.asarray(predictions, dtype=np.float64)
    targets = np.asarray(targets, dtype=np.float64)
    if predictions.shape != targets.shape or predictions.ndim != 1:
        raise ValueError(
            "predictions and targets must be equal-length one-dimensional arrays"
        )
    if predictions.size == 0:
        raise ValueError("cannot evaluate an empty dataset")
    if not np.all(np.isfinite(predictions)) or not np.all(np.isfinite(targets)):
        raise ValueError("predictions and targets must contain only finite values")

    error = predictions - targets
    absolute_error = np.abs(error)
    return {
        "mae_rad": float(np.mean(absolute_error)),
        "rmse_rad": float(np.sqrt(np.mean(np.square(error)))),
        "p95_absolute_error_rad": float(np.percentile(absolute_error, 95)),
        "max_absolute_error_rad": float(np.max(absolute_error)),
        "bias_rad": float(np.mean(error)),
    }


def evaluate_model(
    model: torch.nn.Module, loader: DataLoader, device: torch.device
) -> Tuple[np.ndarray, np.ndarray]:
    """Collect steering predictions and labels in deterministic dataset order."""
    model.eval()
    predictions = []
    targets = []
    with torch.no_grad():
        for scans, commands in loader:
            outputs = model(scans.unsqueeze(1).to(device))
            predictions.append(outputs[:, 1].detach().cpu().numpy())
            targets.append(commands[:, 1].detach().cpu().numpy())
    return np.concatenate(predictions), np.concatenate(targets)


def correction_subset_report(
    candidate_predictions: np.ndarray,
    baseline_predictions: np.ndarray,
    targets: np.ndarray,
    threshold_rad: float,
) -> dict:
    """Measure samples where the teacher materially differs from production."""
    if not np.isfinite(threshold_rad) or threshold_rad <= 0.0:
        raise ValueError("correction threshold must be finite and positive")
    baseline_error = np.abs(baseline_predictions - targets)
    selected = baseline_error >= threshold_rad
    report = {
        "threshold_rad": float(threshold_rad),
        "sample_count": int(np.count_nonzero(selected)),
        "sample_fraction": float(np.mean(selected)),
    }
    if np.any(selected):
        report["baseline"] = summarize_errors(
            baseline_predictions[selected], targets[selected]
        )
        report["candidate"] = summarize_errors(
            candidate_predictions[selected], targets[selected]
        )
    return report


def load_model(checkpoint: Path, device: torch.device) -> Tuple[TinyLidarNet, dict]:
    model = TinyLidarNet(input_dim=750, output_dim=2)
    provenance = load_pretrained_weights(model, checkpoint)
    return model.to(device), provenance


def build_report(
    dataset: MultiSeqConcatDataset,
    candidate_metrics: Dict[str, float],
    candidate_provenance: dict,
    baseline_metrics: Optional[Dict[str, float]] = None,
    baseline_provenance: Optional[dict] = None,
    correction_subset: Optional[dict] = None,
) -> dict:
    report = {
        "schema_version": 1,
        "dataset": {
            "sample_count": len(dataset),
            "sequence_ids": dataset.sequence_ids,
            "split": "val",
        },
        "candidate": {
            "checkpoint": candidate_provenance,
            "steering": candidate_metrics,
        },
    }
    if baseline_metrics is not None and baseline_provenance is not None:
        baseline_mae = baseline_metrics["mae_rad"]
        candidate_mae = candidate_metrics["mae_rad"]
        report["baseline"] = {
            "checkpoint": baseline_provenance,
            "steering": baseline_metrics,
        }
        report["comparison"] = {
            "candidate_minus_baseline_mae_rad": candidate_mae - baseline_mae,
            "candidate_minus_baseline_rmse_rad": (
                candidate_metrics["rmse_rad"] - baseline_metrics["rmse_rad"]
            ),
            "mae_improvement_fraction": (
                (baseline_mae - candidate_mae) / baseline_mae
                if baseline_mae > 0.0
                else 0.0
            ),
            "rmse_improvement_fraction": (
                (baseline_metrics["rmse_rad"] - candidate_metrics["rmse_rad"])
                / baseline_metrics["rmse_rad"]
                if baseline_metrics["rmse_rad"] > 0.0
                else 0.0
            ),
        }
        if correction_subset is not None:
            report["correction_subset"] = correction_subset
    return report


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Evaluate TinyLidarNet steering on an independent run-level split."
    )
    parser.add_argument("--dataset-dir", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--correction-threshold-rad", type=float, default=0.02)
    parser.add_argument(
        "--require-mae-improvement",
        action="store_true",
        help="Return status 2 unless candidate steering MAE is below baseline MAE.",
    )
    parser.add_argument(
        "--require-rmse-improvement",
        action="store_true",
        help="Return status 2 unless candidate steering RMSE is below baseline RMSE.",
    )
    args = parser.parse_args()

    if args.batch_size <= 0:
        parser.error("--batch-size must be positive")
    if (
        args.require_mae_improvement or args.require_rmse_improvement
    ) and args.baseline is None:
        parser.error("improvement gates require --baseline")
    if (
        not np.isfinite(args.correction_threshold_rad)
        or args.correction_threshold_rad <= 0.0
    ):
        parser.error("--correction-threshold-rad must be finite and positive")

    dataset = MultiSeqConcatDataset(
        args.dataset_dir,
        expected_split="val",
        expected_input_dim=750,
        max_range=30.0,
        max_sync_delta_sec=0.05,
    )
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    candidate, candidate_provenance = load_model(args.candidate, device)
    candidate_predictions, targets = evaluate_model(candidate, loader, device)
    candidate_metrics = summarize_errors(candidate_predictions, targets)

    baseline_metrics = None
    baseline_provenance = None
    correction_subset = None
    if args.baseline is not None:
        baseline, baseline_provenance = load_model(args.baseline, device)
        baseline_predictions, baseline_targets = evaluate_model(baseline, loader, device)
        if not np.array_equal(targets, baseline_targets):
            raise RuntimeError("candidate and baseline did not evaluate identical labels")
        baseline_metrics = summarize_errors(baseline_predictions, targets)
        correction_subset = correction_subset_report(
            candidate_predictions,
            baseline_predictions,
            targets,
            args.correction_threshold_rad,
        )

    report = build_report(
        dataset,
        candidate_metrics,
        candidate_provenance,
        baseline_metrics,
        baseline_provenance,
        correction_subset,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    print(rendered, end="")
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")

    if args.require_mae_improvement:
        if candidate_metrics["mae_rad"] >= baseline_metrics["mae_rad"]:
            return 2
    if args.require_rmse_improvement:
        if candidate_metrics["rmse_rad"] >= baseline_metrics["rmse_rad"]:
            return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
