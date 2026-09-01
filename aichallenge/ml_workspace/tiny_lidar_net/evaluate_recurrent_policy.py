#!/usr/bin/env python3
"""Evaluate a recurrent direct policy on complete ordered validation runs."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from lib.recurrent_policy import (
    MultiSeqRecurrentPolicyDataset,
    RecurrentDirectSteeringPolicy,
    direct_policy_metrics,
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def infer_sequence(model, sequence, device) -> np.ndarray:
    model.eval()
    scans = torch.from_numpy(sequence.scans).unsqueeze(0).to(device)
    speeds = torch.from_numpy(sequence.speeds).view(1, -1, 1).to(device)
    with torch.no_grad():
        predictions, _ = model(scans, speeds)
    values = predictions.squeeze(0).cpu().numpy()
    if values.shape != sequence.steers.shape or not np.all(np.isfinite(values)):
        raise ValueError(f"invalid recurrent inference for {sequence.sequence_id}")
    return values


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--val-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--minimum-material-improvement", type=float, default=0.30)
    parser.add_argument("--maximum-anchor-mae-rad", type=float, default=0.03)
    parser.add_argument("--unseen-source-bag-token", default="20260901-130837")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    checkpoint_path = args.checkpoint.expanduser().resolve()
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    if set(checkpoint) != {"model_config", "model_state_dict"}:
        raise ValueError("unexpected recurrent checkpoint contract")
    model = RecurrentDirectSteeringPolicy(**checkpoint["model_config"])
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)

    validation = MultiSeqRecurrentPolicyDataset(
        args.val_dir, expected_split="val"
    )
    all_predictions = []
    all_targets = []
    all_base = []
    per_sequence = []
    unseen = None
    for sequence in validation.datasets:
        predictions = infer_sequence(model, sequence, device)
        metrics = direct_policy_metrics(
            predictions,
            sequence.steers,
            sequence.base_steers,
            args.material_delta_rad,
        )
        record = {
            "sequence_id": sequence.sequence_id,
            "source_bag": sequence.metadata["source"]["bag"],
            "metrics": metrics,
            "prediction": {
                "min_rad": float(np.min(predictions)),
                "max_rad": float(np.max(predictions)),
                "finite": bool(np.all(np.isfinite(predictions))),
            },
        }
        per_sequence.append(record)
        if args.unseen_source_bag_token in record["source_bag"]:
            if unseen is not None:
                raise ValueError("unseen source bag token is ambiguous")
            unseen = record
        all_predictions.append(predictions)
        all_targets.append(sequence.steers)
        all_base.append(sequence.base_steers)
    if unseen is None:
        raise ValueError("unseen source bag token did not match a validation run")

    aggregate = direct_policy_metrics(
        np.concatenate(all_predictions),
        np.concatenate(all_targets),
        np.concatenate(all_base),
        args.material_delta_rad,
    )
    gates = {
        "material_improvement": (
            aggregate["material"]["improvement_fraction"] is not None
            and aggregate["material"]["improvement_fraction"]
            >= args.minimum_material_improvement
        ),
        "anchor_mae": (
            aggregate["anchor"]["candidate_mae_rad"]
            <= args.maximum_anchor_mae_rad
        ),
        "full_validation_not_worse": (
            aggregate["all"]["candidate_mae_rad"]
            <= aggregate["all"]["base_mae_rad"]
        ),
        "unseen_not_worse": (
            unseen["metrics"]["all"]["candidate_mae_rad"]
            <= unseen["metrics"]["all"]["base_mae_rad"]
        ),
        "finite_and_bounded": all(
            record["prediction"]["finite"]
            and record["prediction"]["min_rad"] >= -1.0
            and record["prediction"]["max_rad"] <= 1.0
            for record in per_sequence
        ),
    }
    report = {
        "schema_version": 1,
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": sha256_file(checkpoint_path),
        "model_config": checkpoint["model_config"],
        "dataset": str(args.val_dir.expanduser().resolve()),
        "aggregate": aggregate,
        "per_sequence": per_sequence,
        "unseen_sequence_id": unseen["sequence_id"],
        "gate_thresholds": {
            "material_delta_rad": args.material_delta_rad,
            "minimum_material_improvement": args.minimum_material_improvement,
            "maximum_anchor_mae_rad": args.maximum_anchor_mae_rad,
        },
        "gates": gates,
        "admitted": all(gates.values()),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report["gates"], indent=2, sort_keys=True))
    print(f"admitted={report['admitted']}")
    return 0 if report["admitted"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
