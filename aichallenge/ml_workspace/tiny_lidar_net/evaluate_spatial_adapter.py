#!/usr/bin/env python3
"""Gate a frozen-base spatial correction before any runtime integration."""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader

from lib.checkpoint import load_pretrained_weights
from lib.data import MultiSeqConcatDataset
from lib.model import TinyLidarNet
from lib.normal_anchor import MultiSeqNormalAnchorDataset
from lib.recurrent_policy import MultiSeqRecurrentPolicyDataset
from lib.residual import residual_metrics, write_json
from lib.spatial_adapter import (
    SPATIAL_NORMALIZATION_MODES,
    FrozenTinyLidarSpatialResidual,
)


def predict_paired(model, loader, device, material_delta_rad: float) -> dict:
    residuals = []
    targets = []
    directions = []
    model.eval()
    with torch.no_grad():
        for scans, speeds, teacher, base in loader:
            residual, _, logits, _ = model.forward_components(
                scans.to(device), speeds.to(device)
            )
            residuals.append(residual.cpu().numpy())
            targets.append((teacher - base).numpy())
            directions.append(torch.argmax(logits, dim=1).cpu().numpy())
    predicted = np.concatenate(residuals)
    target = np.concatenate(targets)
    predicted_direction = np.concatenate(directions)
    classes = np.ones(len(target), dtype=np.int64)
    classes[target <= -material_delta_rad] = 0
    classes[target >= material_delta_rad] = 2
    material = classes != 1
    anchor = classes == 1
    return {
        "residual": residual_metrics(predicted, target, material_delta_rad),
        "direction": {
            "accuracy": float(np.mean(predicted_direction == classes)),
            "material_sign_accuracy": (
                None
                if not np.any(material)
                else float(
                    np.mean(predicted_direction[material] == classes[material])
                )
            ),
            "anchor_false_material_fraction": (
                None
                if not np.any(anchor)
                else float(np.mean(predicted_direction[anchor] != 1))
            ),
            "class_support": {
                "left": int(np.count_nonzero(classes == 0)),
                "neutral": int(np.count_nonzero(classes == 1)),
                "right": int(np.count_nonzero(classes == 2)),
            },
        },
        "prediction_abs_max_rad": float(np.max(np.abs(predicted))),
        "finite": bool(np.all(np.isfinite(predicted))),
    }


def predict_normal(model, loader, device, max_range_m: float) -> dict:
    values = []
    model.eval()
    with torch.no_grad():
        for normalized_scans, _ in loader:
            physical_scans = normalized_scans.to(device) * max_range_m
            values.append(model(physical_scans).cpu().numpy())
    return normal_metrics(np.concatenate(values))


def predict_recurrent_normal(model, loader, device) -> dict:
    values = []
    model.eval()
    with torch.no_grad():
        for scans, speeds, _, _ in loader:
            values.append(
                model(scans.to(device), speeds.to(device)).cpu().numpy()
            )
    return normal_metrics(np.concatenate(values))


def normal_metrics(predicted: np.ndarray) -> dict:
    return {
        "samples": len(predicted),
        "mae_rad": float(np.mean(np.abs(predicted))),
        "p95_abs_rad": float(np.quantile(np.abs(predicted), 0.95)),
        "max_abs_rad": float(np.max(np.abs(predicted))),
        "finite": bool(np.all(np.isfinite(predicted))),
    }


def assert_embedded_base_identity(
    model: FrozenTinyLidarSpatialResidual, base_checkpoint: Path
) -> dict:
    reference = TinyLidarNet(input_dim=model.input_dim, output_dim=2)
    provenance = load_pretrained_weights(reference, base_checkpoint)
    mismatches = []
    for key, expected in reference.state_dict().items():
        actual = model.base.state_dict()[key]
        if not torch.equal(actual.cpu(), expected.cpu()):
            mismatches.append(key)
    if mismatches:
        raise ValueError(f"spatial candidate changed frozen base: {mismatches}")
    return provenance


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--normal-dataset-dir", type=Path)
    parser.add_argument("--normal-recurrent-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--input-dim", type=int, default=750)
    parser.add_argument("--hidden-dim", type=int, default=128)
    parser.add_argument("--max-range-m", type=float, default=30.0)
    parser.add_argument("--max-abs-delta-rad", type=float, default=1.2)
    parser.add_argument("--max-speed-mps", type=float, default=12.0)
    parser.add_argument("--use-speed", action="store_true")
    parser.add_argument(
        "--spatial-normalization",
        choices=SPATIAL_NORMALIZATION_MODES,
        default="layer_norm",
    )
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--peer-validation-token", default="20260901-153143/d3")
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--min-material-improvement", type=float, default=0.30)
    parser.add_argument("--min-material-sign-accuracy", type=float, default=0.80)
    parser.add_argument("--max-anchor-mae-rad", type=float, default=0.01)
    parser.add_argument("--max-normal-mae-rad", type=float, default=0.01)
    parser.add_argument("--fail-on-gate", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.batch_size <= 0:
        raise ValueError("spatial evaluation batch size must be positive")
    if (args.normal_dataset_dir is None) == (args.normal_recurrent_root is None):
        raise ValueError("provide exactly one normal validation source")
    if args.use_speed and args.normal_recurrent_root is None:
        raise ValueError("speed-enabled evaluation requires synchronized normal data")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = FrozenTinyLidarSpatialResidual(
        input_dim=args.input_dim,
        hidden_dim=args.hidden_dim,
        max_scan_range_m=args.max_range_m,
        max_abs_delta_rad=args.max_abs_delta_rad,
        use_speed=args.use_speed,
        max_speed_mps=args.max_speed_mps,
        spatial_normalization=args.spatial_normalization,
    )
    candidate_provenance = load_pretrained_weights(model, args.candidate)
    base_provenance = assert_embedded_base_identity(model, args.base_checkpoint)
    model.to(device)
    validation = MultiSeqRecurrentPolicyDataset(
        args.dataset.expanduser().resolve() / "val", "val"
    )
    validation_loader = DataLoader(
        ConcatDataset(validation.datasets),
        batch_size=args.batch_size,
        shuffle=False,
    )
    aggregate = predict_paired(
        model, validation_loader, device, args.material_delta_rad
    )
    peer_records = [
        sequence
        for sequence in validation.datasets
        if args.peer_validation_token
        in str(sequence.metadata.get("source", {}).get("bag", ""))
    ]
    if len(peer_records) != 1:
        raise ValueError(
            f"peer validation token matched {len(peer_records)} sequences"
        )
    peer = predict_paired(
        model,
        DataLoader(peer_records[0], batch_size=args.batch_size, shuffle=False),
        device,
        args.material_delta_rad,
    )
    if args.normal_recurrent_root is not None:
        normal = MultiSeqNormalAnchorDataset(
            args.normal_recurrent_root.expanduser().resolve() / "val", "val"
        )
        independent_normal = predict_recurrent_normal(
            model,
            DataLoader(
                ConcatDataset(normal.datasets),
                batch_size=args.batch_size,
                shuffle=False,
            ),
            device,
        )
    else:
        normal = MultiSeqConcatDataset(
            args.normal_dataset_dir,
            expected_split="val",
            max_range=args.max_range_m,
            expected_input_dim=args.input_dim,
        )
        independent_normal = predict_normal(
            model,
            DataLoader(normal, batch_size=args.batch_size, shuffle=False),
            device,
            args.max_range_m,
        )
    material_improvement = aggregate["residual"][
        "material_mae_improvement_fraction"
    ]
    material_sign = aggregate["direction"]["material_sign_accuracy"]
    peer_material_sign = peer["direction"]["material_sign_accuracy"]
    gates = {
        "embedded_base_identity": True,
        "finite_and_bounded": (
            aggregate["finite"]
            and independent_normal["finite"]
            and aggregate["prediction_abs_max_rad"]
            <= args.max_abs_delta_rad + 1e-6
        ),
        "material_improvement": (
            material_improvement is not None
            and material_improvement >= args.min_material_improvement
        ),
        "material_direction": (
            material_sign is not None
            and material_sign >= args.min_material_sign_accuracy
        ),
        "anchor_leakage": (
            aggregate["residual"]["anchor"]["mae_rad"]
            <= args.max_anchor_mae_rad
        ),
        "independent_normal_leakage": (
            independent_normal["mae_rad"] <= args.max_normal_mae_rad
        ),
        "peer_direction": (
            peer_material_sign is not None
            and peer_material_sign >= args.min_material_sign_accuracy
        ),
        "peer_anchor_leakage": (
            peer["residual"]["anchor"]["mae_rad"] <= args.max_anchor_mae_rad
        ),
    }
    report = {
        "schema_version": 1,
        "purpose": "offline gate; no runtime authority",
        "candidate": candidate_provenance,
        "base_checkpoint": base_provenance,
        "validation_sequence_ids": validation.sequence_ids,
        "peer_sequence_id": peer_records[0].sequence_id,
        "aggregate": aggregate,
        "peer_validation": peer,
        "independent_normal": independent_normal,
        "thresholds": {
            "min_material_improvement": args.min_material_improvement,
            "min_material_sign_accuracy": args.min_material_sign_accuracy,
            "max_anchor_mae_rad": args.max_anchor_mae_rad,
            "max_normal_mae_rad": args.max_normal_mae_rad,
        },
        "gates": gates,
        "result": "pass" if all(gates.values()) else "fail",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, report)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 2 if args.fail_on_gate and report["result"] != "pass" else 0


if __name__ == "__main__":
    raise SystemExit(main())
