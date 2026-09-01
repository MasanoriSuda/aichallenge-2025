#!/usr/bin/env python3
"""Evaluate a recurrent direct policy on complete ordered validation runs."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from lib.checkpoint import load_pretrained_weights
from lib.model import TinyLidarNet
from lib.normal_anchor import MultiSeqNormalAnchorDataset
from lib.recurrent_policy import (
    FrozenTinyLidarRecurrentAdapter,
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


from lib.spatial_adapter import FrozenTinyLidarSpatialResidual


def infer_sequence(
    model, sequence, device, correction_deadband_rad: float = 0.0
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    model.eval()
    scans = torch.from_numpy(sequence.scans).unsqueeze(0).to(device)
    speeds = torch.from_numpy(sequence.speeds).view(1, -1, 1).to(device)
    with torch.no_grad():
        if isinstance(model, FrozenTinyLidarRecurrentAdapter):
            (
                predictions,
                correction,
                embedded_base,
                _,
                direction_logits,
                _,
            ) = model.forward_correction_components(
                scans, speeds
            )
            base = embedded_base.squeeze(0).cpu().numpy()
            predicted_classes = (
                None
                if direction_logits is None
                else torch.argmax(direction_logits, dim=-1).squeeze(0).cpu().numpy()
            )
        else:
            predictions, _ = model(scans, speeds)
            base = sequence.base_steers
            correction = predictions - torch.from_numpy(base).to(
                device=predictions.device,
                dtype=predictions.dtype,
            )
            predicted_classes = None
    correction_values = correction.squeeze(0).cpu().numpy()
    if correction_deadband_rad > 0.0:
        correction_values = np.where(
            np.abs(correction_values) >= correction_deadband_rad,
            correction_values,
            0.0,
        )
    values = np.clip(base + correction_values, -1.0, 1.0)
    if (
        values.shape != sequence.steers.shape
        or base.shape != sequence.steers.shape
        or not np.all(np.isfinite(values))
        or not np.all(np.isfinite(base))
    ):
        raise ValueError(f"invalid recurrent inference for {sequence.sequence_id}")
    return values, base, predicted_classes


def direction_metrics(
    predicted_classes: np.ndarray,
    targets: np.ndarray,
    base: np.ndarray,
    material_delta_rad: float,
) -> dict:
    correction = np.asarray(targets) - np.asarray(base)
    truth = np.ones(len(correction), dtype=np.int64)
    truth[correction <= -material_delta_rad] = 0
    truth[correction >= material_delta_rad] = 2
    predicted = np.asarray(predicted_classes, dtype=np.int64)
    if predicted.shape != truth.shape or np.any((predicted < 0) | (predicted > 2)):
        raise ValueError("invalid recurrent direction classes")
    material = truth != 1
    anchor = ~material
    return {
        "accuracy": float(np.mean(predicted == truth)),
        "material_sign_accuracy": (
            None
            if not np.any(material)
            else float(np.mean(predicted[material] == truth[material]))
        ),
        "anchor_false_material_fraction": (
            None
            if not np.any(anchor)
            else float(np.mean(predicted[anchor] != 1))
        ),
        "class_support": {
            "left": int(np.count_nonzero(truth == 0)),
            "neutral": int(np.count_nonzero(truth == 1)),
            "right": int(np.count_nonzero(truth == 2)),
        },
    }


def infer_normal_sequence(
    model: FrozenTinyLidarRecurrentAdapter,
    sequence,
    device,
    correction_deadband_rad: float = 0.0,
) -> dict:
    model.eval()
    scans = torch.from_numpy(sequence.scans).unsqueeze(0).to(device)
    speeds = torch.from_numpy(sequence.speeds).view(1, -1, 1).to(device)
    with torch.no_grad():
        steering, correction, base, _ = model.forward_components(scans, speeds)
    steering_values = steering.squeeze(0).cpu().numpy()
    correction_values = correction.squeeze(0).cpu().numpy()
    base_values = base.squeeze(0).cpu().numpy()
    if correction_deadband_rad > 0.0:
        correction_values = np.where(
            np.abs(correction_values) >= correction_deadband_rad,
            correction_values,
            0.0,
        )
        steering_values = np.clip(
            base_values + correction_values, -1.0, 1.0
        )
    if not (
        steering_values.shape
        == correction_values.shape
        == base_values.shape
        == sequence.speeds.shape
    ):
        raise ValueError("invalid recurrent normal inference shape")
    if not all(
        np.all(np.isfinite(values))
        for values in (steering_values, correction_values, base_values)
    ):
        raise ValueError("non-finite recurrent normal inference")
    absolute = np.abs(correction_values)
    return {
        "samples": len(absolute),
        "correction_mae_rad": float(np.mean(absolute)),
        "correction_p95_rad": float(np.percentile(absolute, 95)),
        "correction_abs_max_rad": float(np.max(absolute)),
        "nonzero_correction_fraction": float(np.mean(absolute > 0.0)),
        "composed_identity_max_error_rad": float(
            np.max(
                np.abs(
                    steering_values
                    - np.clip(base_values + correction_values, -1.0, 1.0)
                )
            )
        ),
        "finite": True,
    }


def aggregate_normal_metrics(records: list[dict]) -> dict:
    samples = sum(record["metrics"]["samples"] for record in records)
    if samples <= 0:
        raise RuntimeError("normal recurrent evaluation produced no samples")
    weighted_mae = sum(
        record["metrics"]["correction_mae_rad"]
        * record["metrics"]["samples"]
        for record in records
    ) / samples
    return {
        "samples": samples,
        "correction_mae_rad": weighted_mae,
        "correction_p95_rad": max(
            record["metrics"]["correction_p95_rad"] for record in records
        ),
        "correction_abs_max_rad": max(
            record["metrics"]["correction_abs_max_rad"] for record in records
        ),
        "nonzero_correction_fraction": sum(
            record["metrics"]["nonzero_correction_fraction"]
            * record["metrics"]["samples"]
            for record in records
        ) / samples,
        "composed_identity_max_error_rad": max(
            record["metrics"]["composed_identity_max_error_rad"]
            for record in records
        ),
        "finite": all(record["metrics"]["finite"] for record in records),
    }


def assert_embedded_base_identity(
    model: FrozenTinyLidarRecurrentAdapter, checkpoint: Path
) -> dict:
    reference = TinyLidarNet(input_dim=model.input_dim, output_dim=2)
    provenance = load_pretrained_weights(reference, checkpoint)
    mismatches = [
        key
        for key, expected in reference.state_dict().items()
        if not torch.equal(model.base.state_dict()[key].cpu(), expected.cpu())
    ]
    if mismatches:
        raise ValueError(f"recurrent candidate changed frozen base: {mismatches}")
    return provenance


def assert_embedded_spatial_identity(
    model: FrozenTinyLidarRecurrentAdapter, checkpoint: Path
) -> dict:
    if (
        model.spatial_baseline is None
        or model.frozen_spatial_baseline_config is None
    ):
        raise ValueError("recurrent candidate has no frozen spatial baseline")
    reference = FrozenTinyLidarSpatialResidual(
        **model.frozen_spatial_baseline_config
    )
    provenance = load_pretrained_weights(reference, checkpoint)
    mismatches = [
        key
        for key, expected in reference.state_dict().items()
        if not torch.equal(
            model.spatial_baseline.state_dict()[key].cpu(), expected.cpu()
        )
    ]
    if mismatches:
        raise ValueError(
            f"recurrent candidate changed frozen spatial baseline: {mismatches}"
        )
    return provenance


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path)
    parser.add_argument("--production-spatial-checkpoint", type=Path)
    parser.add_argument("--val-dir", type=Path, required=True)
    parser.add_argument("--normal-recurrent-root", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--minimum-material-improvement", type=float, default=0.30)
    parser.add_argument("--maximum-anchor-mae-rad", type=float, default=0.03)
    parser.add_argument("--maximum-normal-mae-rad", type=float, default=0.01)
    parser.add_argument(
        "--correction-deadband-rad",
        type=float,
        default=0.0,
        help="Offline deployment decode: corrections below this magnitude become zero.",
    )
    parser.add_argument("--unseen-source-bag-token", default="seed2033")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not np.isfinite(args.correction_deadband_rad) or args.correction_deadband_rad < 0.0:
        raise ValueError("correction deadband must be finite and non-negative")
    checkpoint_path = args.checkpoint.expanduser().resolve()
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    if set(checkpoint) != {"model_config", "model_state_dict"}:
        raise ValueError("unexpected recurrent checkpoint contract")
    model_config = dict(checkpoint["model_config"])
    model_type = model_config.pop("model_type", "pressure_gru")
    model_class = (
        RecurrentDirectSteeringPolicy
        if model_type == "pressure_gru"
        else FrozenTinyLidarRecurrentAdapter
    )
    if model_type not in {"pressure_gru", "frozen_tinylidar_adapter"}:
        raise ValueError(f"unsupported recurrent model type: {model_type}")
    model = model_class(**model_config)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    base_provenance = None
    spatial_baseline_provenance = None
    if isinstance(model, FrozenTinyLidarRecurrentAdapter):
        if args.base_checkpoint is None:
            raise ValueError("frozen adapter evaluation requires --base-checkpoint")
        base_provenance = assert_embedded_base_identity(
            model, args.base_checkpoint.expanduser().resolve()
        )
        if model.spatial_baseline is not None:
            if args.production_spatial_checkpoint is None:
                raise ValueError(
                    "frozen production baseline requires --production-spatial-checkpoint"
                )
            spatial_baseline_provenance = assert_embedded_spatial_identity(
                model, args.production_spatial_checkpoint.expanduser().resolve()
            )
        if (
            model.spatial_features == "projected_conv5"
            and args.normal_recurrent_root is None
        ):
            raise ValueError(
                "projected conv5 evaluation requires --normal-recurrent-root"
            )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)

    validation = MultiSeqRecurrentPolicyDataset(
        args.val_dir, expected_split="val"
    )
    all_predictions = []
    all_targets = []
    all_base = []
    all_predicted_classes = []
    per_sequence = []
    unseen = None
    for sequence in validation.datasets:
        predictions, embedded_base, predicted_classes = infer_sequence(
            model, sequence, device, args.correction_deadband_rad
        )
        metrics = direct_policy_metrics(
            predictions,
            sequence.steers,
            embedded_base,
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
            "direction": (
                None
                if predicted_classes is None
                else direction_metrics(
                    predicted_classes,
                    sequence.steers,
                    embedded_base,
                    args.material_delta_rad,
                )
            ),
        }
        per_sequence.append(record)
        if args.unseen_source_bag_token in record["source_bag"]:
            if unseen is not None:
                raise ValueError("unseen source bag token is ambiguous")
            unseen = record
        all_predictions.append(predictions)
        all_targets.append(sequence.steers)
        all_base.append(embedded_base)
        if predicted_classes is not None:
            all_predicted_classes.append(predicted_classes)
    if unseen is None:
        raise ValueError("unseen source bag token did not match a validation run")

    aggregate = direct_policy_metrics(
        np.concatenate(all_predictions),
        np.concatenate(all_targets),
        np.concatenate(all_base),
        args.material_delta_rad,
    )
    aggregate_direction = (
        None
        if not all_predicted_classes
        else direction_metrics(
            np.concatenate(all_predicted_classes),
            np.concatenate(all_targets),
            np.concatenate(all_base),
            args.material_delta_rad,
        )
    )
    normal_records = []
    independent_normal = None
    if args.normal_recurrent_root is not None:
        if not isinstance(model, FrozenTinyLidarRecurrentAdapter):
            raise ValueError("normal recurrent gate requires a frozen adapter")
        normal = MultiSeqNormalAnchorDataset(
            args.normal_recurrent_root.expanduser().resolve() / "val", "val"
        )
        for sequence in normal.datasets:
            normal_records.append(
                {
                    "sequence_id": sequence.sequence_id,
                    "source_bag": sequence.metadata["source"]["bag"],
                    "metrics": infer_normal_sequence(
                        model,
                        sequence,
                        device,
                        args.correction_deadband_rad,
                    ),
                }
            )
        independent_normal = aggregate_normal_metrics(normal_records)
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
    if isinstance(model, FrozenTinyLidarRecurrentAdapter):
        gates["embedded_base_identity"] = True
        if model.spatial_baseline is not None:
            gates["embedded_spatial_baseline_identity"] = True
    if independent_normal is not None:
        gates["independent_normal_leakage"] = (
            independent_normal["correction_mae_rad"]
            <= args.maximum_normal_mae_rad
        )
        gates["independent_normal_finite"] = independent_normal["finite"]
    report = {
        "schema_version": 1,
        "checkpoint": str(checkpoint_path),
        "checkpoint_sha256": sha256_file(checkpoint_path),
        "model_config": checkpoint["model_config"],
        "base_checkpoint": base_provenance,
        "production_spatial_checkpoint": spatial_baseline_provenance,
        "dataset": str(args.val_dir.expanduser().resolve()),
        "aggregate": aggregate,
        "aggregate_direction": aggregate_direction,
        "per_sequence": per_sequence,
        "unseen_sequence_id": unseen["sequence_id"],
        "independent_normal": independent_normal,
        "normal_sequences": normal_records,
        "gate_thresholds": {
            "material_delta_rad": args.material_delta_rad,
            "minimum_material_improvement": args.minimum_material_improvement,
            "maximum_anchor_mae_rad": args.maximum_anchor_mae_rad,
            "maximum_normal_mae_rad": args.maximum_normal_mae_rad,
            "correction_deadband_rad": args.correction_deadband_rad,
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
