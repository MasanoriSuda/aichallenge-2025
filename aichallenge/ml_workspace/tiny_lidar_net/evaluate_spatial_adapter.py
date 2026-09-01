#!/usr/bin/env python3
"""Gate a frozen-base spatial correction before any runtime integration."""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader, Subset

from lib.checkpoint import load_pretrained_weights
from lib.data import MultiSeqConcatDataset
from lib.model import TinyLidarNet
from lib.normal_anchor import MultiSeqNormalAnchorDataset
from lib.recurrent_policy import MultiSeqRecurrentPolicyDataset
from lib.residual import residual_metrics, write_json
from lib.spatial_adapter import (
    SPATIAL_HEAD_ARCHITECTURES,
    SPATIAL_NORMALIZATION_MODES,
    FrozenTinyLidarSpatialResidual,
)


SPATIAL_DECODE_MODES = ("soft_mixture", "winner_take_all")


def decode_spatial_components(
    residual: torch.Tensor,
    magnitudes: torch.Tensor,
    direction_logits: torch.Tensor,
    decode_mode: str,
) -> torch.Tensor:
    """Decode one trained mixture without changing its learned parameters."""
    if decode_mode not in SPATIAL_DECODE_MODES:
        raise ValueError(f"unsupported spatial decode mode: {decode_mode}")
    if decode_mode == "soft_mixture":
        return residual
    if magnitudes.ndim != 2 or magnitudes.shape[1] != 2:
        raise ValueError("spatial magnitudes must have left and right columns")
    if direction_logits.shape != (len(magnitudes), 3):
        raise ValueError("spatial direction logits must have three classes")
    direction = torch.argmax(direction_logits, dim=1)
    decoded = torch.zeros_like(residual)
    decoded = torch.where(direction == 0, -magnitudes[:, 0], decoded)
    return torch.where(direction == 2, magnitudes[:, 1], decoded)


def runtime_bounded_metrics(
    predicted: np.ndarray,
    target: np.ndarray,
    material_delta_rad: float,
    authority_bound_rad: float,
) -> dict:
    """Evaluate the exact residual that runtime authority can publish."""
    if not np.isfinite(authority_bound_rad) or authority_bound_rad <= 0.0:
        raise ValueError("runtime authority bound must be finite and positive")
    values = np.asarray(predicted, dtype=np.float64)
    targets = np.asarray(target, dtype=np.float64)
    if values.shape != targets.shape or values.ndim != 1:
        raise ValueError("bounded residual arrays must be aligned and one-dimensional")
    bounded = np.clip(values, -authority_bound_rad, authority_bound_rad)
    material = np.abs(targets) >= material_delta_rad
    material_targets = targets[material]
    zero_mae = (
        None if not np.any(material) else float(np.mean(np.abs(material_targets)))
    )
    oracle_mae = (
        None
        if not np.any(material)
        else float(
            np.mean(
                np.abs(
                    np.clip(
                        material_targets,
                        -authority_bound_rad,
                        authority_bound_rad,
                    )
                    - material_targets
                )
            )
        )
    )
    bounded_residual = residual_metrics(bounded, targets, material_delta_rad)
    achieved_improvement = (
        None
        if zero_mae is None
        else zero_mae - bounded_residual["material"]["mae_rad"]
    )
    attainable_improvement = (
        None if zero_mae is None else zero_mae - oracle_mae
    )
    return {
        "authority_bound_rad": authority_bound_rad,
        "clipped_fraction": float(np.mean(np.abs(values) > authority_bound_rad)),
        "residual": bounded_residual,
        "material_sign_accuracy": (
            None
            if not np.any(material)
            else float(
                np.mean(np.sign(bounded[material]) == np.sign(targets[material]))
            )
        ),
        "oracle": {
            "material_mae_rad": oracle_mae,
            "maximum_material_mae_improvement_fraction": (
                None
                if zero_mae is None or zero_mae <= 0.0
                else attainable_improvement / zero_mae
            ),
            "attainable_improvement_utilization": (
                None
                if attainable_improvement is None or attainable_improvement <= 0.0
                else achieved_improvement / attainable_improvement
            ),
        },
    }


def predict_paired(
    model,
    loader,
    device,
    material_delta_rad: float,
    runtime_authority_bound_rad: float | None = None,
    decode_mode: str = "soft_mixture",
) -> dict:
    residuals = []
    targets = []
    directions = []
    model.eval()
    with torch.no_grad():
        for scans, speeds, teacher, base in loader:
            residual, magnitudes, logits, _ = model.forward_components(
                scans.to(device), speeds.to(device)
            )
            residuals.append(
                decode_spatial_components(
                    residual, magnitudes, logits, decode_mode
                ).cpu().numpy()
            )
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
    report = {
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
    if runtime_authority_bound_rad is not None:
        report["runtime_bounded"] = runtime_bounded_metrics(
            predicted,
            target,
            material_delta_rad,
            runtime_authority_bound_rad,
        )
    return report


def predict_normal(
    model, loader, device, max_range_m: float, decode_mode: str
) -> dict:
    values = []
    model.eval()
    with torch.no_grad():
        for normalized_scans, _ in loader:
            physical_scans = normalized_scans.to(device) * max_range_m
            residual, magnitudes, logits, _ = model.forward_components(
                physical_scans
            )
            values.append(
                decode_spatial_components(
                    residual, magnitudes, logits, decode_mode
                ).cpu().numpy()
            )
    return normal_metrics(np.concatenate(values))


def predict_recurrent_normal(model, loader, device, decode_mode: str) -> dict:
    values = []
    model.eval()
    with torch.no_grad():
        for scans, speeds, _, _ in loader:
            residual, magnitudes, logits, _ = model.forward_components(
                scans.to(device), speeds.to(device)
            )
            values.append(
                decode_spatial_components(
                    residual, magnitudes, logits, decode_mode
                ).cpu().numpy()
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


def select_unique_source_sequence(sequences, source_token: str):
    """Select one immutable run by source-bag token without split leakage."""
    if not source_token:
        return None
    matches = [
        sequence
        for sequence in sequences
        if source_token
        in str(sequence.metadata.get("source", {}).get("bag", ""))
    ]
    if len(matches) != 1:
        raise ValueError(
            f"source token {source_token!r} matched {len(matches)} sequences"
        )
    return matches[0]


def causal_tail(sequence, tail_samples: int):
    """Return a non-empty causal suffix for failure-local evaluation."""
    if tail_samples <= 0:
        raise ValueError("tail_samples must be positive")
    start = max(0, len(sequence) - tail_samples)
    return Subset(sequence, range(start, len(sequence)))


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
    parser.add_argument("--use-base-steering", action="store_true")
    parser.add_argument(
        "--spatial-normalization",
        choices=SPATIAL_NORMALIZATION_MODES,
        default="layer_norm",
    )
    parser.add_argument("--projection-dim", type=int, default=0)
    parser.add_argument("--projection-seed", type=int, default=2026)
    parser.add_argument(
        "--head-architecture",
        choices=SPATIAL_HEAD_ARCHITECTURES,
        default="signed_mixture",
    )
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument(
        "--runtime-authority-bound-rad",
        type=float,
        default=0.0,
        help=(
            "If positive, report and gate the residual after the exact symmetric "
            "clip used by runtime spatial authority."
        ),
    )
    parser.add_argument(
        "--runtime-decode-mode",
        choices=SPATIAL_DECODE_MODES,
        default="soft_mixture",
        help=(
            "Exact runtime decoding contract to gate. winner_take_all emits "
            "zero for neutral and the selected side magnitude otherwise."
        ),
    )
    parser.add_argument("--peer-validation-token", default="20260901-153143/d3")
    parser.add_argument(
        "--focus-validation-token",
        default="",
        help="Optional held-out source-bag token reported separately.",
    )
    parser.add_argument(
        "--tail-samples",
        type=int,
        default=200,
        help="Causal suffix length for the focused validation run.",
    )
    parser.add_argument("--batch-size", type=int, default=512)
    parser.add_argument("--min-material-improvement", type=float, default=0.30)
    parser.add_argument("--min-material-sign-accuracy", type=float, default=0.80)
    parser.add_argument("--max-anchor-mae-rad", type=float, default=0.01)
    parser.add_argument("--max-normal-mae-rad", type=float, default=0.01)
    parser.add_argument("--fail-on-gate", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.batch_size <= 0 or args.tail_samples <= 0:
        raise ValueError("spatial evaluation batch and tail sizes must be positive")
    if (
        not np.isfinite(args.runtime_authority_bound_rad)
        or args.runtime_authority_bound_rad < 0.0
    ):
        raise ValueError("runtime authority bound must be finite and non-negative")
    runtime_bound = (
        args.runtime_authority_bound_rad
        if args.runtime_authority_bound_rad > 0.0
        else None
    )
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
        use_base_steering=args.use_base_steering,
        max_speed_mps=args.max_speed_mps,
        spatial_normalization=args.spatial_normalization,
        projection_dim=args.projection_dim,
        projection_seed=args.projection_seed,
        head_architecture=args.head_architecture,
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
        model,
        validation_loader,
        device,
        args.material_delta_rad,
        runtime_bound,
        args.runtime_decode_mode,
    )
    per_sequence = [
        {
            "sequence_id": sequence.sequence_id,
            "source_bag": str(sequence.metadata["source"]["bag"]),
            "metrics": predict_paired(
                model,
                DataLoader(
                    sequence, batch_size=args.batch_size, shuffle=False
                ),
                device,
                args.material_delta_rad,
                runtime_bound,
                args.runtime_decode_mode,
            ),
        }
        for sequence in validation.datasets
    ]
    peer_record = select_unique_source_sequence(
        validation.datasets, args.peer_validation_token
    )
    peer = predict_paired(
        model,
        DataLoader(peer_record, batch_size=args.batch_size, shuffle=False),
        device,
        args.material_delta_rad,
        runtime_bound,
        args.runtime_decode_mode,
    )
    focus_record = select_unique_source_sequence(
        validation.datasets, args.focus_validation_token
    )
    focus = None
    if focus_record is not None:
        focus = {
            "sequence_id": focus_record.sequence_id,
            "source_bag": str(focus_record.metadata["source"]["bag"]),
            "full": predict_paired(
                model,
                DataLoader(
                    focus_record, batch_size=args.batch_size, shuffle=False
                ),
                device,
                args.material_delta_rad,
                runtime_bound,
                args.runtime_decode_mode,
            ),
            "tail_samples": min(args.tail_samples, len(focus_record)),
            "tail": predict_paired(
                model,
                DataLoader(
                    causal_tail(focus_record, args.tail_samples),
                    batch_size=args.batch_size,
                    shuffle=False,
                ),
                device,
                args.material_delta_rad,
                runtime_bound,
                args.runtime_decode_mode,
            ),
        }
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
            args.runtime_decode_mode,
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
            args.runtime_decode_mode,
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
    if runtime_bound is not None:
        gates["runtime_bounded_material_improvement"] = (
            aggregate["runtime_bounded"]["residual"][
                "material_mae_improvement_fraction"
            ]
            >= args.min_material_improvement
        )
    report = {
        "schema_version": 1,
        "purpose": "offline gate; no runtime authority",
        "candidate": candidate_provenance,
        "base_checkpoint": base_provenance,
        "validation_sequence_ids": validation.sequence_ids,
        "validation_sequences": per_sequence,
        "peer_sequence_id": peer_record.sequence_id,
        "aggregate": aggregate,
        "peer_validation": peer,
        "focus_validation": focus,
        "independent_normal": independent_normal,
        "thresholds": {
            "min_material_improvement": args.min_material_improvement,
            "min_material_sign_accuracy": args.min_material_sign_accuracy,
            "max_anchor_mae_rad": args.max_anchor_mae_rad,
            "max_normal_mae_rad": args.max_normal_mae_rad,
            "runtime_authority_bound_rad": runtime_bound,
            "runtime_decode_mode": args.runtime_decode_mode,
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
