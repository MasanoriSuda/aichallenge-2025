#!/usr/bin/env python3
"""Train a speed-conditioned recurrent direct steering policy offline."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import random
from typing import Optional

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader, WeightedRandomSampler

from lib.checkpoint import load_pretrained_weights
from lib.normal_anchor import MultiSeqNormalAnchorDataset
from lib.recurrent_policy import (
    FrozenTinyLidarRecurrentAdapter,
    MultiSeqRecurrentPolicyDataset,
    RecurrentDirectSteeringPolicy,
    recurrent_chunk_dataset,
    weighted_direct_policy_smooth_l1,
)
from lib.residual import signed_direction_targets, signed_expert_training_loss


def seed_everything(seed: int) -> torch.Generator:
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.benchmark = False
    torch.backends.cudnn.deterministic = True
    generator = torch.Generator()
    generator.manual_seed(seed)
    return generator


def sequence_balanced_chunk_weights(chunk_dataset) -> torch.Tensor:
    """Assign equal total sampling mass to every source run."""
    weights = []
    for sequence_chunks in chunk_dataset.datasets:
        count = len(sequence_chunks)
        if count <= 0:
            raise ValueError("recurrent sequence produced no chunks")
        weights.extend([1.0 / count] * count)
    values = torch.tensor(weights, dtype=torch.float64)
    return values / torch.sum(values)


def outcome_run_balanced_chunk_weights(
    chunk_dataset,
) -> tuple[torch.Tensor, dict[str, int]]:
    """Assign equal total sampling mass to each certified teacher run."""
    run_ids = []
    chunk_counts: dict[str, int] = {}
    for sequence_chunks in chunk_dataset.datasets:
        certificate = sequence_chunks.sequence.metadata.get("outcome_certificate")
        run_id = (
            None
            if not isinstance(certificate, dict)
            else certificate.get("source_run_id")
        )
        if not isinstance(run_id, str) or not run_id.strip():
            raise ValueError(
                "outcome-run balanced sampling requires certified source_run_id"
            )
        count = len(sequence_chunks)
        if count <= 0:
            raise ValueError("recurrent sequence produced no chunks")
        run_ids.append((run_id, count))
        chunk_counts[run_id] = chunk_counts.get(run_id, 0) + count

    weights = []
    for run_id, count in run_ids:
        weights.extend([1.0 / chunk_counts[run_id]] * count)
    values = torch.tensor(weights, dtype=torch.float64)
    return values / torch.sum(values), dict(sorted(chunk_counts.items()))


def batch_loss(model, batch, device, args) -> torch.Tensor:
    scans, speeds, targets, base = batch
    scans = scans.to(device)
    speeds = speeds.to(device)
    if isinstance(model, FrozenTinyLidarRecurrentAdapter):
        (
            predictions,
            _,
            embedded_base,
            magnitudes,
            direction_logits,
            _,
        ) = model.forward_correction_components(scans, speeds)
        base = embedded_base
        if model.correction_head == "signed_expert":
            correction_targets = targets.to(device) - embedded_base
            correction_targets = correction_targets[:, args.burn_in_steps:].reshape(-1)
            return signed_expert_training_loss(
                magnitudes[:, args.burn_in_steps:].reshape(-1, 2),
                direction_logits[:, args.burn_in_steps:].reshape(-1, 3),
                correction_targets,
                args.material_delta_rad,
                args.direction_class_weights,
                args.direction_loss_weight,
            )
    else:
        predictions, _ = model(scans, speeds)
        base = base.to(device)
    return weighted_direct_policy_smooth_l1(
        predictions,
        targets.to(device),
        base,
        material_delta_rad=args.material_delta_rad,
        material_weight=args.material_weight,
        burn_in_steps=args.burn_in_steps,
    )


def base_distillation_loss(model, batch, device, args) -> torch.Tensor:
    scans, speeds, _, base = batch
    predictions, _ = model(scans.to(device), speeds.to(device))
    predictions = predictions[:, args.burn_in_steps:]
    targets = base.to(device)[:, args.burn_in_steps:]
    return torch.nn.functional.smooth_l1_loss(predictions, targets)


def normal_anchor_loss(model, batch, device, args) -> torch.Tensor:
    scans, speeds, _, _ = batch
    if not isinstance(model, FrozenTinyLidarRecurrentAdapter):
        raise TypeError("normal recurrent anchors require a frozen adapter")
    _, corrections, _, magnitudes, direction_logits, _ = (
        model.forward_correction_components(
            scans.to(device), speeds.to(device)
        )
    )
    if model.correction_head == "signed_expert":
        logits = direction_logits[:, args.burn_in_steps:].reshape(-1, 3)
        neutral = torch.ones(logits.shape[0], dtype=torch.long, device=device)
        return args.direction_loss_weight * torch.nn.functional.cross_entropy(
            logits, neutral
        )
    corrections = corrections[:, args.burn_in_steps:]
    return torch.nn.functional.smooth_l1_loss(
        corrections, torch.zeros_like(corrections)
    )


def recurrent_direction_class_weights(
    model: FrozenTinyLidarRecurrentAdapter,
    sequences,
    device: torch.device,
    material_delta_rad: float,
) -> torch.Tensor:
    counts = torch.zeros(3, dtype=torch.float64)
    model.eval()
    with torch.no_grad():
        for sequence in sequences:
            scans = torch.from_numpy(sequence.scans).unsqueeze(0).to(device)
            speeds = torch.from_numpy(sequence.speeds).view(1, -1, 1).to(device)
            base = model.base_steering(scans, speeds).squeeze(0)
            targets = torch.from_numpy(sequence.steers).to(device) - base
            classes = signed_direction_targets(targets, material_delta_rad)
            counts += torch.bincount(classes.cpu(), minlength=3).to(torch.float64)
    if torch.any(counts <= 0):
        raise ValueError(f"recurrent train split lacks a direction: {counts.tolist()}")
    inverse = torch.sum(counts) / (3.0 * counts)
    return (inverse / torch.mean(inverse)).to(dtype=torch.float32, device=device)


def evaluate_objective(model, loader, device, args) -> float:
    model.eval()
    weighted_total = 0.0
    samples = 0
    with torch.no_grad():
        for batch in loader:
            loss = batch_loss(model, batch, device, args)
            batch_size = int(batch[0].shape[0])
            weighted_total += float(loss.item()) * batch_size
            samples += batch_size
    if samples == 0:
        raise RuntimeError("recurrent validation produced no chunks")
    return weighted_total / samples


def evaluate_distillation_objective(model, loader, device, args) -> float:
    model.eval()
    weighted_total = 0.0
    samples = 0
    with torch.no_grad():
        for batch in loader:
            loss = base_distillation_loss(model, batch, device, args)
            batch_size = int(batch[0].shape[0])
            weighted_total += float(loss.item()) * batch_size
            samples += batch_size
    if samples == 0:
        raise RuntimeError("recurrent validation produced no chunks")
    return weighted_total / samples


def evaluate_normal_objective(model, loader, device, args) -> float:
    model.eval()
    weighted_total = 0.0
    samples = 0
    with torch.no_grad():
        for batch in loader:
            loss = normal_anchor_loss(model, batch, device, args)
            batch_size = int(batch[0].shape[0])
            weighted_total += float(loss.item()) * batch_size
            samples += batch_size
    if samples == 0:
        raise RuntimeError("normal recurrent validation produced no chunks")
    return weighted_total / samples


def fit_projected_spatial_statistics(
    model: FrozenTinyLidarRecurrentAdapter,
    sequences,
    device: torch.device,
    batch_size: int,
) -> dict:
    """Fit per-feature statistics from training runs only."""
    loader = DataLoader(
        ConcatDataset(list(sequences)),
        batch_size=batch_size,
        shuffle=False,
        num_workers=0,
    )
    count = 0
    total = None
    squared_total = None
    model.eval()
    with torch.no_grad():
        for scans, _, _, _ in loader:
            projected = model.projected_spatial_features(scans.to(device)).to(
                torch.float64
            )
            batch_total = torch.sum(projected, dim=0)
            batch_squared = torch.sum(projected * projected, dim=0)
            total = batch_total if total is None else total + batch_total
            squared_total = (
                batch_squared
                if squared_total is None
                else squared_total + batch_squared
            )
            count += len(projected)
    if count == 0 or total is None or squared_total is None:
        raise RuntimeError("spatial statistics source is empty")
    mean = total / count
    variance = torch.clamp(squared_total / count - mean * mean, min=0.0)
    scale = torch.clamp(torch.sqrt(variance), min=1e-4)
    model.set_spatial_statistics(mean.to(torch.float32), scale.to(torch.float32))
    return {
        "samples": count,
        "minimum_scale": float(torch.min(scale).item()),
        "maximum_scale": float(torch.max(scale).item()),
        "mean_abs": float(torch.mean(torch.abs(mean)).item()),
    }


def combined_validation_objective(
    model,
    successor_loader,
    normal_loader: Optional[DataLoader],
    device,
    args,
) -> tuple[float, float, Optional[float]]:
    successor = evaluate_objective(model, successor_loader, device, args)
    normal = (
        None
        if normal_loader is None
        else evaluate_normal_objective(model, normal_loader, device, args)
    )
    combined = successor + (
        0.0 if normal is None else args.normal_anchor_weight * normal
    )
    return combined, successor, normal


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-dir", type=Path, required=True)
    parser.add_argument("--val-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--model-type",
        choices=("pressure_gru", "frozen_tinylidar_adapter"),
        default="pressure_gru",
    )
    parser.add_argument("--base-checkpoint", type=Path)
    parser.add_argument(
        "--production-spatial-checkpoint",
        type=Path,
        help=(
            "Frozen packaged spatial adapter to preserve as the recurrent "
            "candidate's exact zero-correction baseline."
        ),
    )
    parser.add_argument(
        "--normal-recurrent-root",
        type=Path,
        help=(
            "Independent production-normal root. Its train split supplies exact "
            "zero-correction anchors and its val split participates in model selection."
        ),
    )
    parser.add_argument(
        "--adapter-pressure-tokens",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--adapter-spatial-features",
        choices=("compact_fc3", "projected_conv5"),
        default="compact_fc3",
    )
    parser.add_argument("--adapter-spatial-projection-dim", type=int, default=128)
    parser.add_argument("--adapter-spatial-projection-seed", type=int, default=2026)
    parser.add_argument(
        "--adapter-spatial-normalization",
        choices=("none", "fixed_train_statistics"),
        default="none",
    )
    parser.add_argument(
        "--adapter-use-speed",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--adapter-correction-head",
        choices=("direct", "signed_expert"),
        default="direct",
    )
    parser.add_argument("--input-dim", type=int, default=750)
    parser.add_argument("--speed-embedding-dim", type=int, default=64)
    parser.add_argument("--hidden-dim", type=int, default=512)
    parser.add_argument("--max-speed-mps", type=float, default=12.0)
    parser.add_argument(
        "--max-speed-sync-delta-sec",
        type=float,
        default=0.05,
        help=(
            "Maximum causal speed age admitted by successor datasets. The "
            "default preserves the original 50 ms dataset contract; a looser "
            "runtime contract must be selected explicitly and is recorded in "
            "the training manifest."
        ),
    )
    parser.add_argument("--max-abs-steering-rad", type=float, default=1.0)
    parser.add_argument("--chunk-length", type=int, default=64)
    parser.add_argument("--chunk-stride", type=int, default=32)
    parser.add_argument("--burn-in-steps", type=int, default=8)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--material-weight", type=float, default=2.0)
    parser.add_argument("--normal-anchor-weight", type=float, default=1.0)
    parser.add_argument("--direction-loss-weight", type=float, default=1.0)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--distillation-epochs", type=int, default=25)
    parser.add_argument("--learning-rate", type=float, default=2e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-5)
    parser.add_argument("--gradient-clip-norm", type=float, default=1.0)
    parser.add_argument("--early-stop-patience", type=int, default=7)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=2026)
    sampling_group = parser.add_mutually_exclusive_group()
    sampling_group.add_argument(
        "--sequence-balanced-successor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Give every source run equal successor fine-tuning mass. This is "
            "off by default because failure prefixes have unequal durations."
        ),
    )
    sampling_group.add_argument(
        "--outcome-run-balanced-successor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Give each immutable executed-teacher source_run_id equal total "
            "sampling mass while preserving within-run chunk frequency."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if (
        args.chunk_length <= 1
        or args.chunk_stride <= 0
        or not 0 <= args.burn_in_steps < args.chunk_length
        or args.batch_size <= 0
        or args.epochs <= 0
        or args.distillation_epochs < 0
        or args.num_workers < 0
        or args.learning_rate <= 0.0
        or args.weight_decay < 0.0
        or args.gradient_clip_norm <= 0.0
        or args.early_stop_patience <= 0
        or args.normal_anchor_weight < 0.0
        or args.direction_loss_weight <= 0.0
        or not np.isfinite(args.max_speed_sync_delta_sec)
        or args.max_speed_sync_delta_sec <= 0.0
    ):
        raise ValueError("invalid recurrent training configuration")

    generator = seed_everything(args.seed)
    train_sequences = MultiSeqRecurrentPolicyDataset(
        args.train_dir,
        expected_split="train",
        max_speed_sync_delta_sec=args.max_speed_sync_delta_sec,
    )
    val_sequences = MultiSeqRecurrentPolicyDataset(
        args.val_dir,
        expected_split="val",
        max_speed_sync_delta_sec=args.max_speed_sync_delta_sec,
    )
    overlap = set(train_sequences.sequence_ids) & set(val_sequences.sequence_ids)
    if overlap:
        raise ValueError(f"train/validation recurrent identity overlap: {overlap}")
    train_chunks = recurrent_chunk_dataset(
        train_sequences, args.chunk_length, args.chunk_stride
    )
    val_chunks = recurrent_chunk_dataset(
        val_sequences, args.chunk_length, args.chunk_length
    )
    normal_train_sequences = None
    normal_val_sequences = None
    normal_train_loader = None
    normal_val_loader = None
    if args.normal_recurrent_root is not None:
        normal_root = args.normal_recurrent_root.expanduser().resolve()
        normal_train_sequences = MultiSeqNormalAnchorDataset(
            normal_root / "train", "train"
        )
        normal_val_sequences = MultiSeqNormalAnchorDataset(
            normal_root / "val", "val"
        )
        all_successor_ids = set(
            train_sequences.sequence_ids + val_sequences.sequence_ids
        )
        all_normal_ids = set(
            normal_train_sequences.sequence_ids + normal_val_sequences.sequence_ids
        )
        identity_overlap = all_successor_ids & all_normal_ids
        if identity_overlap:
            raise ValueError(
                f"successor/normal recurrent identity overlap: {identity_overlap}"
            )
        normal_train_chunks = recurrent_chunk_dataset(
            normal_train_sequences, args.chunk_length, args.chunk_stride
        )
        normal_val_chunks = recurrent_chunk_dataset(
            normal_val_sequences, args.chunk_length, args.chunk_length
        )
        normal_train_loader = DataLoader(
            normal_train_chunks,
            batch_size=args.batch_size,
            shuffle=True,
            num_workers=args.num_workers,
            generator=generator,
            pin_memory=torch.cuda.is_available(),
        )
        normal_val_loader = DataLoader(
            normal_val_chunks,
            batch_size=args.batch_size,
            shuffle=False,
            num_workers=args.num_workers,
            pin_memory=torch.cuda.is_available(),
        )
    successor_sampler = None
    successor_sampling = {"mode": "natural", "outcome_run_chunk_counts": {}}
    if args.sequence_balanced_successor:
        successor_sampler = WeightedRandomSampler(
            sequence_balanced_chunk_weights(train_chunks),
            num_samples=len(train_chunks),
            replacement=True,
            generator=generator,
        )
        successor_sampling["mode"] = "sequence_balanced"
    elif args.outcome_run_balanced_successor:
        weights, run_chunk_counts = outcome_run_balanced_chunk_weights(train_chunks)
        successor_sampler = WeightedRandomSampler(
            weights,
            num_samples=len(train_chunks),
            replacement=True,
            generator=generator,
        )
        successor_sampling = {
            "mode": "outcome_run_balanced",
            "outcome_run_chunk_counts": run_chunk_counts,
        }
    distillation_loader = DataLoader(
        train_chunks,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        generator=generator,
        pin_memory=torch.cuda.is_available(),
    )
    train_loader = DataLoader(
        train_chunks,
        batch_size=args.batch_size,
        shuffle=successor_sampler is None,
        sampler=successor_sampler,
        num_workers=args.num_workers,
        generator=generator,
        pin_memory=torch.cuda.is_available(),
    )
    val_loader = DataLoader(
        val_chunks,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available(),
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if args.model_type == "pressure_gru":
        model_config = {
            "model_type": args.model_type,
            "input_dim": args.input_dim,
            "speed_embedding_dim": args.speed_embedding_dim,
            "hidden_dim": args.hidden_dim,
            "max_speed_mps": args.max_speed_mps,
            "max_abs_steering_rad": args.max_abs_steering_rad,
        }
        model = RecurrentDirectSteeringPolicy(
            **{
                key: value
                for key, value in model_config.items()
                if key != "model_type"
            }
        ).to(device)
        base_provenance = None
        if args.distillation_epochs <= 0:
            raise ValueError("pressure_gru requires positive distillation epochs")
    else:
        if args.base_checkpoint is None:
            raise ValueError("frozen adapter requires --base-checkpoint")
        if (
            args.adapter_spatial_features == "projected_conv5"
            and (
                args.normal_recurrent_root is None
                or args.production_spatial_checkpoint is None
            )
        ):
            raise ValueError(
                "projected conv5 adapter requires normal data and frozen production spatial checkpoint"
            )
        spatial_baseline_config = None
        if args.production_spatial_checkpoint is not None:
            spatial_baseline_config = {
                "input_dim": args.input_dim,
                "hidden_dim": 128,
                "max_scan_range_m": 30.0,
                "max_abs_delta_rad": 1.2,
                "use_speed": True,
                "use_base_steering": True,
                "max_speed_mps": 12.0,
                "spatial_normalization": "fixed_train_statistics",
                "projection_dim": 128,
                "projection_seed": 2026,
                "head_architecture": "signed_mixture",
            }
        model_config = {
            "model_type": args.model_type,
            "input_dim": args.input_dim,
            "speed_embedding_dim": args.speed_embedding_dim,
            "hidden_dim": args.hidden_dim,
            "max_scan_range_m": 30.0,
            "max_speed_mps": args.max_speed_mps,
            "max_abs_correction_rad": 0.64,
            "max_abs_steering_rad": args.max_abs_steering_rad,
            "include_pressure_tokens": args.adapter_pressure_tokens,
            "spatial_features": args.adapter_spatial_features,
            "spatial_projection_dim": args.adapter_spatial_projection_dim,
            "spatial_projection_seed": args.adapter_spatial_projection_seed,
            "spatial_normalization": args.adapter_spatial_normalization,
            "use_speed": args.adapter_use_speed,
            "frozen_spatial_baseline_config": spatial_baseline_config,
            "correction_head": args.adapter_correction_head,
        }
        model = FrozenTinyLidarRecurrentAdapter(
            **{
                key: value
                for key, value in model_config.items()
                if key != "model_type"
            }
        )
        base_provenance = load_pretrained_weights(model.base, args.base_checkpoint)
        spatial_baseline_provenance = None
        if args.production_spatial_checkpoint is not None:
            spatial_baseline_provenance = load_pretrained_weights(
                model.spatial_baseline, args.production_spatial_checkpoint
            )
            mismatches = [
                key
                for key, expected in model.base.state_dict().items()
                if not torch.equal(
                    model.spatial_baseline.base.state_dict()[key], expected
                )
            ]
            if mismatches:
                raise ValueError(
                    f"production spatial baseline embeds a different TinyLidarNet: {mismatches}"
                )
        model.to(device)
        if args.adapter_correction_head == "signed_expert":
            args.direction_class_weights = recurrent_direction_class_weights(
                model,
                train_sequences.datasets,
                device,
                args.material_delta_rad,
            )
        else:
            args.direction_class_weights = None
        spatial_statistics = None
        if args.adapter_spatial_features == "projected_conv5":
            if args.adapter_spatial_normalization != "fixed_train_statistics":
                raise ValueError(
                    "projected conv5 adapter requires fixed_train_statistics"
                )
            spatial_statistics = fit_projected_spatial_statistics(
                model,
                [
                    *train_sequences.datasets,
                    *normal_train_sequences.datasets,
                ],
                device,
                args.batch_size,
            )
        if args.distillation_epochs != 0:
            raise ValueError("frozen adapter must use --distillation-epochs 0")
    if args.model_type == "pressure_gru":
        spatial_statistics = None
        spatial_baseline_provenance = None
        args.direction_class_weights = None
    output_dir = (
        args.output_root.expanduser().resolve()
        / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    output_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        "schema_version": 1,
        "model": type(model).__name__,
        "model_config": model_config,
        "device": str(device),
        "base_checkpoint": base_provenance,
        "production_spatial_checkpoint": spatial_baseline_provenance,
        "spatial_feature_statistics": spatial_statistics,
        "direction_class_weights": (
            None
            if args.direction_class_weights is None
            else args.direction_class_weights.detach().cpu().tolist()
        ),
        "train_sequence_ids": train_sequences.sequence_ids,
        "val_sequence_ids": val_sequences.sequence_ids,
        "train_chunks": len(train_chunks),
        "val_chunks": len(val_chunks),
        "normal_train_sequence_ids": (
            [] if normal_train_sequences is None else normal_train_sequences.sequence_ids
        ),
        "normal_val_sequence_ids": (
            [] if normal_val_sequences is None else normal_val_sequences.sequence_ids
        ),
        "successor_sampling": successor_sampling,
        "config": {
            key: (
                str(value)
                if isinstance(value, Path)
                else value.detach().cpu().tolist()
                if isinstance(value, torch.Tensor)
                else value
            )
            for key, value in vars(args).items()
        },
    }
    (output_dir / "training-manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    distillation_history = []
    best_distillation_loss = None
    if args.distillation_epochs > 0:
        distilled_path = output_dir / "distilled_model.pth"
        distillation_optimizer = torch.optim.AdamW(
            (parameter for parameter in model.parameters() if parameter.requires_grad),
            lr=args.learning_rate,
            weight_decay=args.weight_decay,
        )
        best_distillation_loss = evaluate_distillation_objective(
            model, val_loader, device, args
        )
        torch.save(
            {"model_config": model_config, "model_state_dict": model.state_dict()},
            distilled_path,
        )
        for epoch in range(args.distillation_epochs):
            model.train()
            total_loss = 0.0
            batches = 0
            for batch in distillation_loader:
                distillation_optimizer.zero_grad(set_to_none=True)
                loss = base_distillation_loss(model, batch, device, args)
                if not torch.isfinite(loss):
                    raise FloatingPointError(
                        f"non-finite distillation loss at epoch {epoch}"
                    )
                loss.backward()
                torch.nn.utils.clip_grad_norm_(
                    (
                        parameter
                        for parameter in model.parameters()
                        if parameter.requires_grad
                    ),
                    args.gradient_clip_norm,
                )
                distillation_optimizer.step()
                total_loss += float(loss.item())
                batches += 1
            train_loss = total_loss / max(batches, 1)
            val_loss = evaluate_distillation_objective(model, val_loader, device, args)
            distillation_history.append(
                {"epoch": epoch, "train_loss": train_loss, "val_loss": val_loss}
            )
            print(
                f"distill_epoch={epoch:03d} train={train_loss:.8f} "
                f"val={val_loss:.8f} best={best_distillation_loss:.8f}"
            )
            if val_loss < best_distillation_loss - 1e-10:
                best_distillation_loss = val_loss
                torch.save(
                    {
                        "model_config": model_config,
                        "model_state_dict": model.state_dict(),
                    },
                    distilled_path,
                )
        distilled = torch.load(distilled_path, map_location=device, weights_only=True)
        model.load_state_dict(distilled["model_state_dict"], strict=True)

    optimizer = torch.optim.AdamW(
        (parameter for parameter in model.parameters() if parameter.requires_grad),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )
    best_path = output_dir / "best_model.pth"
    best_loss, best_successor_loss, best_normal_loss = combined_validation_objective(
        model, val_loader, normal_val_loader, device, args
    )
    torch.save(
        {"model_config": model_config, "model_state_dict": model.state_dict()},
        best_path,
    )
    history = [{
        "epoch": -1,
        "train_loss": None,
        "val_loss": best_loss,
        "successor_val_loss": best_successor_loss,
        "normal_val_loss": best_normal_loss,
    }]
    patience = 0
    for epoch in range(args.epochs):
        model.train()
        total_loss = 0.0
        successor_total_loss = 0.0
        normal_total_loss = 0.0
        batches = 0
        normal_iterator = (
            None if normal_train_loader is None else iter(normal_train_loader)
        )
        for batch in train_loader:
            optimizer.zero_grad(set_to_none=True)
            successor_loss = batch_loss(model, batch, device, args)
            normal_loss = None
            if normal_train_loader is not None:
                try:
                    normal_batch = next(normal_iterator)
                except StopIteration:
                    normal_iterator = iter(normal_train_loader)
                    normal_batch = next(normal_iterator)
                normal_loss = normal_anchor_loss(model, normal_batch, device, args)
            loss = successor_loss + (
                0.0
                if normal_loss is None
                else args.normal_anchor_weight * normal_loss
            )
            if not torch.isfinite(loss):
                raise FloatingPointError(f"non-finite recurrent loss at epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(
                (parameter for parameter in model.parameters() if parameter.requires_grad),
                args.gradient_clip_norm,
            )
            optimizer.step()
            total_loss += float(loss.item())
            successor_total_loss += float(successor_loss.item())
            normal_total_loss += 0.0 if normal_loss is None else float(normal_loss.item())
            batches += 1
        train_loss = total_loss / max(batches, 1)
        val_loss, successor_val_loss, normal_val_loss = combined_validation_objective(
            model, val_loader, normal_val_loader, device, args
        )
        history.append(
            {
                "epoch": epoch,
                "train_loss": train_loss,
                "successor_train_loss": successor_total_loss / max(batches, 1),
                "normal_train_loss": (
                    None
                    if normal_train_loader is None
                    else normal_total_loss / max(batches, 1)
                ),
                "val_loss": val_loss,
                "successor_val_loss": successor_val_loss,
                "normal_val_loss": normal_val_loss,
            }
        )
        print(
            f"epoch={epoch:03d} train={train_loss:.8f} val={val_loss:.8f} "
            f"successor_val={successor_val_loss:.8f} "
            f"normal_val={normal_val_loss} best={best_loss:.8f}"
        )
        if val_loss < best_loss - 1e-10:
            best_loss = val_loss
            best_successor_loss = successor_val_loss
            best_normal_loss = normal_val_loss
            patience = 0
            torch.save(
                {
                    "model_config": model_config,
                    "model_state_dict": model.state_dict(),
                },
                best_path,
            )
        else:
            patience += 1
            if patience >= args.early_stop_patience:
                break

    summary = {
        "best_distillation_validation_loss": best_distillation_loss,
        "distillation_history": distillation_history,
        "best_validation_loss": best_loss,
        "best_successor_validation_loss": best_successor_loss,
        "best_normal_validation_loss": best_normal_loss,
        "epochs_completed": len(history) - 1,
        "history": history,
        "checkpoint": str(best_path),
    }
    (output_dir / "training-summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(best_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
