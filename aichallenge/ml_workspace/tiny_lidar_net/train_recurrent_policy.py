#!/usr/bin/env python3
"""Train a speed-conditioned recurrent direct steering policy offline."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import random

import numpy as np
import torch
from torch.utils.data import DataLoader, WeightedRandomSampler

from lib.checkpoint import load_pretrained_weights
from lib.recurrent_policy import (
    FrozenTinyLidarRecurrentAdapter,
    MultiSeqRecurrentPolicyDataset,
    RecurrentDirectSteeringPolicy,
    recurrent_chunk_dataset,
    weighted_direct_policy_smooth_l1,
)


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


def batch_loss(model, batch, device, args) -> torch.Tensor:
    scans, speeds, targets, base = batch
    predictions, _ = model(scans.to(device), speeds.to(device))
    return weighted_direct_policy_smooth_l1(
        predictions,
        targets.to(device),
        base.to(device),
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
        "--adapter-pressure-tokens",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--input-dim", type=int, default=750)
    parser.add_argument("--speed-embedding-dim", type=int, default=64)
    parser.add_argument("--hidden-dim", type=int, default=512)
    parser.add_argument("--max-speed-mps", type=float, default=12.0)
    parser.add_argument("--max-abs-steering-rad", type=float, default=1.0)
    parser.add_argument("--chunk-length", type=int, default=64)
    parser.add_argument("--chunk-stride", type=int, default=32)
    parser.add_argument("--burn-in-steps", type=int, default=8)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--material-weight", type=float, default=2.0)
    parser.add_argument("--batch-size", type=int, default=8)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--distillation-epochs", type=int, default=25)
    parser.add_argument("--learning-rate", type=float, default=2e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-5)
    parser.add_argument("--gradient-clip-norm", type=float, default=1.0)
    parser.add_argument("--early-stop-patience", type=int, default=7)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument(
        "--sequence-balanced-successor",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Give every source run equal successor fine-tuning mass. This is "
            "off by default because failure prefixes have unequal durations."
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
    ):
        raise ValueError("invalid recurrent training configuration")

    generator = seed_everything(args.seed)
    train_sequences = MultiSeqRecurrentPolicyDataset(
        args.train_dir, expected_split="train"
    )
    val_sequences = MultiSeqRecurrentPolicyDataset(
        args.val_dir, expected_split="val"
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
    successor_sampler = None
    if args.sequence_balanced_successor:
        successor_sampler = WeightedRandomSampler(
            sequence_balanced_chunk_weights(train_chunks),
            num_samples=len(train_chunks),
            replacement=True,
            generator=generator,
        )
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
        }
        model = FrozenTinyLidarRecurrentAdapter(
            **{
                key: value
                for key, value in model_config.items()
                if key != "model_type"
            }
        )
        base_provenance = load_pretrained_weights(model.base, args.base_checkpoint)
        model.to(device)
        if args.distillation_epochs != 0:
            raise ValueError("frozen adapter must use --distillation-epochs 0")
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
        "train_sequence_ids": train_sequences.sequence_ids,
        "val_sequence_ids": val_sequences.sequence_ids,
        "train_chunks": len(train_chunks),
        "val_chunks": len(val_chunks),
        "config": {
            key: str(value) if isinstance(value, Path) else value
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
    best_loss = evaluate_objective(model, val_loader, device, args)
    torch.save(
        {"model_config": model_config, "model_state_dict": model.state_dict()},
        best_path,
    )
    history = [{"epoch": -1, "train_loss": None, "val_loss": best_loss}]
    patience = 0
    for epoch in range(args.epochs):
        model.train()
        total_loss = 0.0
        batches = 0
        for batch in train_loader:
            optimizer.zero_grad(set_to_none=True)
            loss = batch_loss(model, batch, device, args)
            if not torch.isfinite(loss):
                raise FloatingPointError(f"non-finite recurrent loss at epoch {epoch}")
            loss.backward()
            torch.nn.utils.clip_grad_norm_(
                (parameter for parameter in model.parameters() if parameter.requires_grad),
                args.gradient_clip_norm,
            )
            optimizer.step()
            total_loss += float(loss.item())
            batches += 1
        train_loss = total_loss / max(batches, 1)
        val_loss = evaluate_objective(model, val_loader, device, args)
        history.append(
            {"epoch": epoch, "train_loss": train_loss, "val_loss": val_loss}
        )
        print(
            f"epoch={epoch:03d} train={train_loss:.8f} val={val_loss:.8f} "
            f"best={best_loss:.8f}"
        )
        if val_loss < best_loss - 1e-10:
            best_loss = val_loss
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
