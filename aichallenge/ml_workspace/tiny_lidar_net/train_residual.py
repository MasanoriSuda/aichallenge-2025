#!/usr/bin/env python3
"""Train a zero-default steering residual without modifying the base policy."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import random

import numpy as np
import torch
from torch.utils.data import DataLoader, WeightedRandomSampler

from lib.checkpoint import load_pretrained_weights, sha256_file
from lib.data import assert_disjoint_sequence_ids
from lib.residual import (
    MultiSeqResidualDataset,
    RESIDUAL_HEAD_ARCHITECTURES,
    RESIDUAL_INPUT_MODES,
    SignedMixtureSteeringResidualNet,
    SteeringResidualNet,
    residual_metrics,
    residual_input_channels,
    residual_training_loss,
    save_numpy_state,
    sequence_balanced_sample_weights,
    signed_direction_targets,
    signed_mixture_training_loss,
    write_json,
)


def as_model_input(scans: torch.Tensor) -> torch.Tensor:
    if scans.ndim == 2:
        return scans.unsqueeze(1)
    if scans.ndim == 3:
        return scans
    raise ValueError(f"unexpected residual scan batch shape: {tuple(scans.shape)}")


def build_model(args):
    model_type = (
        SteeringResidualNet
        if args.head_architecture == "binary_gate"
        else SignedMixtureSteeringResidualNet
    )
    return model_type(
        input_dim=args.input_dim,
        max_abs_delta_rad=args.max_abs_delta_rad,
        input_channels=residual_input_channels(args.architecture),
    )


def direction_class_weights(dataset, material_delta_rad: float) -> torch.Tensor:
    """Balance directions after the run-balanced sampler, not by raw length."""
    masses = []
    for sequence in dataset.datasets:
        targets = torch.from_numpy(sequence.steering_deltas)
        classes = signed_direction_targets(targets, material_delta_rad)
        counts = torch.bincount(classes, minlength=3).to(dtype=torch.float64)
        masses.append(counts / counts.sum())
    mean_mass = torch.stack(masses).mean(dim=0)
    if torch.any(mean_mass <= 0.0):
        raise ValueError(
            f"signed mixture training requires all direction classes: {mean_mass}"
        )
    inverse = 1.0 / mean_mass
    return (inverse / inverse.mean()).to(dtype=torch.float32)


def training_loss(model, scans, targets, args, class_weights):
    model_input = as_model_input(scans)
    if args.head_architecture == "binary_gate":
        predictions, corrections, gate_logits = model.forward_components(model_input)
        return residual_training_loss(
            predictions,
            corrections,
            gate_logits,
            targets,
            args.material_delta_rad,
            args.material_weight,
            args.gate_loss_weight,
            args.anchor_leakage_weight,
        )
    predictions, magnitudes, direction_logits, _ = model.forward_components(
        model_input
    )
    return signed_mixture_training_loss(
        predictions,
        magnitudes,
        direction_logits,
        targets,
        args.material_delta_rad,
        args.material_weight,
        class_weights,
        args.direction_loss_weight,
        args.anchor_leakage_weight,
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


def infer(model, loader, device) -> tuple[np.ndarray, np.ndarray]:
    model.eval()
    predictions = []
    targets = []
    with torch.no_grad():
        for scans, target in loader:
            output = model(as_model_input(scans).to(device)).cpu().numpy()
            predictions.append(output)
            targets.append(target.numpy())
    return np.concatenate(predictions), np.concatenate(targets)


def validation_objective(model, loader, device, args, class_weights) -> float:
    """Evaluate the exact training objective without updating model state."""
    model.eval()
    total = 0.0
    batches = 0
    with torch.no_grad():
        for scans, targets in loader:
            loss = training_loss(
                model,
                scans.to(device),
                targets.to(device),
                args,
                class_weights,
            )
            total += float(loss.item())
            batches += 1
    if batches == 0:
        raise RuntimeError("validation loader produced no batches")
    return total / batches


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--train-dir", type=Path, required=True)
    parser.add_argument("--val-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--init-checkpoint",
        type=Path,
        help=(
            "Optional admitted residual checkpoint used to warm-start a DAgger "
            "update. The frozen TinyLidarNet base is never modified."
        ),
    )
    parser.add_argument("--input-dim", type=int, default=750)
    parser.add_argument(
        "--architecture",
        choices=RESIDUAL_INPUT_MODES,
        default="stateless",
    )
    parser.add_argument(
        "--head-architecture",
        choices=RESIDUAL_HEAD_ARCHITECTURES,
        default="binary_gate",
    )
    parser.add_argument("--max-range-m", type=float, default=30.0)
    parser.add_argument("--max-abs-delta-rad", type=float, default=1.28)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--material-weight", type=float, default=20.0)
    parser.add_argument("--gate-loss-weight", type=float, default=0.01)
    parser.add_argument("--direction-loss-weight", type=float, default=1.0)
    parser.add_argument("--anchor-leakage-weight", type=float, default=0.5)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--early-stop-patience", type=int, default=5)
    parser.add_argument("--num-workers", type=int, default=4)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument(
        "--sequence-balanced-sampling",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Give each source run equal sampling mass so short causal DAgger "
            "prefixes are not diluted by long successful recordings."
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.batch_size <= 0 or args.epochs <= 0 or args.num_workers < 0:
        raise ValueError("invalid residual training iteration configuration")
    if args.learning_rate <= 0.0 or args.early_stop_patience <= 0:
        raise ValueError("invalid residual optimizer configuration")
    if args.direction_loss_weight < 0.0:
        raise ValueError("direction_loss_weight must be non-negative")

    generator = seed_everything(args.seed)
    train_dataset = MultiSeqResidualDataset(
        args.train_dir,
        expected_split="train",
        max_range=args.max_range_m,
        expected_input_dim=args.input_dim,
        material_delta_rad=args.material_delta_rad,
        input_mode=args.architecture,
    )
    val_dataset = MultiSeqResidualDataset(
        args.val_dir,
        expected_split="val",
        max_range=args.max_range_m,
        expected_input_dim=args.input_dim,
        material_delta_rad=args.material_delta_rad,
        input_mode=args.architecture,
    )
    assert_disjoint_sequence_ids(
        train_dataset.sequence_ids, val_dataset.sequence_ids
    )
    train_sampler = None
    if args.sequence_balanced_sampling:
        train_sampler = WeightedRandomSampler(
            sequence_balanced_sample_weights(
                [len(sequence) for sequence in train_dataset.datasets]
            ),
            num_samples=len(train_dataset),
            replacement=True,
            generator=generator,
        )
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=train_sampler is None,
        sampler=train_sampler,
        num_workers=args.num_workers,
        generator=generator,
        pin_memory=torch.cuda.is_available(),
        drop_last=False,
    )
    val_loader = DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=torch.cuda.is_available(),
        drop_last=False,
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = build_model(args).to(device)
    class_weights = (
        direction_class_weights(train_dataset, args.material_delta_rad).to(device)
        if args.head_architecture == "signed_mixture"
        else None
    )
    initialization = None
    if args.init_checkpoint is not None:
        initialization = load_pretrained_weights(model, args.init_checkpoint)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_root.expanduser().resolve() / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        "schema_version": 1,
        "model": type(model).__name__,
        "architecture": args.architecture,
        "head_architecture": args.head_architecture,
        "direction_class_weights": (
            None if class_weights is None else class_weights.cpu().tolist()
        ),
        "target": "precontact_teacher_minus_frozen_production_base_steering_rad",
        "initialization": initialization,
        "train_sequence_ids": train_dataset.sequence_ids,
        "val_sequence_ids": val_dataset.sequence_ids,
        "train_samples": len(train_dataset),
        "val_samples": len(val_dataset),
        "config": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
    }
    write_json(output_dir / "training-manifest.json", manifest)

    initial_val_loss = validation_objective(
        model, val_loader, device, args, class_weights
    )
    best_loss = initial_val_loss
    patience = 0
    history = []
    best_path = output_dir / "best_model.pth"
    torch.save(model.state_dict(), best_path)
    for epoch in range(args.epochs):
        model.train()
        train_total = 0.0
        train_batches = 0
        for scans, targets in train_loader:
            targets = targets.to(device)
            loss = training_loss(
                model,
                scans.to(device),
                targets,
                args,
                class_weights,
            )
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            train_total += float(loss.item())
            train_batches += 1

        train_loss = train_total / train_batches
        val_loss = validation_objective(
            model, val_loader, device, args, class_weights
        )
        history.append(
            {"epoch": epoch + 1, "train_loss": train_loss, "val_loss": val_loss}
        )
        print(
            f"epoch={epoch + 1:03d} train_loss={train_loss:.7f} "
            f"val_loss={val_loss:.7f}"
        )
        if val_loss < best_loss:
            best_loss = val_loss
            patience = 0
            torch.save(model.state_dict(), best_path)
        else:
            patience += 1
            if patience >= args.early_stop_patience:
                break

    # Keep the final optimizer state as a diagnostic artifact.  Promotion still
    # uses only the best validation state below; the last state exposes whether
    # a DAgger update learned the hard case at the cost of another gate.
    last_model_path = output_dir / "last_model.pth"
    last_candidate_path = output_dir / "last_candidate.npy"
    torch.save(model.state_dict(), last_model_path)
    save_numpy_state(model, last_candidate_path)

    try:
        best_state = torch.load(best_path, map_location=device, weights_only=True)
    except TypeError:
        best_state = torch.load(best_path, map_location=device)
    model.load_state_dict(best_state, strict=True)
    candidate_path = output_dir / "candidate.npy"
    save_numpy_state(model, candidate_path)
    val_predictions, val_targets = infer(model, val_loader, device)
    result = {
        "schema_version": 1,
        "candidate": str(candidate_path),
        "candidate_sha256": sha256_file(candidate_path),
        "last_candidate": str(last_candidate_path),
        "last_candidate_sha256": sha256_file(last_candidate_path),
        "best_weighted_val_loss": best_loss,
        "initial_weighted_val_loss": initial_val_loss,
        "epochs_completed": len(history),
        "metrics": residual_metrics(
            val_predictions, val_targets, args.material_delta_rad
        ),
        "history": history,
    }
    write_json(output_dir / "offline-evaluation.json", result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
