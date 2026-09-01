#!/usr/bin/env python3
"""Train one LiDAR-only spatial correction head on a frozen TinyLidarNet."""

import argparse
from datetime import datetime
import json
from pathlib import Path
import random

import numpy as np
import torch
from torch.utils.data import ConcatDataset, DataLoader, WeightedRandomSampler

from lib.checkpoint import load_pretrained_weights, sha256_file
from lib.recurrent_policy import MultiSeqRecurrentPolicyDataset
from lib.residual import (
    residual_metrics,
    save_numpy_state,
    sequence_balanced_sample_weights,
    signed_direction_targets,
    signed_mixture_training_loss,
    write_json,
)
from lib.spatial_adapter import FrozenTinyLidarSpatialResidual


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


def direction_class_weights(source, material_delta_rad: float) -> torch.Tensor:
    masses = []
    for sequence in source.datasets:
        targets = torch.from_numpy(sequence.steers - sequence.base_steers)
        classes = signed_direction_targets(targets, material_delta_rad)
        counts = torch.bincount(classes, minlength=3).to(dtype=torch.float64)
        masses.append(counts / counts.sum())
    mean_mass = torch.stack(masses).mean(dim=0)
    if torch.any(mean_mass <= 0.0):
        raise ValueError(f"spatial adapter split lacks a direction: {mean_mass}")
    inverse = 1.0 / mean_mass
    return (inverse / inverse.mean()).to(dtype=torch.float32)


def adapter_loss(model, scans, teacher, base, args, class_weights):
    targets = teacher - base
    residual, magnitudes, direction_logits, _ = model.forward_components(scans)
    return signed_mixture_training_loss(
        residual,
        magnitudes,
        direction_logits,
        targets,
        args.material_delta_rad,
        args.material_weight,
        class_weights,
        args.direction_loss_weight,
        args.anchor_leakage_weight,
    )


def validation_loss(model, loader, device, args, class_weights) -> float:
    model.eval()
    total = 0.0
    samples = 0
    with torch.no_grad():
        for scans, _, teacher, base in loader:
            scans = scans.to(device)
            teacher = teacher.to(device)
            base = base.to(device)
            loss = adapter_loss(model, scans, teacher, base, args, class_weights)
            total += float(loss.item()) * len(scans)
            samples += len(scans)
    if samples == 0:
        raise RuntimeError("spatial adapter validation loader is empty")
    return total / samples


def infer(model, loader, device, material_delta_rad: float) -> dict:
    model.eval()
    residuals = []
    targets = []
    predicted_classes = []
    with torch.no_grad():
        for scans, _, teacher, base in loader:
            residual, _, logits, _ = model.forward_components(scans.to(device))
            residuals.append(residual.cpu().numpy())
            targets.append((teacher - base).numpy())
            predicted_classes.append(torch.argmax(logits, dim=1).cpu().numpy())
    predicted = np.concatenate(residuals)
    target = np.concatenate(targets)
    classes = np.ones(len(target), dtype=np.int64)
    classes[target <= -material_delta_rad] = 0
    classes[target >= material_delta_rad] = 2
    directions = np.concatenate(predicted_classes)
    material = classes != 1
    anchors = classes == 1
    return {
        "residual": residual_metrics(predicted, target, material_delta_rad),
        "direction": {
            "accuracy": float(np.mean(directions == classes)),
            "material_sign_accuracy": (
                None
                if not np.any(material)
                else float(np.mean(directions[material] == classes[material]))
            ),
            "anchor_false_material_fraction": (
                None
                if not np.any(anchors)
                else float(np.mean(directions[anchors] != 1))
            ),
            "class_support": {
                "left": int(np.count_nonzero(classes == 0)),
                "neutral": int(np.count_nonzero(classes == 1)),
                "right": int(np.count_nonzero(classes == 2)),
            },
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--base-checkpoint", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--input-dim", type=int, default=750)
    parser.add_argument("--hidden-dim", type=int, default=128)
    parser.add_argument("--max-range-m", type=float, default=30.0)
    parser.add_argument("--max-abs-delta-rad", type=float, default=1.2)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--material-weight", type=float, default=5.0)
    parser.add_argument("--direction-loss-weight", type=float, default=1.0)
    parser.add_argument("--anchor-leakage-weight", type=float, default=0.5)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--patience", type=int, default=7)
    parser.add_argument("--seed", type=int, default=2026)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if min(args.input_dim, args.hidden_dim, args.batch_size, args.epochs, args.patience) <= 0:
        raise ValueError("spatial adapter dimensions and iterations must be positive")
    if args.learning_rate <= 0.0 or args.material_weight < 1.0:
        raise ValueError("invalid spatial adapter optimizer configuration")
    generator = seed_everything(args.seed)
    root = args.dataset.expanduser().resolve()
    train_source = MultiSeqRecurrentPolicyDataset(root / "train", "train")
    validation_source = MultiSeqRecurrentPolicyDataset(root / "val", "val")
    overlap = set(train_source.sequence_ids) & set(validation_source.sequence_ids)
    if overlap:
        raise ValueError(f"spatial adapter sequence overlap: {sorted(overlap)}")
    train_dataset = ConcatDataset(train_source.datasets)
    validation_dataset = ConcatDataset(validation_source.datasets)
    sampler = WeightedRandomSampler(
        sequence_balanced_sample_weights(
            [len(sequence) for sequence in train_source.datasets]
        ),
        num_samples=len(train_dataset),
        replacement=True,
        generator=generator,
    )
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        sampler=sampler,
        num_workers=0,
    )
    validation_loader = DataLoader(
        validation_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=0,
    )
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = FrozenTinyLidarSpatialResidual(
        input_dim=args.input_dim,
        hidden_dim=args.hidden_dim,
        max_scan_range_m=args.max_range_m,
        max_abs_delta_rad=args.max_abs_delta_rad,
    )
    base_provenance = load_pretrained_weights(model.base, args.base_checkpoint)
    model.to(device)
    trainable = [parameter for parameter in model.parameters() if parameter.requires_grad]
    optimizer = torch.optim.AdamW(trainable, lr=args.learning_rate, weight_decay=1e-5)
    class_weights = direction_class_weights(
        train_source, args.material_delta_rad
    ).to(device)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_root.expanduser().resolve() / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        "schema_version": 1,
        "purpose": "offline frozen-base static spatial correction candidate",
        "base_checkpoint": base_provenance,
        "base_checkpoint_sha256": sha256_file(args.base_checkpoint),
        "train_sequence_ids": train_source.sequence_ids,
        "validation_sequence_ids": validation_source.sequence_ids,
        "train_samples": len(train_dataset),
        "validation_samples": len(validation_dataset),
        "direction_class_weights": class_weights.cpu().tolist(),
        "config": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
    }
    write_json(output_dir / "training-manifest.json", manifest)
    best_loss = validation_loss(
        model, validation_loader, device, args, class_weights
    )
    initial_loss = best_loss
    best_state = {
        key: value.detach().cpu().clone() for key, value in model.state_dict().items()
    }
    history = []
    wait = 0
    for epoch in range(args.epochs):
        model.train()
        total = 0.0
        samples = 0
        for scans, _, teacher, base in train_loader:
            scans = scans.to(device)
            teacher = teacher.to(device)
            base = base.to(device)
            optimizer.zero_grad(set_to_none=True)
            loss = adapter_loss(model, scans, teacher, base, args, class_weights)
            if not torch.isfinite(loss):
                raise FloatingPointError("non-finite spatial adapter loss")
            loss.backward()
            optimizer.step()
            total += float(loss.item()) * len(scans)
            samples += len(scans)
        current_loss = validation_loss(
            model, validation_loader, device, args, class_weights
        )
        history.append(
            {
                "epoch": epoch + 1,
                "train_loss": total / samples,
                "validation_loss": current_loss,
            }
        )
        print(
            f"epoch={epoch + 1:03d} train_loss={total / samples:.7f} "
            f"validation_loss={current_loss:.7f}"
        )
        if current_loss < best_loss - 1e-6:
            best_loss = current_loss
            best_state = {
                key: value.detach().cpu().clone()
                for key, value in model.state_dict().items()
            }
            wait = 0
        else:
            wait += 1
            if wait >= args.patience:
                break
    model.load_state_dict(best_state, strict=True)
    candidate = output_dir / "candidate.npy"
    torch.save(best_state, output_dir / "best_model.pth")
    save_numpy_state(model, candidate)
    result = {
        "schema_version": 1,
        "candidate": str(candidate),
        "candidate_sha256": sha256_file(candidate),
        "initial_validation_loss": initial_loss,
        "best_validation_loss": best_loss,
        "epochs_completed": len(history),
        "validation": infer(
            model, validation_loader, device, args.material_delta_rad
        ),
        "history": history,
    }
    write_json(output_dir / "offline-evaluation.json", result)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
