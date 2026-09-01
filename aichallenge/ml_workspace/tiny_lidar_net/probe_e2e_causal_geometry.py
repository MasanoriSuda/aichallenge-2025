#!/usr/bin/env python3
"""Diagnose causal LiDAR action separability with a local CNN and GRU."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset

from lib.checkpoint import load_pretrained_weights, sha256_file
from lib.model import TinyLidarNet
from lib.normal_anchor import MultiSeqNormalAnchorDataset
from lib.recurrent_policy import MultiSeqRecurrentPolicyDataset
from probe_e2e_action_separability import (
    action_classes,
    probe_metrics,
    seed_everything,
)


SCHEMA_VERSION = 1


@dataclass(frozen=True)
class TemporalProbeSequence:
    sequence_id: str
    source_bag: str
    features: np.ndarray
    labels: np.ndarray
    is_normal: bool


class CausalGeometryActionProbe(nn.Module):
    def __init__(self, input_dim: int = 752, hidden_dim: int = 64):
        super().__init__()
        if input_dim != 752 or hidden_dim <= 0:
            raise ValueError("causal probe requires scan[750]+speed+base")
        self.encoder = nn.Sequential(
            nn.Conv1d(1, 16, kernel_size=9, stride=3),
            nn.ReLU(),
            nn.Conv1d(16, 24, kernel_size=7, stride=3),
            nn.ReLU(),
            nn.Conv1d(24, 32, kernel_size=5, stride=2),
            nn.ReLU(),
            nn.Conv1d(32, 32, kernel_size=3),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(8),
        )
        self.context = nn.Sequential(nn.Linear(2, 16), nn.ReLU())
        self.frame = nn.Sequential(nn.Linear(32 * 8 + 16, 64), nn.ReLU())
        self.recurrent = nn.GRU(64, hidden_dim, batch_first=True)
        self.output = nn.Linear(hidden_dim, 3)

    def forward(
        self, values: torch.Tensor, hidden: torch.Tensor | None = None
    ) -> tuple[torch.Tensor, torch.Tensor]:
        if values.ndim != 3 or values.shape[2] != 752:
            raise ValueError("causal probe input must have shape (N, T, 752)")
        batch, time, _ = values.shape
        scan = values[:, :, :750].reshape(batch * time, 1, 750)
        context = values[:, :, 750:].reshape(batch * time, 2)
        encoded = self.encoder(scan).flatten(start_dim=1)
        token = self.frame(
            torch.cat((encoded, self.context(context)), dim=1)
        ).reshape(batch, time, -1)
        recurrent, next_hidden = self.recurrent(token, hidden)
        return self.output(recurrent), next_hidden


class TemporalProbeChunkDataset(Dataset):
    def __init__(
        self,
        sequences: list[TemporalProbeSequence],
        chunk_length: int,
        stride: int,
    ):
        if chunk_length <= 1 or stride <= 0:
            raise ValueError("invalid temporal probe chunk contract")
        self.sequences = sequences
        self.chunk_length = int(chunk_length)
        self.records: list[tuple[int, int]] = []
        for sequence_index, sequence in enumerate(sequences):
            if len(sequence.labels) < chunk_length:
                raise ValueError(
                    f"temporal probe sequence too short: {sequence.sequence_id}"
                )
            starts = list(range(0, len(sequence.labels) - chunk_length + 1, stride))
            final_start = len(sequence.labels) - chunk_length
            if starts[-1] != final_start:
                starts.append(final_start)
            self.records.extend((sequence_index, start) for start in starts)

    def __len__(self) -> int:
        return len(self.records)

    def __getitem__(self, index: int):
        sequence_index, start = self.records[index]
        stop = start + self.chunk_length
        sequence = self.sequences[sequence_index]
        return sequence.features[start:stop], sequence.labels[start:stop]


def base_steering_predictions(
    model: TinyLidarNet,
    scans_m: np.ndarray,
    device: torch.device,
    batch_size: int = 512,
) -> np.ndarray:
    scans = np.asarray(scans_m, dtype=np.float32)
    if scans.ndim != 2 or scans.shape[1] != 750 or batch_size <= 0:
        raise ValueError("base replay requires physical scan matrix")
    outputs = []
    model.eval()
    with torch.no_grad():
        for start in range(0, len(scans), batch_size):
            values = torch.from_numpy(scans[start : start + batch_size]).to(device)
            values = torch.clamp(values / 30.0, 0.0, 1.0).unsqueeze(1)
            outputs.append(model(values)[:, 1].cpu().numpy())
    return np.concatenate(outputs).astype(np.float32, copy=False)


def compose_temporal_features(
    scans_m: np.ndarray,
    speeds_mps: np.ndarray,
    base_steering_rad: np.ndarray,
) -> np.ndarray:
    scans = np.asarray(scans_m, dtype=np.float32)
    speeds = np.asarray(speeds_mps, dtype=np.float32)
    base = np.asarray(base_steering_rad, dtype=np.float32)
    if (
        scans.ndim != 2
        or scans.shape[1] != 750
        or speeds.shape != (len(scans),)
        or base.shape != (len(scans),)
    ):
        raise ValueError("causal probe physical inputs must align")
    return np.concatenate(
        (
            np.clip(scans / 30.0, 0.0, 1.0),
            np.clip(speeds / 12.0, 0.0, 1.5)[:, None],
            base[:, None],
        ),
        axis=1,
    ).astype(np.float32, copy=False)


def make_sequences(
    source,
    base_model: TinyLidarNet,
    device: torch.device,
    material_delta_rad: float,
    is_normal: bool,
) -> list[TemporalProbeSequence]:
    output = []
    for sequence in source.datasets:
        base = base_steering_predictions(base_model, sequence.scans, device)
        labels = (
            np.ones(len(sequence), dtype=np.int64)
            if is_normal
            else action_classes(
                sequence.steers, sequence.base_steers, material_delta_rad
            )
        )
        output.append(
            TemporalProbeSequence(
                sequence_id=(
                    f"normal:{sequence.sequence_id}"
                    if is_normal
                    else sequence.sequence_id
                ),
                source_bag=str(sequence.metadata["source"]["bag"]),
                features=compose_temporal_features(
                    sequence.scans, sequence.speeds, base
                ),
                labels=labels,
                is_normal=is_normal,
            )
        )
    return output


def causal_logits(
    model: CausalGeometryActionProbe,
    sequence: TemporalProbeSequence,
    device: torch.device,
    block_size: int = 256,
) -> np.ndarray:
    if block_size <= 0:
        raise ValueError("causal replay block must be positive")
    model.eval()
    outputs = []
    hidden = None
    with torch.no_grad():
        for start in range(0, len(sequence.labels), block_size):
            values = torch.from_numpy(
                sequence.features[start : start + block_size]
            ).unsqueeze(0).to(device)
            logits, hidden = model(values, hidden)
            outputs.append(logits.squeeze(0).cpu().numpy())
    return np.concatenate(outputs).astype(np.float32, copy=False)


def validation_loss(
    model: CausalGeometryActionProbe,
    sequences: list[TemporalProbeSequence],
    class_weights: torch.Tensor,
    device: torch.device,
) -> float:
    losses = []
    samples = 0
    for sequence in sequences:
        logits = torch.from_numpy(causal_logits(model, sequence, device)).to(device)
        labels = torch.from_numpy(sequence.labels).to(device)
        losses.append(
            float(
                F.cross_entropy(logits, labels, weight=class_weights).item()
            )
            * len(labels)
        )
        samples += len(labels)
    return float(sum(losses) / samples)


def train_probe(
    train_sequences: list[TemporalProbeSequence],
    validation_sequences: list[TemporalProbeSequence],
    device: torch.device,
    seed: int,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    patience: int,
    chunk_length: int,
    stride: int,
    burn_in_steps: int,
) -> tuple[CausalGeometryActionProbe, list[dict]]:
    if burn_in_steps < 0 or burn_in_steps >= chunk_length:
        raise ValueError("burn-in must leave supervised temporal samples")
    generator = seed_everything(seed)
    dataset = TemporalProbeChunkDataset(train_sequences, chunk_length, stride)
    loader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        generator=generator,
        num_workers=0,
    )
    labels = np.concatenate([sequence.labels for sequence in train_sequences])
    counts = np.bincount(labels, minlength=3).astype(np.float64)
    if np.any(counts == 0):
        raise ValueError("causal probe training lacks an action class")
    class_weights = torch.tensor(
        np.sum(counts) / (3.0 * counts), dtype=torch.float32, device=device
    )
    model = CausalGeometryActionProbe().to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(), lr=learning_rate, weight_decay=1e-5
    )
    best_loss = float("inf")
    best_state = {
        key: value.detach().cpu().clone()
        for key, value in model.state_dict().items()
    }
    wait = 0
    history = []
    for epoch in range(epochs):
        model.train()
        total = 0.0
        seen = 0
        for features, targets in loader:
            values = features.to(device)
            labels_batch = targets.to(device)
            optimizer.zero_grad(set_to_none=True)
            logits, _ = model(values)
            loss = F.cross_entropy(
                logits[:, burn_in_steps:].reshape(-1, 3),
                labels_batch[:, burn_in_steps:].reshape(-1),
                weight=class_weights,
            )
            if not torch.isfinite(loss):
                raise FloatingPointError("non-finite causal probe loss")
            loss.backward()
            optimizer.step()
            supervised = labels_batch[:, burn_in_steps:].numel()
            total += float(loss.item()) * supervised
            seen += supervised
        current_validation = validation_loss(
            model, validation_sequences, class_weights, device
        )
        history.append(
            {
                "epoch": epoch,
                "train_loss": total / seen,
                "validation_loss": current_validation,
            }
        )
        if current_validation < best_loss - 1e-6:
            best_loss = current_validation
            best_state = {
                key: value.detach().cpu().clone()
                for key, value in model.state_dict().items()
            }
            wait = 0
        else:
            wait += 1
            if wait >= patience:
                break
    model.load_state_dict(best_state)
    return model, history


def evaluate_sequences(
    model: CausalGeometryActionProbe,
    sequences: list[TemporalProbeSequence],
    device: torch.device,
    peer_token: str,
    focus_token: str,
    tail_samples: int,
) -> dict:
    predictions = []
    targets = []
    normal_predictions = []
    normal_targets = []
    per_sequence = []
    peer = None
    focus = None
    for sequence in sequences:
        predicted = np.argmax(causal_logits(model, sequence, device), axis=1)
        record = {
            "sequence_id": sequence.sequence_id,
            "source_bag": sequence.source_bag,
            "is_normal": sequence.is_normal,
            "metrics": probe_metrics(predicted, sequence.labels),
            "tail_samples": min(tail_samples, len(sequence.labels)),
            "tail_metrics": probe_metrics(
                predicted[-tail_samples:], sequence.labels[-tail_samples:]
            ),
        }
        per_sequence.append(record)
        predictions.append(predicted)
        targets.append(sequence.labels)
        if sequence.is_normal:
            normal_predictions.append(predicted)
            normal_targets.append(sequence.labels)
        if peer_token in sequence.source_bag:
            if peer is not None:
                raise ValueError("peer token is ambiguous")
            peer = record
        if focus_token and focus_token in sequence.source_bag:
            if focus is not None:
                raise ValueError("focus token is ambiguous")
            focus = record
    if peer is None or (focus_token and focus is None):
        raise ValueError("causal validation token did not match exactly once")
    return {
        "aggregate": probe_metrics(
            np.concatenate(predictions), np.concatenate(targets)
        ),
        "normal_validation": probe_metrics(
            np.concatenate(normal_predictions), np.concatenate(normal_targets)
        ),
        "peer_validation": peer,
        "focus_validation": focus,
        "per_sequence": per_sequence,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--normal-recurrent-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--patience", type=int, default=5)
    parser.add_argument("--chunk-length", type=int, default=32)
    parser.add_argument("--stride", type=int, default=16)
    parser.add_argument("--burn-in-steps", type=int, default=8)
    parser.add_argument("--tail-samples", type=int, default=200)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--peer-validation-token", default="20260901-153143/d3")
    parser.add_argument("--focus-validation-token", default="20260901-130837/d1")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if min(
        args.epochs,
        args.batch_size,
        args.patience,
        args.chunk_length,
        args.stride,
        args.tail_samples,
    ) <= 0:
        raise ValueError("causal probe dimensions must be positive")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    base = TinyLidarNet(input_dim=750, output_dim=2)
    checkpoint = args.checkpoint.expanduser().resolve()
    checkpoint_provenance = load_pretrained_weights(base, checkpoint)
    base.to(device).eval()

    teacher_root = args.dataset.expanduser().resolve()
    normal_root = args.normal_recurrent_root.expanduser().resolve()
    teacher_train = MultiSeqRecurrentPolicyDataset(teacher_root / "train", "train")
    teacher_val = MultiSeqRecurrentPolicyDataset(teacher_root / "val", "val")
    normal_train = MultiSeqNormalAnchorDataset(normal_root / "train", "train")
    normal_val = MultiSeqNormalAnchorDataset(normal_root / "val", "val")
    train_sequences = make_sequences(
        teacher_train, base, device, args.material_delta_rad, False
    ) + make_sequences(normal_train, base, device, args.material_delta_rad, True)
    validation_sequences = make_sequences(
        teacher_val, base, device, args.material_delta_rad, False
    ) + make_sequences(normal_val, base, device, args.material_delta_rad, True)
    train_ids = {sequence.sequence_id for sequence in train_sequences}
    validation_ids = {sequence.sequence_id for sequence in validation_sequences}
    if train_ids & validation_ids:
        raise ValueError("causal probe train/validation identity overlap")
    train_bags = {sequence.source_bag for sequence in train_sequences}
    validation_bags = {
        sequence.source_bag for sequence in validation_sequences
    }
    if train_bags & validation_bags:
        raise ValueError("causal probe train/validation source-bag overlap")

    model, history = train_probe(
        train_sequences,
        validation_sequences,
        device,
        args.seed,
        args.epochs,
        args.batch_size,
        args.learning_rate,
        args.patience,
        args.chunk_length,
        args.stride,
        args.burn_in_steps,
    )
    evaluation = evaluate_sequences(
        model,
        validation_sequences,
        device,
        args.peer_validation_token,
        args.focus_validation_token,
        args.tail_samples,
    )
    report = {
        "schema_version": SCHEMA_VERSION,
        "purpose": "diagnostic causal action probe; not a runtime checkpoint",
        "dataset": str(teacher_root),
        "normal_recurrent_root": str(normal_root),
        "checkpoint": checkpoint_provenance,
        "checkpoint_sha256": sha256_file(checkpoint),
        "device": str(device),
        "config": {
            "seed": args.seed,
            "epochs": args.epochs,
            "batch_size": args.batch_size,
            "learning_rate": args.learning_rate,
            "patience": args.patience,
            "chunk_length": args.chunk_length,
            "stride": args.stride,
            "burn_in_steps": args.burn_in_steps,
            "tail_samples": args.tail_samples,
            "material_delta_rad": args.material_delta_rad,
        },
        "model_parameter_count": int(
            sum(parameter.numel() for parameter in model.parameters())
        ),
        "epochs_completed": len(history),
        "best_validation_loss": float(
            min(record["validation_loss"] for record in history)
        ),
        **evaluation,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        json.dumps(
            {
                "aggregate": report["aggregate"],
                "normal_validation": report["normal_validation"],
                "peer_validation": report["peer_validation"],
                "focus_validation": report["focus_validation"],
                "epochs_completed": report["epochs_completed"],
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
