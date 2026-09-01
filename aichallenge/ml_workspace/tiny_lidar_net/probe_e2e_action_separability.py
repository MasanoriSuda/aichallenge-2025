#!/usr/bin/env python3
"""Compare frozen E2E state representations with a diagnostic action probe.

The probe predicts only whether the admitted successor teacher requests a
left, neutral, or right correction relative to frozen candidate3.  It is not a
runtime controller and cannot be promoted as a checkpoint.
"""

import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import random
import sys
from typing import Any, Iterable

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from lib.checkpoint import load_pretrained_weights
from lib.model import TinyLidarNet
from lib.recurrent_policy import MultiSeqRecurrentPolicyDataset


SCHEMA_VERSION = 1
VARIANTS = (
    "static_fc3",
    "static_conv5_no_speed",
    "static_conv5",
    "temporal_conv5",
)


@dataclass(frozen=True)
class ProbeSequence:
    sequence_id: str
    source_bag: str
    features: np.ndarray
    labels: np.ndarray


class ActionProbe(nn.Module):
    def __init__(self, input_dim: int, hidden_dim: int = 64):
        super().__init__()
        if input_dim <= 0 or hidden_dim <= 0:
            raise ValueError("probe dimensions must be positive")
        self.network = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 3),
        )

    def forward(self, values: torch.Tensor) -> torch.Tensor:
        return self.network(values)


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


def action_classes(
    teacher_steers_rad: np.ndarray,
    base_steers_rad: np.ndarray,
    material_delta_rad: float,
) -> np.ndarray:
    teacher = np.asarray(teacher_steers_rad, dtype=np.float32)
    base = np.asarray(base_steers_rad, dtype=np.float32)
    if teacher.shape != base.shape or teacher.ndim != 1:
        raise ValueError("teacher and base steering must be equal 1D arrays")
    if not np.isfinite(material_delta_rad) or material_delta_rad <= 0.0:
        raise ValueError("material delta must be finite and positive")
    correction = teacher - base
    labels = np.ones(len(correction), dtype=np.int64)
    labels[correction <= -material_delta_rad] = 0
    labels[correction >= material_delta_rad] = 2
    return labels


def temporal_features(
    current: np.ndarray,
    speeds_mps: np.ndarray,
    lags: tuple[int, ...] = (1, 8),
    max_speed_mps: float = 12.0,
) -> np.ndarray:
    features = np.asarray(current, dtype=np.float32)
    speeds = np.asarray(speeds_mps, dtype=np.float32)
    if features.ndim != 2 or speeds.shape != (len(features),):
        raise ValueError("temporal features and speeds must align")
    if not lags or any(lag <= 0 for lag in lags):
        raise ValueError("temporal lags must be positive")
    if max_speed_mps <= 0.0:
        raise ValueError("maximum speed must be positive")
    feature_parts = [features]
    speed_parts = [np.clip(speeds / max_speed_mps, 0.0, 1.5)[:, None]]
    for lag in lags:
        previous_indices = np.maximum(np.arange(len(features)) - lag, 0)
        feature_parts.append(features - features[previous_indices])
        speed_parts.append(
            ((speeds - speeds[previous_indices]) / max_speed_mps)[:, None]
        )
    return np.concatenate((*feature_parts, *speed_parts), axis=1).astype(
        np.float32, copy=False
    )


def standardize_from_train(
    train: np.ndarray, validation: np.ndarray
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    train_values = np.asarray(train, dtype=np.float32)
    validation_values = np.asarray(validation, dtype=np.float32)
    if train_values.ndim != 2 or validation_values.ndim != 2:
        raise ValueError("probe features must be two-dimensional")
    if train_values.shape[1] != validation_values.shape[1]:
        raise ValueError("train and validation feature dimensions differ")
    mean = np.mean(train_values, axis=0)
    std = np.std(train_values, axis=0)
    scale = np.maximum(std, 1e-4)
    return (
        ((train_values - mean) / scale).astype(np.float32),
        ((validation_values - mean) / scale).astype(np.float32),
        {
            "minimum_scale": float(np.min(scale)),
            "maximum_scale": float(np.max(scale)),
        },
    )


def probe_metrics(predicted: np.ndarray, labels: np.ndarray) -> dict[str, Any]:
    predictions = np.asarray(predicted, dtype=np.int64)
    targets = np.asarray(labels, dtype=np.int64)
    if predictions.shape != targets.shape or targets.ndim != 1 or not len(targets):
        raise ValueError("probe predictions and labels must be non-empty aligned arrays")
    supports = [int(np.count_nonzero(targets == index)) for index in range(3)]
    recalls = []
    for index, support in enumerate(supports):
        recalls.append(
            None
            if support == 0
            else float(np.mean(predictions[targets == index] == index))
        )
    present_recalls = [value for value in recalls if value is not None]
    material = targets != 1
    neutral = targets == 1
    return {
        "samples": len(targets),
        "class_support": {"left": supports[0], "neutral": supports[1], "right": supports[2]},
        "class_recall": {"left": recalls[0], "neutral": recalls[1], "right": recalls[2]},
        "accuracy": float(np.mean(predictions == targets)),
        "balanced_accuracy": float(np.mean(present_recalls)),
        "material_sign_accuracy": (
            None if not np.any(material) else float(np.mean(predictions[material] == targets[material]))
        ),
        "anchor_false_material_fraction": (
            None if not np.any(neutral) else float(np.mean(predictions[neutral] != 1))
        ),
    }


def random_projection(input_dim: int, output_dim: int, seed: int) -> np.ndarray:
    if min(input_dim, output_dim) <= 0:
        raise ValueError("projection dimensions must be positive")
    generator = np.random.default_rng(seed)
    return (
        generator.standard_normal((input_dim, output_dim), dtype=np.float32)
        / np.sqrt(float(output_dim))
    ).astype(np.float32)


def frozen_features(
    model: TinyLidarNet,
    scans_m: np.ndarray,
    device: torch.device,
) -> tuple[np.ndarray, np.ndarray]:
    scans = np.asarray(scans_m, dtype=np.float32)
    if scans.ndim != 2 or scans.shape[1] != 750:
        raise ValueError("frozen feature input must have shape (N, 750)")
    conv5_outputs = []
    fc3_outputs = []
    model.eval()
    with torch.no_grad():
        for start in range(0, len(scans), 512):
            values = torch.from_numpy(scans[start : start + 512]).to(device)
            values = torch.clamp(values / 30.0, 0.0, 1.0).unsqueeze(1)
            values = F.relu(model.conv1(values))
            values = F.relu(model.conv2(values))
            values = F.relu(model.conv3(values))
            values = F.relu(model.conv4(values))
            values = F.relu(model.conv5(values))
            flattened = torch.flatten(values, start_dim=1)
            conv5_outputs.append(flattened.cpu().numpy())
            compact = F.relu(model.fc1(flattened))
            compact = F.relu(model.fc2(compact))
            compact = F.relu(model.fc3(compact))
            fc3_outputs.append(compact.cpu().numpy())
    return (
        np.concatenate(conv5_outputs).astype(np.float32, copy=False),
        np.concatenate(fc3_outputs).astype(np.float32, copy=False),
    )


def make_probe_sequences(
    source,
    model: TinyLidarNet,
    variant: str,
    projection: np.ndarray,
    material_delta_rad: float,
    device: torch.device,
    feature_cache: dict[str, tuple[np.ndarray, np.ndarray]],
) -> list[ProbeSequence]:
    output = []
    for sequence in source.datasets:
        cached = feature_cache.get(sequence.sequence_id)
        if cached is None:
            conv5, fc3 = frozen_features(model, sequence.scans, device)
            if conv5.shape[1] != projection.shape[0]:
                raise ValueError("conv5 projection dimension mismatch")
            projected = conv5 @ projection
            cached = (projected, fc3)
            feature_cache[sequence.sequence_id] = cached
        projected, fc3 = cached
        if variant == "static_fc3":
            features = np.concatenate(
                (fc3, np.clip(sequence.speeds / 12.0, 0.0, 1.5)[:, None]),
                axis=1,
            )
        elif variant == "static_conv5_no_speed":
            features = projected
        elif variant == "static_conv5":
            features = np.concatenate(
                (projected, np.clip(sequence.speeds / 12.0, 0.0, 1.5)[:, None]),
                axis=1,
            )
        elif variant == "temporal_conv5":
            features = temporal_features(projected, sequence.speeds)
        else:
            raise ValueError(f"unsupported probe variant: {variant}")
        output.append(
            ProbeSequence(
                sequence_id=sequence.sequence_id,
                source_bag=str(sequence.metadata["source"]["bag"]),
                features=features,
                labels=action_classes(
                    sequence.steers, sequence.base_steers, material_delta_rad
                ),
            )
        )
    return output


def concatenate_sequences(
    sequences: Iterable[ProbeSequence],
) -> tuple[np.ndarray, np.ndarray]:
    sequence_list = list(sequences)
    if not sequence_list:
        raise ValueError("probe split has no sequences")
    return (
        np.concatenate([item.features for item in sequence_list]),
        np.concatenate([item.labels for item in sequence_list]),
    )


def train_probe(
    train_features: np.ndarray,
    train_labels: np.ndarray,
    validation_features: np.ndarray,
    validation_labels: np.ndarray,
    device: torch.device,
    generator: torch.Generator,
    epochs: int,
    batch_size: int,
    learning_rate: float,
    patience: int,
) -> tuple[ActionProbe, list[dict[str, float]]]:
    model = ActionProbe(train_features.shape[1]).to(device)
    counts = np.bincount(train_labels, minlength=3).astype(np.float64)
    if np.any(counts == 0):
        raise ValueError(f"training split lacks an action class: {counts.tolist()}")
    weights = torch.tensor(
        np.sum(counts) / (3.0 * counts), dtype=torch.float32, device=device
    )
    optimizer = torch.optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=1e-5)
    train_x = torch.from_numpy(train_features)
    train_y = torch.from_numpy(train_labels)
    validation_x = torch.from_numpy(validation_features).to(device)
    validation_y = torch.from_numpy(validation_labels).to(device)
    best_state = {key: value.detach().cpu().clone() for key, value in model.state_dict().items()}
    best_loss = float("inf")
    wait = 0
    history = []
    for epoch in range(epochs):
        model.train()
        order = torch.randperm(len(train_x), generator=generator)
        total = 0.0
        seen = 0
        for start in range(0, len(order), batch_size):
            selected = order[start : start + batch_size]
            values = train_x[selected].to(device)
            labels = train_y[selected].to(device)
            optimizer.zero_grad(set_to_none=True)
            loss = F.cross_entropy(model(values), labels, weight=weights)
            if not torch.isfinite(loss):
                raise FloatingPointError("non-finite action probe loss")
            loss.backward()
            optimizer.step()
            total += float(loss.item()) * len(selected)
            seen += len(selected)
        model.eval()
        with torch.no_grad():
            validation_loss = float(
                F.cross_entropy(model(validation_x), validation_y, weight=weights).item()
            )
        history.append(
            {
                "epoch": epoch,
                "train_loss": total / seen,
                "validation_loss": validation_loss,
            }
        )
        if validation_loss < best_loss - 1e-6:
            best_loss = validation_loss
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


def predict(model: ActionProbe, features: np.ndarray, device: torch.device) -> np.ndarray:
    model.eval()
    outputs = []
    with torch.no_grad():
        for start in range(0, len(features), 2048):
            logits = model(torch.from_numpy(features[start : start + 2048]).to(device))
            outputs.append(torch.argmax(logits, dim=1).cpu().numpy())
    return np.concatenate(outputs)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--projection-dim", type=int, default=128)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=1024)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--patience", type=int, default=6)
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--peer-validation-token", default="20260901-153143/d3")
    args = parser.parse_args()
    if min(args.projection_dim, args.epochs, args.batch_size, args.patience) <= 0:
        parser.error("projection, training and patience values must be positive")
    if args.material_delta_rad <= 0.0 or args.learning_rate <= 0.0:
        parser.error("material delta and learning rate must be positive")

    seed_everything(args.seed)
    dataset = args.dataset.expanduser().resolve()
    checkpoint = args.checkpoint.expanduser().resolve()
    train_source = MultiSeqRecurrentPolicyDataset(
        dataset / "train", expected_split="train"
    )
    validation_source = MultiSeqRecurrentPolicyDataset(
        dataset / "val", expected_split="val"
    )
    if set(train_source.sequence_ids) & set(validation_source.sequence_ids):
        raise ValueError("probe train/validation sequence identity overlap")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    base_model = TinyLidarNet(input_dim=750, output_dim=2)
    checkpoint_provenance = load_pretrained_weights(base_model, checkpoint)
    base_model.to(device).eval()
    with torch.no_grad():
        dummy = torch.zeros(1, 1, 750, device=device)
        conv5 = F.relu(base_model.conv1(dummy))
        conv5 = F.relu(base_model.conv2(conv5))
        conv5 = F.relu(base_model.conv3(conv5))
        conv5 = F.relu(base_model.conv4(conv5))
        conv5 = F.relu(base_model.conv5(conv5))
        conv5_dim = int(torch.flatten(conv5, start_dim=1).shape[1])
    projection = random_projection(conv5_dim, args.projection_dim, args.seed)

    variant_reports = {}
    feature_cache: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for variant in VARIANTS:
        train_sequences = make_probe_sequences(
            train_source,
            base_model,
            variant,
            projection,
            args.material_delta_rad,
            device,
            feature_cache,
        )
        validation_sequences = make_probe_sequences(
            validation_source,
            base_model,
            variant,
            projection,
            args.material_delta_rad,
            device,
            feature_cache,
        )
        train_features, train_labels = concatenate_sequences(train_sequences)
        validation_features, validation_labels = concatenate_sequences(
            validation_sequences
        )
        train_features, validation_features, scaling = standardize_from_train(
            train_features, validation_features
        )
        offset = 0
        normalized_validation_sequences = []
        for sequence in validation_sequences:
            length = len(sequence.features)
            normalized_validation_sequences.append(
                ProbeSequence(
                    sequence_id=sequence.sequence_id,
                    source_bag=sequence.source_bag,
                    features=validation_features[offset : offset + length],
                    labels=sequence.labels,
                )
            )
            offset += length
        # Each representation gets an equivalent deterministic optimization
        # run.  Reusing one advanced RNG would couple a variant's result to
        # the order in which other variants happened to run.
        generator = seed_everything(args.seed)
        model, history = train_probe(
            train_features,
            train_labels,
            validation_features,
            validation_labels,
            device,
            generator,
            args.epochs,
            args.batch_size,
            args.learning_rate,
            args.patience,
        )
        validation_predictions = predict(model, validation_features, device)
        per_sequence = []
        peer_validation = None
        for sequence in normalized_validation_sequences:
            record = {
                "sequence_id": sequence.sequence_id,
                "source_bag": sequence.source_bag,
                "metrics": probe_metrics(
                    predict(model, sequence.features, device), sequence.labels
                ),
            }
            per_sequence.append(record)
            if args.peer_validation_token in sequence.source_bag:
                if peer_validation is not None:
                    raise ValueError("peer validation token is ambiguous")
                peer_validation = record
        if peer_validation is None:
            raise ValueError("peer validation token did not match a sequence")
        variant_reports[variant] = {
            "feature_dim": int(train_features.shape[1]),
            "scaling": scaling,
            "epochs_completed": len(history),
            "best_validation_loss": float(
                min(item["validation_loss"] for item in history)
            ),
            "aggregate": probe_metrics(validation_predictions, validation_labels),
            "peer_validation": peer_validation,
            "per_sequence": per_sequence,
        }

    static = variant_reports["static_fc3"]["aggregate"]
    spatial_no_speed = variant_reports["static_conv5_no_speed"]["aggregate"]
    spatial = variant_reports["static_conv5"]["aggregate"]
    temporal = variant_reports["temporal_conv5"]["aggregate"]
    peer_temporal = variant_reports["temporal_conv5"]["peer_validation"]["metrics"]
    report = {
        "schema_version": SCHEMA_VERSION,
        "purpose": "offline diagnostic probe; not a runtime checkpoint",
        "dataset": str(dataset),
        "checkpoint": checkpoint_provenance,
        "checkpoint_sha256": sha256_file(checkpoint),
        "device": str(device),
        "config": {
            "material_delta_rad": args.material_delta_rad,
            "projection_dim": args.projection_dim,
            "seed": args.seed,
            "peer_validation_token": args.peer_validation_token,
        },
        "variants": variant_reports,
        "comparison": {
            "spatial_minus_compact_balanced_accuracy": (
                spatial["balanced_accuracy"] - static["balanced_accuracy"]
            ),
            "spatial_minus_compact_material_sign_accuracy": (
                spatial["material_sign_accuracy"] - static["material_sign_accuracy"]
            ),
            "spatial_speed_minus_no_speed_balanced_accuracy": (
                spatial["balanced_accuracy"]
                - spatial_no_speed["balanced_accuracy"]
            ),
            "spatial_speed_minus_no_speed_material_sign_accuracy": (
                spatial["material_sign_accuracy"]
                - spatial_no_speed["material_sign_accuracy"]
            ),
            "temporal_minus_static_balanced_accuracy": (
                temporal["balanced_accuracy"] - static["balanced_accuracy"]
            ),
            "temporal_minus_static_material_sign_accuracy": (
                temporal["material_sign_accuracy"] - static["material_sign_accuracy"]
            ),
            "temporal_minus_spatial_balanced_accuracy": (
                temporal["balanced_accuracy"] - spatial["balanced_accuracy"]
            ),
            "temporal_minus_spatial_material_sign_accuracy": (
                temporal["material_sign_accuracy"] - spatial["material_sign_accuracy"]
            ),
            "peer_temporal_material_samples": (
                peer_temporal["class_support"]["left"]
                + peer_temporal["class_support"]["right"]
            ),
            "peer_temporal_material_sign_accuracy": peer_temporal[
                "material_sign_accuracy"
            ],
            "peer_temporal_anchor_false_material_fraction": peer_temporal[
                "anchor_false_material_fraction"
            ],
        },
    }
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report["comparison"], indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
