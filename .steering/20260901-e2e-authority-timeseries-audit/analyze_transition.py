#!/usr/bin/env python3
"""Replay one spatial adapter on an immutable recurrent failure prefix."""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch.utils.data import DataLoader

from evaluate_spatial_adapter import runtime_bounded_metrics
from lib.checkpoint import load_pretrained_weights, sha256_file
from lib.recurrent_policy import MultiSeqRecurrentPolicyDataset
from lib.spatial_adapter import FrozenTinyLidarSpatialResidual


def signed_targets(values: np.ndarray, threshold: float) -> np.ndarray:
    result = np.zeros(len(values), dtype=np.int8)
    result[values <= -threshold] = -1
    result[values >= threshold] = 1
    return result


def contiguous_segments(values: np.ndarray):
    if not len(values):
        return []
    starts = np.concatenate(([0], np.flatnonzero(values[1:] != values[:-1]) + 1))
    stops = np.concatenate((starts[1:], [len(values)]))
    return [
        (int(start), int(stop), int(values[start]))
        for start, stop in zip(starts, stops)
    ]


def sustained_sign_index(
    predictions: np.ndarray, start: int, stop: int, sign: int, samples: int
):
    if sign == 0:
        return None
    for index in range(start, stop - samples + 1):
        window = predictions[index : index + samples]
        if len(window) == samples and np.all(np.sign(window) == sign):
            return index
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--source-token", required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--tail-samples", type=int, default=200)
    parser.add_argument("--material-delta-rad", type=float, default=0.02)
    parser.add_argument("--sustain-samples", type=int, default=5)
    parser.add_argument(
        "--authority-bound-rad", type=float, action="append", required=True
    )
    args = parser.parse_args()
    if min(args.tail_samples, args.sustain_samples) <= 0:
        parser.error("tail and sustain samples must be positive")

    manifest_path = args.candidate.parent / "training-manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    config = manifest["config"]
    source = MultiSeqRecurrentPolicyDataset(args.dataset / "val", "val")
    matches = [
        sequence
        for sequence in source.datasets
        if args.source_token in str(sequence.metadata["source"]["bag"])
    ]
    if len(matches) != 1:
        raise ValueError(f"source token matched {len(matches)} sequences")
    sequence = matches[0]
    model = FrozenTinyLidarSpatialResidual(
        input_dim=int(config["input_dim"]),
        hidden_dim=int(config["hidden_dim"]),
        max_scan_range_m=float(config["max_range_m"]),
        max_abs_delta_rad=float(config["max_abs_delta_rad"]),
        use_speed=bool(config["use_speed"]),
        max_speed_mps=float(config["max_speed_mps"]),
        spatial_normalization=str(config["spatial_normalization"]),
        projection_dim=int(config["projection_dim"]),
        projection_seed=int(config["projection_seed"]),
        head_architecture=str(config["head_architecture"]),
    )
    load_pretrained_weights(model, args.candidate)
    model.eval()
    predictions = []
    with torch.no_grad():
        for scans, speeds, _, _ in DataLoader(sequence, batch_size=512, shuffle=False):
            predictions.append(model(scans, speeds).numpy())
    predicted = np.concatenate(predictions)
    target = sequence.steers - sequence.base_steers
    tail_start = max(0, len(sequence) - args.tail_samples)
    tail_target = target[tail_start:]
    tail_predicted = predicted[tail_start:]
    labels = signed_targets(tail_target, args.material_delta_rad)
    timestamps = sequence.scan_timestamps_ns[tail_start:]
    segments = []
    for start, stop, sign in contiguous_segments(labels):
        sustained = sustained_sign_index(
            tail_predicted, start, stop, sign, args.sustain_samples
        )
        segments.append(
            {
                "start_index": tail_start + start,
                "stop_index": tail_start + stop,
                "samples": stop - start,
                "target_sign": sign,
                "start_sec": float(timestamps[start] - timestamps[0]) / 1e9,
                "stop_sec": float(timestamps[stop - 1] - timestamps[0]) / 1e9,
                "mean_target_rad": float(np.mean(tail_target[start:stop])),
                "mean_prediction_rad": float(np.mean(tail_predicted[start:stop])),
                "sustained_prediction_index": (
                    None if sustained is None else tail_start + sustained
                ),
                "sustained_delay_sec": (
                    None
                    if sustained is None
                    else float(timestamps[sustained] - timestamps[start]) / 1e9
                ),
            }
        )

    bounds = {}
    for bound in args.authority_bound_rad:
        bounds[f"{bound:.6f}"] = runtime_bounded_metrics(
            tail_predicted,
            tail_target,
            args.material_delta_rad,
            bound,
        )
    report = {
        "schema_version": 1,
        "purpose": "diagnostic-only; no runtime authority change",
        "candidate": {
            "path": str(args.candidate.resolve()),
            "sha256": sha256_file(args.candidate),
        },
        "source": {
            "sequence_id": sequence.sequence_id,
            "bag": sequence.metadata["source"]["bag"],
            "samples": len(sequence),
            "tail_samples": len(tail_target),
            "tail_duration_sec": float(timestamps[-1] - timestamps[0]) / 1e9,
        },
        "target_support": {
            "left": int(np.count_nonzero(labels == -1)),
            "neutral": int(np.count_nonzero(labels == 0)),
            "right": int(np.count_nonzero(labels == 1)),
        },
        "segments": segments,
        "authority_bounds": bounds,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
