#!/usr/bin/env python3
"""Convert one admitted recurrent adapter to the NumPy runtime contract."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import torch

from lib.recurrent_policy import FrozenTinyLidarRecurrentAdapter


RUNTIME_SOURCE = (
    Path(__file__).resolve().parents[2]
    / "workspace"
    / "src"
    / "aichallenge_submit"
    / "tiny_lidar_net_controller"
)
sys.path.insert(0, str(RUNTIME_SOURCE))
from tiny_lidar_net_controller.model.tinylidarnet import (  # noqa: E402
    RecurrentSteeringAdapterNp,
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def runtime_config(model_config: dict, correction_deadband_rad: float) -> dict:
    if model_config.get("model_type") != "frozen_tinylidar_adapter":
        raise ValueError("runtime recurrent export requires frozen_tinylidar_adapter")
    if model_config.get("spatial_features") != "projected_conv5":
        raise ValueError("runtime recurrent export requires projected_conv5")
    if model_config.get("spatial_normalization") != "fixed_train_statistics":
        raise ValueError("runtime recurrent export requires fixed train statistics")
    if model_config.get("include_pressure_tokens", False):
        raise ValueError("runtime recurrent export rejects duplicate pressure tokens")
    if model_config.get("correction_head", "direct") != "direct":
        raise ValueError("runtime recurrent export currently supports direct correction")
    spatial = model_config.get("frozen_spatial_baseline_config")
    if not isinstance(spatial, dict):
        raise ValueError("runtime recurrent export requires a frozen spatial baseline")
    expected_spatial = {
        "spatial_normalization": "fixed_train_statistics",
        "use_speed": True,
        "use_base_steering": True,
        "head_architecture": "signed_mixture",
    }
    mismatches = {
        key: (spatial.get(key), expected)
        for key, expected in expected_spatial.items()
        if spatial.get(key) != expected
    }
    if mismatches:
        raise ValueError(f"unsupported frozen spatial baseline: {mismatches}")
    return {
        "input_dim": int(model_config["input_dim"]),
        "hidden_dim": int(model_config["hidden_dim"]),
        "projection_dim": int(model_config["spatial_projection_dim"]),
        "use_speed": bool(model_config.get("use_speed", True)),
        "speed_embedding_dim": int(model_config["speed_embedding_dim"]),
        "max_speed_mps": float(model_config["max_speed_mps"]),
        "max_abs_correction_rad": float(model_config["max_abs_correction_rad"]),
        "max_abs_steering_rad": float(model_config["max_abs_steering_rad"]),
        "correction_deadband_rad": float(correction_deadband_rad),
        "spatial_baseline_hidden_dim": int(spatial["hidden_dim"]),
        "spatial_baseline_projection_dim": int(spatial["projection_dim"]),
        "spatial_baseline_max_speed_mps": float(spatial["max_speed_mps"]),
        "spatial_baseline_max_abs_delta_rad": float(spatial["max_abs_delta_rad"]),
    }


def convert_recurrent_checkpoint(
    checkpoint_path: Path,
    output_path: Path,
    manifest_path: Path,
    correction_deadband_rad: float,
) -> dict:
    checkpoint_path = checkpoint_path.expanduser().resolve()
    output_path = output_path.expanduser().resolve()
    manifest_path = manifest_path.expanduser().resolve()
    if not np.isfinite(correction_deadband_rad) or correction_deadband_rad < 0.0:
        raise ValueError("correction deadband must be finite and non-negative")
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    if set(checkpoint) != {"model_config", "model_state_dict"}:
        raise ValueError("unexpected recurrent checkpoint contract")
    model_config = dict(checkpoint["model_config"])
    pytorch_config = dict(model_config)
    pytorch_config.pop("model_type", None)
    model = FrozenTinyLidarRecurrentAdapter(**pytorch_config)
    model.load_state_dict(checkpoint["model_state_dict"], strict=True)
    runtime_model_config = runtime_config(model_config, correction_deadband_rad)
    runtime = RecurrentSteeringAdapterNp(**runtime_model_config)

    exported = {
        key.replace(".", "_"): value.detach().cpu().numpy().astype(
            np.float32, copy=False
        )
        for key, value in model.state_dict().items()
    }
    expected_keys = set(runtime.params)
    provided_keys = set(exported)
    if expected_keys != provided_keys:
        raise ValueError(
            "recurrent NumPy key mismatch: "
            f"missing={sorted(expected_keys - provided_keys)}, "
            f"unexpected={sorted(provided_keys - expected_keys)}"
        )
    for key, expected in runtime.params.items():
        value = exported[key]
        if value.shape != expected.shape:
            raise ValueError(
                f"recurrent NumPy shape mismatch for {key}: "
                f"expected={expected.shape}, actual={value.shape}"
            )
        if not np.all(np.isfinite(value)):
            raise ValueError(f"recurrent NumPy parameter is non-finite: {key}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.save(output_path, exported)
    report = {
        "schema_version": 1,
        "source_checkpoint": str(checkpoint_path),
        "source_checkpoint_sha256": sha256_file(checkpoint_path),
        "output_checkpoint": str(output_path),
        "output_checkpoint_sha256": sha256_file(output_path),
        "parameter_count": len(exported),
        "model_config": model_config,
        "runtime_config": runtime_model_config,
    }
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--correction-deadband-rad", type=float, default=0.02)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output = args.output.expanduser().resolve()
    manifest = (
        output.with_suffix(output.suffix + ".manifest.json")
        if args.manifest is None
        else args.manifest
    )
    report = convert_recurrent_checkpoint(
        args.checkpoint,
        output,
        manifest,
        args.correction_deadband_rad,
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
