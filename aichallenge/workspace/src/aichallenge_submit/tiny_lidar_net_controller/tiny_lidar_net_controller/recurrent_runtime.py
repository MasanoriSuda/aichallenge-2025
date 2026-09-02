"""Strict construction boundary for self-described recurrent artifacts."""

from dataclasses import dataclass
import hashlib
from typing import Optional

import numpy as np

from tiny_lidar_net_controller.model.tinylidarnet import (
    RecurrentSteeringAdapterNp,
)


@dataclass(frozen=True)
class LoadedRecurrentRuntime:
    """A validated NumPy runtime and its immutable construction identity."""

    model: RecurrentSteeringAdapterNp
    runtime_config: dict
    artifact_contract: str
    loaded_parameter_count: int
    sha256: str


def sha256_file(path: str) -> str:
    """Return the content identity without accepting a directory or stream."""
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _verify_sha256(path: str, expected_sha256: str) -> str:
    expected = expected_sha256.strip().lower()
    if expected and (
        len(expected) != 64
        or any(character not in "0123456789abcdef" for character in expected)
    ):
        raise ValueError(
            "recurrent expected SHA256 must be 64 hexadecimal characters"
        )
    actual = sha256_file(path)
    if expected and actual != expected:
        raise ValueError(
            "recurrent SHA256 mismatch: "
            f"expected {expected}, got {actual}"
        )
    return actual


def _read_normalized_weights(path: str) -> dict:
    weights = np.load(path, allow_pickle=True)
    if isinstance(weights, np.lib.npyio.NpzFile):
        weight_dict = dict(weights.items())
        weights.close()
    elif isinstance(weights, np.ndarray) and weights.dtype == object:
        weight_dict = weights.item()
    elif isinstance(weights, dict):
        weight_dict = weights
    else:
        raise ValueError(f"unsupported recurrent weight format: {type(weights)}")

    normalized = {}
    for key, value in weight_dict.items():
        normalized_key = key.replace(".", "_")
        if normalized_key in normalized:
            raise ValueError(
                f"duplicate normalized recurrent parameter: {normalized_key}"
            )
        normalized[normalized_key] = np.asarray(value)
    return normalized


def _strict_load(model: RecurrentSteeringAdapterNp, weights: dict) -> int:
    expected_keys = set(model.params)
    provided_keys = set(weights)
    missing = sorted(expected_keys - provided_keys)
    unexpected = sorted(provided_keys - expected_keys)
    if missing or unexpected:
        raise ValueError(
            "recurrent weight key mismatch: "
            f"missing={missing}, unexpected={unexpected}"
        )

    validated = {}
    for key, expected in model.params.items():
        value = weights[key]
        if value.shape != expected.shape:
            raise ValueError(
                f"recurrent weight shape mismatch for {key}: "
                f"expected={expected.shape}, actual={value.shape}"
            )
        if not np.issubdtype(value.dtype, np.number):
            raise ValueError(
                f"recurrent weight {key} must be numeric, got {value.dtype}"
            )
        value = value.astype(np.float32, copy=False)
        if not np.all(np.isfinite(value)):
            raise ValueError(f"recurrent weight {key} contains non-finite values")
        validated[key] = value
    model.params.update(validated)
    return len(validated)


def load_recurrent_runtime(
    path: str,
    expected_sha256: str,
    *,
    legacy_runtime_config: Optional[dict] = None,
) -> LoadedRecurrentRuntime:
    """Load one artifact without mixing embedded and caller-owned config."""
    actual_sha256 = _verify_sha256(path, expected_sha256)
    artifact_config, weights = RecurrentSteeringAdapterNp.split_artifact(
        _read_normalized_weights(path)
    )
    if artifact_config is None:
        if legacy_runtime_config is None:
            raise ValueError(
                "legacy recurrent artifact requires explicit runtime config"
            )
        runtime_config = dict(legacy_runtime_config)
        artifact_contract = "legacy-config"
    else:
        runtime_config = dict(artifact_config)
        artifact_contract = "self-described-v1"

    model = RecurrentSteeringAdapterNp(**runtime_config)
    parameter_count = _strict_load(model, weights)
    return LoadedRecurrentRuntime(
        model=model,
        runtime_config=runtime_config,
        artifact_contract=artifact_contract,
        loaded_parameter_count=parameter_count,
        sha256=actual_sha256,
    )
