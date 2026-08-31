"""Strict checkpoint loading shared by TinyLidarNet training workflows."""

import hashlib
from pathlib import Path
from typing import Dict, Mapping

import numpy as np
import torch


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _validate_torch_state(
    model: torch.nn.Module, provided: Mapping[str, torch.Tensor]
) -> Dict[str, torch.Tensor]:
    expected = model.state_dict()
    missing = sorted(set(expected) - set(provided))
    unexpected = sorted(set(provided) - set(expected))
    if missing or unexpected:
        raise ValueError(
            f"checkpoint key mismatch: missing={missing}, unexpected={unexpected}"
        )

    validated = {}
    for key, expected_tensor in expected.items():
        value = provided[key]
        if not isinstance(value, torch.Tensor):
            raise ValueError(f"checkpoint parameter {key} is not a tensor")
        if tuple(value.shape) != tuple(expected_tensor.shape):
            raise ValueError(
                f"checkpoint shape mismatch for {key}: "
                f"expected={tuple(expected_tensor.shape)}, actual={tuple(value.shape)}"
            )
        value = value.detach().to(device="cpu", dtype=expected_tensor.dtype)
        if not torch.isfinite(value).all():
            raise ValueError(f"checkpoint parameter {key} contains non-finite values")
        validated[key] = value
    return validated


def _load_numpy_state(
    model: torch.nn.Module, checkpoint_path: Path
) -> Dict[str, torch.Tensor]:
    loaded = np.load(checkpoint_path, allow_pickle=True)
    try:
        if isinstance(loaded, np.lib.npyio.NpzFile):
            raw = dict(loaded.items())
        elif isinstance(loaded, np.ndarray) and loaded.dtype == object:
            raw = loaded.item()
        else:
            raise ValueError(
                f"unsupported NumPy checkpoint payload: {type(loaded).__name__}"
            )
    finally:
        if isinstance(loaded, np.lib.npyio.NpzFile):
            loaded.close()

    if not isinstance(raw, dict):
        raise ValueError("NumPy checkpoint must contain a parameter dictionary")
    normalized = {}
    for key, value in raw.items():
        if not isinstance(key, str):
            raise ValueError("NumPy checkpoint parameter keys must be strings")
        normalized_key = key.replace(".", "_")
        if normalized_key in normalized:
            raise ValueError(f"duplicate normalized parameter key: {normalized_key}")
        array = np.asarray(value)
        if not np.issubdtype(array.dtype, np.number):
            raise ValueError(f"checkpoint parameter {key} must be numeric")
        normalized[normalized_key] = array

    expected_numpy_keys = {
        torch_key.replace(".", "_"): torch_key for torch_key in model.state_dict()
    }
    missing = sorted(set(expected_numpy_keys) - set(normalized))
    unexpected = sorted(set(normalized) - set(expected_numpy_keys))
    if missing or unexpected:
        raise ValueError(
            f"checkpoint key mismatch: missing={missing}, unexpected={unexpected}"
        )
    state = {
        torch_key: torch.from_numpy(normalized[numpy_key])
        for numpy_key, torch_key in expected_numpy_keys.items()
    }
    return _validate_torch_state(model, state)


def load_pretrained_weights(
    model: torch.nn.Module, checkpoint_path: Path
) -> dict:
    """Load `.npy` runtime weights or a raw PyTorch state dict, strictly."""
    checkpoint_path = checkpoint_path.expanduser().resolve()
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")

    suffix = checkpoint_path.suffix.lower()
    if suffix in {".npy", ".npz"}:
        state = _load_numpy_state(model, checkpoint_path)
        checkpoint_format = "numpy-runtime"
    elif suffix in {".pth", ".pt"}:
        try:
            raw = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
        except TypeError:
            raw = torch.load(checkpoint_path, map_location="cpu")
        if not isinstance(raw, dict):
            raise ValueError("PyTorch checkpoint must be a raw state dictionary")
        state = _validate_torch_state(model, raw)
        checkpoint_format = "pytorch-state-dict"
    else:
        raise ValueError(f"Unsupported checkpoint extension: {suffix}")

    model.load_state_dict(state, strict=True)
    return {
        "path": str(checkpoint_path),
        "format": checkpoint_format,
        "sha256": sha256_file(checkpoint_path),
        "parameter_count": len(state),
    }
