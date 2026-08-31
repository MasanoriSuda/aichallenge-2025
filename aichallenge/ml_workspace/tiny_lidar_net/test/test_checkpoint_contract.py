from pathlib import Path

import numpy as np
import pytest
import torch

from lib.checkpoint import load_pretrained_weights
from lib.model import TinyLidarNet
from convert_weight import convert_checkpoint


def numpy_state(model: torch.nn.Module) -> dict:
    return {
        key.replace(".", "_"): value.detach().cpu().numpy()
        for key, value in model.state_dict().items()
    }


def test_loads_runtime_numpy_checkpoint_into_pytorch(tmp_path: Path) -> None:
    torch.manual_seed(7)
    source = TinyLidarNet(input_dim=750, output_dim=2)
    checkpoint = tmp_path / "weights.npy"
    np.save(checkpoint, numpy_state(source))

    target = TinyLidarNet(input_dim=750, output_dim=2)
    metadata = load_pretrained_weights(target, checkpoint)

    assert metadata["format"] == "numpy-runtime"
    assert metadata["parameter_count"] == len(source.state_dict())
    for key, value in source.state_dict().items():
        torch.testing.assert_close(target.state_dict()[key], value)


def test_rejects_missing_numpy_parameter(tmp_path: Path) -> None:
    model = TinyLidarNet(input_dim=750, output_dim=2)
    values = numpy_state(model)
    values.pop("fc4_bias")
    checkpoint = tmp_path / "missing.npy"
    np.save(checkpoint, values)
    with pytest.raises(ValueError, match="key mismatch"):
        load_pretrained_weights(model, checkpoint)


def test_rejects_numpy_shape_mismatch(tmp_path: Path) -> None:
    model = TinyLidarNet(input_dim=750, output_dim=2)
    values = numpy_state(model)
    values["fc4_bias"] = np.zeros(3, dtype=np.float32)
    checkpoint = tmp_path / "wrong-shape.npy"
    np.save(checkpoint, values)
    with pytest.raises(ValueError, match="shape mismatch"):
        load_pretrained_weights(model, checkpoint)


def test_rejects_nonfinite_numpy_parameter(tmp_path: Path) -> None:
    model = TinyLidarNet(input_dim=750, output_dim=2)
    values = numpy_state(model)
    values["fc4_bias"] = np.array([np.nan, 0.0], dtype=np.float32)
    checkpoint = tmp_path / "nonfinite.npy"
    np.save(checkpoint, values)
    with pytest.raises(ValueError, match="non-finite"):
        load_pretrained_weights(model, checkpoint)


def test_conversion_uses_the_strict_checkpoint_contract(tmp_path: Path) -> None:
    model = TinyLidarNet(input_dim=750, output_dim=2)
    invalid = dict(model.state_dict())
    invalid.pop("fc4.bias")
    source = tmp_path / "invalid.pth"
    output = tmp_path / "candidate.npy"
    torch.save(invalid, source)

    with pytest.raises(ValueError, match="checkpoint key mismatch"):
        convert_checkpoint("tinylidarnet", 750, 2, source, output)

    assert not output.exists()
