from pathlib import Path
import sys

import numpy as np
import pytest
import torch

from convert_recurrent_policy import convert_recurrent_checkpoint
from lib.recurrent_policy import FrozenTinyLidarRecurrentAdapter


REPOSITORY_ROOT = Path(__file__).resolve().parents[4]
RUNTIME_SOURCE = (
    REPOSITORY_ROOT
    / "aichallenge"
    / "workspace"
    / "src"
    / "aichallenge_submit"
    / "tiny_lidar_net_controller"
)
sys.path.insert(0, str(RUNTIME_SOURCE))
from tiny_lidar_net_controller.model.tinylidarnet import (  # noqa: E402
    RecurrentSteeringAdapterNp,
)


def recurrent_config(correction_head: str = "direct") -> dict:
    return {
        "model_type": "frozen_tinylidar_adapter",
        "input_dim": 750,
        "speed_embedding_dim": 5,
        "hidden_dim": 7,
        "max_scan_range_m": 30.0,
        "max_speed_mps": 12.0,
        "max_abs_correction_rad": 0.64,
        "max_abs_steering_rad": 1.0,
        "include_pressure_tokens": False,
        "spatial_features": "projected_conv5",
        "spatial_projection_dim": 12,
        "spatial_projection_seed": 2026,
        "spatial_normalization": "fixed_train_statistics",
        "use_speed": False,
        "frozen_spatial_baseline_config": {
            "input_dim": 750,
            "hidden_dim": 16,
            "max_scan_range_m": 30.0,
            "max_abs_delta_rad": 1.2,
            "use_speed": True,
            "use_base_steering": True,
            "max_speed_mps": 12.0,
            "spatial_normalization": "fixed_train_statistics",
            "projection_dim": 8,
            "projection_seed": 2026,
            "head_architecture": "signed_mixture",
        },
        "correction_head": correction_head,
    }


def make_nontrivial_model() -> tuple[dict, FrozenTinyLidarRecurrentAdapter]:
    torch.manual_seed(2043)
    config = recurrent_config()
    pytorch_config = dict(config)
    pytorch_config.pop("model_type")
    model = FrozenTinyLidarRecurrentAdapter(**pytorch_config)
    model.spatial_baseline.base.load_state_dict(model.base.state_dict(), strict=True)
    model.set_spatial_statistics(
        torch.linspace(-0.1, 0.1, 12), torch.linspace(0.5, 1.5, 12)
    )
    model.spatial_baseline.set_spatial_statistics(
        torch.linspace(-0.2, 0.2, 8), torch.linspace(0.6, 1.4, 8)
    )
    with torch.no_grad():
        model.spatial_baseline.direction_head.weight.normal_(0.0, 0.04)
        model.spatial_baseline.direction_head.bias.copy_(
            torch.tensor([-0.1, 0.2, 0.3])
        )
        model.spatial_baseline.magnitude_head.weight.normal_(0.0, 0.03)
        model.spatial_baseline.magnitude_head.bias.copy_(
            torch.tensor([-0.3, 0.1])
        )
        model.correction_output.weight.normal_(0.0, 0.05)
        model.correction_output.bias.fill_(0.02)
    model.eval()
    return config, model


def test_numpy_recurrent_runtime_matches_pytorch_sequence(tmp_path: Path) -> None:
    config, model = make_nontrivial_model()
    checkpoint = tmp_path / "candidate.pth"
    torch.save(
        {"model_config": config, "model_state_dict": model.state_dict()},
        checkpoint,
    )
    output = tmp_path / "candidate.npy"
    manifest = tmp_path / "candidate.manifest.json"
    report = convert_recurrent_checkpoint(
        checkpoint, output, manifest, correction_deadband_rad=0.0
    )
    runtime = RecurrentSteeringAdapterNp(**report["runtime_config"])
    exported = np.load(output, allow_pickle=True).item()
    assert set(exported) == set(runtime.params)
    runtime.params.update(exported)

    scans = torch.rand(2, 6, 750) * 30.0
    speeds = torch.rand(2, 6, 1) * 5.0
    with torch.no_grad():
        expected = model.forward_correction_components(scans, speeds)
    actual_steering = []
    actual_correction = []
    actual_base = []
    hidden = None
    shared_hidden = None
    for index in range(scans.shape[1]):
        normalized_scan = scans[:, index].numpy()[:, None, :] / 30.0
        steering, correction, raw, base, hidden = runtime.forward_components(
            normalized_scan,
            speeds[:, index, 0].numpy(),
            hidden,
        )
        conv5 = runtime._conv5_features(normalized_scan, 'base_')
        shared_correction, shared_raw, shared_hidden = (
            runtime.forward_correction_from_conv5_features(
                conv5,
                speeds[:, index, 0].numpy(),
                shared_hidden,
            )
        )
        np.testing.assert_array_equal(shared_correction, correction)
        np.testing.assert_array_equal(shared_raw, raw)
        np.testing.assert_array_equal(shared_hidden, hidden)
        actual_steering.append(steering)
        actual_correction.append(correction)
        actual_base.append(base)
        np.testing.assert_allclose(correction, raw, rtol=0.0, atol=0.0)

    for expected_value, actual_values in (
        (expected[0], actual_steering),
        (expected[1], actual_correction),
        (expected[2], actual_base),
    ):
        actual = np.stack(actual_values, axis=1)
        np.testing.assert_allclose(
            actual, expected_value.numpy(), rtol=3e-5, atol=3e-5
        )
    np.testing.assert_allclose(
        hidden, expected[-1].squeeze(0).numpy(), rtol=3e-5, atol=3e-5
    )
    assert report["parameter_count"] == len(runtime.params)
    assert manifest.is_file()


def test_runtime_deadband_suppresses_raw_recurrent_correction() -> None:
    runtime = RecurrentSteeringAdapterNp(
        input_dim=750,
        hidden_dim=4,
        projection_dim=6,
        speed_embedding_dim=3,
        correction_deadband_rad=0.02,
        spatial_baseline_hidden_dim=8,
        spatial_baseline_projection_dim=5,
    )
    runtime.params["spatial_scale"].fill(1.0)
    runtime.params["spatial_baseline_spatial_scale"].fill(1.0)
    runtime.params["correction_output_bias"].fill(
        np.arctanh(0.01 / runtime.max_abs_correction_rad)
    )
    steering, applied, raw, base, hidden = runtime.forward_components(
        np.ones((1, 1, 750), dtype=np.float32),
        np.ones(1, dtype=np.float32),
    )

    assert raw[0] == pytest.approx(0.01, abs=1e-6)
    assert applied[0] == 0.0
    np.testing.assert_allclose(steering, base, rtol=0.0, atol=0.0)
    assert hidden.shape == (1, 4)


def test_converter_rejects_non_direct_recurrent_head(tmp_path: Path) -> None:
    config = recurrent_config(correction_head="signed_expert")
    pytorch_config = dict(config)
    pytorch_config.pop("model_type")
    model = FrozenTinyLidarRecurrentAdapter(**pytorch_config)
    checkpoint = tmp_path / "signed.pth"
    torch.save(
        {"model_config": config, "model_state_dict": model.state_dict()},
        checkpoint,
    )

    with pytest.raises(ValueError, match="supports direct correction"):
        convert_recurrent_checkpoint(
            checkpoint,
            tmp_path / "signed.npy",
            tmp_path / "signed.json",
            correction_deadband_rad=0.02,
        )
