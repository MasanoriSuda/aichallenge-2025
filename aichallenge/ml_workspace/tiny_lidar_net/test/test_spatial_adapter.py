from pathlib import Path
import sys

import pytest
import numpy as np
import torch

from lib.spatial_adapter import FrozenTinyLidarSpatialResidual
from lib.residual import save_numpy_state
from train_spatial_adapter import ZeroResidualAnchorSequence


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
    SpatialSteeringAdapterNp,
)


def make_model() -> FrozenTinyLidarSpatialResidual:
    torch.manual_seed(2026)
    return FrozenTinyLidarSpatialResidual(input_dim=750, hidden_dim=16)


def test_untrained_spatial_adapter_is_exact_zero_for_every_scan():
    model = make_model()
    scans = torch.rand(4, 750) * 30.0

    residual, magnitudes, logits, probabilities = model.forward_components(scans)

    assert torch.equal(residual, torch.zeros_like(residual))
    assert magnitudes.shape == (4, 2)
    assert logits.shape == (4, 3)
    assert probabilities.shape == (4, 3)


def test_spatial_adapter_freezes_every_base_parameter():
    model = make_model()

    assert all(not parameter.requires_grad for parameter in model.base.parameters())
    assert any(
        parameter.requires_grad
        for name, parameter in model.named_parameters()
        if not name.startswith("base.")
    )


def test_spatial_adapter_validates_physical_scan_contract():
    model = make_model()

    with pytest.raises(ValueError, match="shape"):
        model(torch.zeros(2, 749))
    invalid = torch.zeros(2, 750)
    invalid[0, 0] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        model(invalid)


def test_speed_enabled_adapter_requires_valid_aligned_speed():
    model = FrozenTinyLidarSpatialResidual(
        input_dim=750, hidden_dim=16, use_speed=True
    )
    scans = torch.rand(3, 750) * 30.0

    with pytest.raises(ValueError, match="one speed per scan"):
        model(scans)
    with pytest.raises(ValueError, match="non-negative"):
        model(scans, torch.tensor([1.0, -0.1, 2.0]))
    assert torch.equal(
        model(scans, torch.tensor([1.0, 2.0, 3.0])),
        torch.zeros(3),
    )


def test_base_steering_conditioning_is_frozen_and_shape_explicit():
    model = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=16,
        projection_dim=32,
        use_base_steering=True,
    )
    scans = torch.rand(3, 750) * 30.0

    assert model.spatial_head[0].in_features == 33
    assert model.base_steering(scans).shape == (3,)
    assert torch.equal(model(scans), torch.zeros(3))
    assert all(not parameter.requires_grad for parameter in model.base.parameters())


def test_fixed_train_statistics_are_explicit_and_immutable_state():
    model = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=16,
        spatial_normalization="fixed_train_statistics",
    )
    mean = torch.linspace(0.0, 1.0, model.spatial_dim)
    scale = torch.linspace(0.5, 1.5, model.spatial_dim)

    model.set_spatial_statistics(mean, scale)

    assert torch.equal(model.spatial_mean, mean)
    assert torch.equal(model.spatial_scale, scale)
    assert "spatial_mean" in model.state_dict()
    assert "spatial_scale" in model.state_dict()
    with pytest.raises(ValueError, match="positive"):
        model.set_spatial_statistics(mean, torch.zeros_like(scale))


def test_random_projection_is_seeded_frozen_candidate_state():
    first = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=16,
        projection_dim=32,
        projection_seed=17,
    )
    second = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=16,
        projection_dim=32,
        projection_seed=17,
    )
    different = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=16,
        projection_dim=32,
        projection_seed=18,
    )

    assert first.representation_dim == 32
    assert first.spatial_projection.shape == (first.spatial_dim, 32)
    assert torch.equal(first.spatial_projection, second.spatial_projection)
    assert not torch.equal(first.spatial_projection, different.spatial_projection)
    assert not any(
        name == "spatial_projection"
        for name, _ in first.named_parameters()
    )
    assert "spatial_projection" in first.state_dict()


def test_factorized_head_starts_at_exact_zero_with_neutral_factorization():
    model = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=16,
        head_architecture="factorized_gate",
    )
    scans = torch.rand(4, 750) * 30.0

    residual, magnitudes, logits, probabilities = model.forward_components(scans)

    assert torch.equal(residual, torch.zeros_like(residual))
    assert magnitudes.shape == (4, 2)
    assert logits.shape == (4, 3)
    assert probabilities[:, 1].tolist() == pytest.approx([0.5] * 4)
    assert probabilities[:, 0].tolist() == pytest.approx([0.25] * 4)
    assert probabilities[:, 2].tolist() == pytest.approx([0.25] * 4)
    assert "activation_head.weight" in model.state_dict()
    assert "sign_head.weight" in model.state_dict()
    assert "direction_head.weight" not in model.state_dict()


class FakeNormalizedSequence:
    sequence_id = "normal-run"

    def __init__(self):
        self.values = np.full((2, 750), 0.5, dtype=np.float32)

    def __len__(self):
        return len(self.values)

    def __getitem__(self, index):
        return self.values[index], np.asarray([1.0, -0.2], dtype=np.float32)


def test_zero_residual_anchor_ignores_labels_and_restores_metres():
    anchor = ZeroResidualAnchorSequence(FakeNormalizedSequence(), 30.0)

    scan, speed, teacher, base = anchor[0]

    assert anchor.sequence_id == "normal-anchor:normal-run"
    assert scan.tolist() == pytest.approx([15.0] * 750)
    assert (speed, teacher, base) == (0.0, 0.0, 0.0)
    assert anchor.correction_targets().tolist() == [0.0, 0.0]


def test_numpy_spatial_shadow_matches_exported_pytorch_candidate(
    tmp_path: Path,
):
    torch.manual_seed(2042)
    model = FrozenTinyLidarSpatialResidual(
        input_dim=750,
        hidden_dim=16,
        projection_dim=32,
        projection_seed=17,
        use_speed=True,
        use_base_steering=True,
        max_speed_mps=12.0,
        max_abs_delta_rad=1.2,
        spatial_normalization="fixed_train_statistics",
        head_architecture="signed_mixture",
    )
    mean = torch.linspace(-0.2, 0.2, model.representation_dim)
    scale = torch.linspace(0.5, 1.5, model.representation_dim)
    model.set_spatial_statistics(mean, scale)
    with torch.no_grad():
        model.direction_head.weight.normal_(mean=0.0, std=0.05)
        model.direction_head.bias.copy_(torch.tensor([-0.2, 0.1, 0.3]))
        model.magnitude_head.weight.normal_(mean=0.0, std=0.03)
        model.magnitude_head.bias.copy_(torch.tensor([-0.4, 0.2]))
    model.eval()

    checkpoint = tmp_path / "candidate.npy"
    save_numpy_state(model, checkpoint)
    exported = np.load(checkpoint, allow_pickle=True).item()
    runtime = SpatialSteeringAdapterNp(
        input_dim=750,
        hidden_dim=16,
        projection_dim=32,
        use_speed=True,
        use_base_steering=True,
        max_speed_mps=12.0,
        max_abs_delta_rad=1.2,
    )
    assert set(exported) == set(runtime.params)
    runtime.params.update(
        {key: np.asarray(value, dtype=np.float32) for key, value in exported.items()}
    )

    scans_m = torch.rand(3, 750) * 30.0
    speeds_mps = torch.tensor([0.0, 4.2, 13.0], dtype=torch.float32)
    with torch.no_grad():
        expected = model.forward_components(scans_m, speeds_mps)
    actual = runtime.forward_components(
        scans_m.numpy()[:, None, :] / 30.0,
        speeds_mps.numpy(),
    )
    shared_features = runtime._spatial_features(
        scans_m.numpy()[:, None, :] / 30.0
    )
    shared = runtime.forward_components_from_spatial_features(
        shared_features,
        speeds_mps.numpy(),
    )

    for expected_value, actual_value in zip(expected, actual):
        np.testing.assert_allclose(
            actual_value,
            expected_value.numpy(),
            rtol=2e-5,
            atol=2e-5,
        )
    for standalone_value, shared_value in zip(actual, shared):
        np.testing.assert_array_equal(shared_value, standalone_value)
