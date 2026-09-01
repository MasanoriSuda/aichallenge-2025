import pytest
import torch

from lib.spatial_adapter import FrozenTinyLidarSpatialResidual


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
