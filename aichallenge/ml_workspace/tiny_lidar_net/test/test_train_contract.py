import pytest
import torch

from train import select_trainable_parameters


class ToyModel(torch.nn.Module):
    def __init__(self):
        super().__init__()
        self.encoder = torch.nn.Linear(3, 2)
        self.head = torch.nn.Linear(2, 1)


def test_empty_selection_preserves_full_network_training():
    model = ToyModel()
    parameters, names = select_trainable_parameters(model, [])
    assert len(parameters) == 4
    assert set(names) == {
        "encoder.weight",
        "encoder.bias",
        "head.weight",
        "head.bias",
    }
    assert all(parameter.requires_grad for parameter in model.parameters())


def test_selected_layer_freezes_every_other_parameter():
    model = ToyModel()
    parameters, names = select_trainable_parameters(model, ["head"])
    assert len(parameters) == 2
    assert names == ["head.weight", "head.bias"]
    assert not model.encoder.weight.requires_grad
    assert not model.encoder.bias.requires_grad
    assert model.head.weight.requires_grad
    assert model.head.bias.requires_grad


def test_unknown_trainable_layer_is_rejected():
    with pytest.raises(ValueError, match="Unknown trainable layers"):
        select_trainable_parameters(ToyModel(), ["missing"])
