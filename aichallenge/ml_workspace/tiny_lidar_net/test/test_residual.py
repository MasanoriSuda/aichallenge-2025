import json
from pathlib import Path

import numpy as np
import pytest
import torch

from lib.residual import (
    RESIDUAL_INPUT_MODES,
    SignedMixtureSteeringResidualNet,
    SteeringResidualNet,
    SteeringResidualSequenceDataset,
    compose_residual_input,
    residual_metrics,
    residual_input_channels,
    residual_training_loss,
    sequence_balanced_sample_weights,
    signed_direction_targets,
    signed_mixture_training_loss,
    weighted_residual_smooth_l1,
)
from tiny_lidar_net_controller.model.tinylidarnet import SteeringResidualNetNp


def make_sequence(root: Path, *, corrupt_delta: bool = False) -> Path:
    sequence = root / "paired-sequence"
    sequence.mkdir()
    scans = np.full((2, 750), 15.0, dtype=np.float32)
    successor = np.array([0.10, -0.30], dtype=np.float32)
    base = np.array([0.10, -0.10], dtype=np.float32)
    # The historical gap teacher remains diagnostic provenance.  Runtime does
    # not execute it, so it may not define the learned residual baseline.
    reference = np.array([0.10, -0.25], dtype=np.float32)
    delta = successor - base
    if corrupt_delta:
        delta[1] = 0.3
    timestamps = np.array([1, 2], dtype=np.int64)
    arrays = {
        "scans.npy": scans,
        "steers.npy": successor,
        "accelerations.npy": np.full(2, 0.6, dtype=np.float32),
        "delta_times.npy": np.zeros(2, dtype=np.float64),
        "scan_timestamps_ns.npy": timestamps,
        "control_timestamps_ns.npy": timestamps,
        "base_steers.npy": base,
        "reference_steers.npy": reference,
        "steering_deltas.npy": delta,
    }
    for name, values in arrays.items():
        np.save(sequence / name, values)
    metadata = {
        "schema_version": 1,
        "sequence_id": sequence.name,
        "split": "train",
        "label_source": "lidar_precontact_teacher_dagger",
        "scan_shape": [750],
        "max_scan_range_m": 30.0,
        "max_sync_delta_sec": 0.0,
        "counts": {"accepted_samples": 2},
        "relabeling": {
            "residual_target": {
                "target_definition": (
                    "successor_steering_minus_base_steering"
                ),
                "runtime_composition": "base_steering_plus_learned_residual",
                "successor_teacher": "LidarPrecontactTeacher",
                "base_policy": "frozen_production_tiny_lidar_net",
                "material_delta_rad": 0.02,
            }
        },
    }
    (sequence / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )
    return sequence


def test_residual_dataset_validates_runtime_composition_identity(tmp_path: Path):
    dataset = SteeringResidualSequenceDataset(
        make_sequence(tmp_path), expected_split="train"
    )
    scan, target = dataset[1]
    assert scan.shape == (750,)
    assert target == pytest.approx(-0.2)


def test_scan_delta_dataset_uses_only_previous_sample_in_same_sequence(
    tmp_path: Path,
):
    sequence = make_sequence(tmp_path)
    scans = np.load(sequence / "scans.npy")
    scans[0] = 6.0
    scans[1] = 12.0
    np.save(sequence / "scans.npy", scans)
    dataset = SteeringResidualSequenceDataset(
        sequence, expected_split="train", input_mode="scan_delta"
    )
    first, _ = dataset[0]
    second, _ = dataset[1]
    assert first.shape == (2, 750)
    np.testing.assert_allclose(first[0], 0.2)
    np.testing.assert_allclose(first[1], 0.0)
    np.testing.assert_allclose(second[0], 0.4)
    np.testing.assert_allclose(second[1], 0.2)


def test_residual_input_mode_contract_is_explicit():
    assert RESIDUAL_INPUT_MODES == ("stateless", "scan_delta")
    assert residual_input_channels("stateless") == 1
    assert residual_input_channels("scan_delta") == 2
    with pytest.raises(ValueError, match="unsupported residual input mode"):
        compose_residual_input(np.zeros(2), np.zeros(2), "history-ish")


def test_residual_target_does_not_use_diagnostic_reference_teacher(tmp_path: Path):
    dataset = SteeringResidualSequenceDataset(
        make_sequence(tmp_path), expected_split="train"
    )
    # Successor-reference is only -0.05.  Runtime is base+residual, so the
    # auditable target must instead be successor-base == -0.20.
    assert dataset.steering_deltas[1] == pytest.approx(-0.2)
    assert (
        dataset.steers[1] - dataset.reference_steers[1]
    ) == pytest.approx(-0.05)


def test_residual_dataset_rejects_inconsistent_delta(tmp_path: Path):
    with pytest.raises(ValueError, match="residual identity mismatch"):
        SteeringResidualSequenceDataset(
            make_sequence(tmp_path, corrupt_delta=True), expected_split="train"
        )


def test_residual_model_is_exactly_zero_before_training():
    model = SteeringResidualNet(input_dim=750)
    output = model(torch.rand(4, 1, 750))
    torch.testing.assert_close(output, torch.zeros(4), rtol=0.0, atol=0.0)


def test_signed_mixture_is_exactly_zero_before_training():
    model = SignedMixtureSteeringResidualNet(input_dim=750, input_channels=2)
    output = model(torch.rand(4, 2, 750))
    torch.testing.assert_close(output, torch.zeros(4), rtol=0.0, atol=0.0)


def test_signed_direction_targets_keep_opposing_actions_separate():
    classes = signed_direction_targets(
        torch.tensor([-0.2, -0.01, 0.0, 0.01, 0.2]),
        material_delta_rad=0.02,
    )
    assert classes.tolist() == [0, 1, 1, 1, 2]


def test_signed_mixture_loss_prefers_correct_direction_classes():
    targets = torch.tensor([-0.3, 0.0, 0.4])
    magnitudes = torch.tensor([[0.3, 0.1], [0.1, 0.1], [0.1, 0.4]])
    correct_logits = torch.tensor(
        [[5.0, 0.0, -5.0], [0.0, 5.0, 0.0], [-5.0, 0.0, 5.0]]
    )
    wrong_logits = torch.flip(correct_logits, dims=(1,))
    weights = torch.ones(3)
    correct = signed_mixture_training_loss(
        residual=torch.tensor([-0.3, 0.0, 0.4]),
        magnitudes=magnitudes,
        direction_logits=correct_logits,
        targets=targets,
        material_delta_rad=0.02,
        material_weight=20.0,
        direction_class_weights=weights,
        direction_loss_weight=1.0,
        anchor_leakage_weight=0.5,
    )
    wrong = signed_mixture_training_loss(
        residual=torch.tensor([0.3, 0.0, -0.4]),
        magnitudes=magnitudes,
        direction_logits=wrong_logits,
        targets=targets,
        material_delta_rad=0.02,
        material_weight=20.0,
        direction_class_weights=weights,
        direction_loss_weight=1.0,
        anchor_leakage_weight=0.5,
    )
    assert correct < wrong


def test_numpy_runtime_matches_torch_residual_model():
    torch.manual_seed(7)
    model = SteeringResidualNet(input_dim=750)
    with torch.no_grad():
        model.correction_head.weight.normal_(0.0, 0.1)
        model.gate_head.weight.normal_(0.0, 0.1)
    runtime = SteeringResidualNetNp(input_dim=750)
    runtime.params.update({
        key.replace(".", "_"): value.detach().numpy()
        for key, value in model.state_dict().items()
    })
    sample = np.linspace(0.0, 1.0, 750, dtype=np.float32)[None, None, :]
    expected = model(torch.from_numpy(sample)).detach().numpy()
    actual = runtime(sample).reshape(-1)
    np.testing.assert_allclose(actual, expected, rtol=2e-4, atol=2e-5)


def test_numpy_runtime_matches_torch_scan_delta_residual_model():
    torch.manual_seed(11)
    model = SteeringResidualNet(input_dim=750, input_channels=2)
    with torch.no_grad():
        model.correction_head.weight.normal_(0.0, 0.1)
        model.gate_head.weight.normal_(0.0, 0.1)
    runtime = SteeringResidualNetNp(input_dim=750, input_channels=2)
    runtime.params.update({
        key.replace(".", "_"): value.detach().numpy()
        for key, value in model.state_dict().items()
    })
    current = np.linspace(0.0, 1.0, 750, dtype=np.float32)
    previous = np.linspace(0.1, 0.9, 750, dtype=np.float32)
    sample = compose_residual_input(
        current, previous, "scan_delta"
    )[None, :, :]
    expected = model(torch.from_numpy(sample)).detach().numpy()
    actual = runtime(sample).reshape(-1)
    np.testing.assert_allclose(actual, expected, rtol=2e-4, atol=2e-5)


def test_material_weight_preserves_anchor_but_amplifies_correction():
    predictions = torch.tensor([0.0, 0.0])
    targets = torch.tensor([0.0, 0.4])
    unweighted = weighted_residual_smooth_l1(
        predictions, targets, material_delta_rad=0.02, material_weight=1.0
    )
    weighted = weighted_residual_smooth_l1(
        predictions, targets, material_delta_rad=0.02, material_weight=20.0
    )
    assert weighted > unweighted


def test_learned_gate_penalizes_false_activation_on_anchor():
    targets = torch.tensor([0.0, 0.2])
    quiet = residual_training_loss(
        residual=torch.tensor([0.0, 0.1]),
        correction=torch.tensor([0.0, 0.2]),
        gate_logits=torch.tensor([-5.0, 0.0]),
        targets=targets,
        material_delta_rad=0.02,
        material_weight=20.0,
        gate_loss_weight=0.01,
        anchor_leakage_weight=0.5,
    )
    leaking = residual_training_loss(
        residual=torch.tensor([0.1, 0.1]),
        correction=torch.tensor([0.2, 0.2]),
        gate_logits=torch.tensor([0.0, 0.0]),
        targets=targets,
        material_delta_rad=0.02,
        material_weight=20.0,
        gate_loss_weight=0.01,
        anchor_leakage_weight=0.5,
    )
    assert quiet < leaking


def test_residual_metrics_separate_material_and_anchor_samples():
    metrics = residual_metrics(
        predictions=np.array([0.0, 0.1, -0.1]),
        targets=np.array([0.0, 0.2, -0.2]),
        material_delta_rad=0.02,
    )
    assert metrics["anchor"]["samples"] == 1
    assert metrics["anchor"]["mae_rad"] == pytest.approx(0.0)
    assert metrics["material"]["samples"] == 2
    assert metrics["material"]["mean_prediction_rad"] == pytest.approx(0.0)
    assert metrics["material"]["mean_target_rad"] == pytest.approx(0.0)
    assert metrics["material_mae_improvement_fraction"] == pytest.approx(0.5)


def test_sequence_balanced_weights_give_each_run_equal_probability_mass():
    weights = sequence_balanced_sample_weights([2, 8, 20]).numpy()
    assert weights.shape == (30,)
    assert np.sum(weights[:2]) == pytest.approx(1.0 / 3.0)
    assert np.sum(weights[2:10]) == pytest.approx(1.0 / 3.0)
    assert np.sum(weights[10:]) == pytest.approx(1.0 / 3.0)


@pytest.mark.parametrize("lengths", [[], [0], [2, -1]])
def test_sequence_balanced_weights_reject_invalid_lengths(lengths):
    with pytest.raises(ValueError, match="non-empty and positive"):
        sequence_balanced_sample_weights(lengths)
