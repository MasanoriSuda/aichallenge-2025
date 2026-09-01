import json
from pathlib import Path

import numpy as np
import pytest
import torch

from build_recurrent_dataset import longest_true_run, recurrent_sequence_id
from lib.recurrent_policy import (
    RECURRENT_DATASET_SCHEMA_VERSION,
    MultiSeqRecurrentPolicyDataset,
    RecurrentDirectSteeringPolicy,
    RecurrentPolicyChunkDataset,
    RecurrentPolicySequenceDataset,
    direct_policy_metrics,
    weighted_direct_policy_smooth_l1,
)


def write_sequence(root: Path, split: str, sequence_id: str, count: int = 9) -> Path:
    sequence = root / split / sequence_id
    sequence.mkdir(parents=True)
    timestamps = np.arange(count, dtype=np.int64) * 100_000_000 + 1
    arrays = {
        "scans.npy": np.full((count, 750), 2.0, dtype=np.float32),
        "speeds.npy": np.linspace(1.0, 2.0, count, dtype=np.float32),
        "steers.npy": np.linspace(-0.2, 0.2, count, dtype=np.float32),
        "base_steers.npy": np.zeros(count, dtype=np.float32),
        "scan_timestamps_ns.npy": timestamps,
        "speed_timestamps_ns.npy": timestamps + 5_000_000,
        "speed_sync_deltas_sec.npy": np.full(count, 0.005),
    }
    for name, values in arrays.items():
        np.save(sequence / name, values)
    metadata = {
        "schema_version": RECURRENT_DATASET_SCHEMA_VERSION,
        "sequence_id": sequence_id,
        "split": split,
        "label_source": "lidar_precontact_teacher_recurrent_direct",
        "scan_shape": [750],
        "max_scan_range_m": 30.0,
        "max_speed_sync_delta_sec": 0.05,
    }
    (sequence / "metadata.json").write_text(json.dumps(metadata))
    return sequence


def test_longest_true_run_does_not_bridge_temporal_holes() -> None:
    assert longest_true_run(np.array([False, True, True, False, True])) == (1, 3)
    assert longest_true_run(np.array([False, False])) == (0, 0)
    assert longest_true_run(np.array([True, True])) == (0, 2)


def test_recurrent_identity_binds_speed_contract() -> None:
    first = recurrent_sequence_id("run-a", "/speed", 0.05)
    assert first == recurrent_sequence_id("run-a", "/speed", 0.05)
    assert first != recurrent_sequence_id("run-a", "/speed", 0.04)
    assert first != recurrent_sequence_id("run-a", "/other-speed", 0.05)


def test_sequence_dataset_enforces_speed_sync_contract(tmp_path: Path) -> None:
    sequence = write_sequence(tmp_path, "train", "sequence-a")
    dataset = RecurrentPolicySequenceDataset(sequence, expected_split="train")
    assert len(dataset) == 9
    scan, speed, steer, base = dataset[0]
    assert scan.shape == (750,)
    assert isinstance(speed, np.float32)
    assert isinstance(steer, np.float32)
    assert isinstance(base, np.float32)

    np.save(sequence / "speed_sync_deltas_sec.npy", np.full(9, 0.051))
    with pytest.raises(ValueError, match="speed sync delta violation"):
        RecurrentPolicySequenceDataset(sequence, expected_split="train")


def test_chunks_never_cross_sequence_boundary(tmp_path: Path) -> None:
    first = RecurrentPolicySequenceDataset(
        write_sequence(tmp_path, "train", "first", 9), expected_split="train"
    )
    second = RecurrentPolicySequenceDataset(
        write_sequence(tmp_path, "train", "second", 10), expected_split="train"
    )
    first_chunks = RecurrentPolicyChunkDataset(first, chunk_length=4, stride=3)
    second_chunks = RecurrentPolicyChunkDataset(second, chunk_length=4, stride=3)
    assert first_chunks.starts == [0, 3, 5]
    assert second_chunks.starts == [0, 3, 6]
    assert first_chunks[2][0].shape == (4, 750)
    assert second_chunks[0][1].shape == (4, 1)


def test_multi_sequence_loader_preserves_run_identity(tmp_path: Path) -> None:
    write_sequence(tmp_path, "val", "first")
    write_sequence(tmp_path, "val", "second")
    loaded = MultiSeqRecurrentPolicyDataset(tmp_path / "val", expected_split="val")
    assert loaded.sequence_ids == ["first", "second"]


def test_pressure_is_monotonic_and_direct_output_is_bounded() -> None:
    pressure_model = RecurrentDirectSteeringPolicy(input_dim=1, hidden_dim=8)
    ranges_over_time = torch.tensor([[[0.0], [1.0], [2.0], [30.0]]])
    pressure = pressure_model.pressure_tokens(ranges_over_time)
    assert torch.all(pressure[:, :-1] >= pressure[:, 1:])

    model = RecurrentDirectSteeringPolicy(input_dim=4, hidden_dim=8)
    scans = torch.tensor([[[0.0, 1.0, 2.0, 30.0]]])
    predictions, hidden = model(scans, torch.tensor([[[3.0]]]))
    assert predictions.shape == (1, 1)
    assert hidden.shape == (1, 1, 8)
    assert torch.all(torch.isfinite(predictions))
    assert torch.all(torch.abs(predictions) <= 1.0)


def test_direct_policy_metrics_separate_anchor_and_material() -> None:
    metrics = direct_policy_metrics(
        predictions=np.array([0.01, 0.25]),
        targets=np.array([0.0, 0.3]),
        base_predictions=np.array([0.0, 0.0]),
        material_delta_rad=0.02,
    )
    assert metrics["anchor"]["samples"] == 1
    assert metrics["material"]["samples"] == 1
    assert metrics["material"]["improvement_fraction"] == pytest.approx(5 / 6)


def test_direct_policy_loss_weights_material_and_honors_burn_in() -> None:
    predictions = torch.tensor([[0.9, 0.0, 0.0]])
    targets = torch.tensor([[0.0, 0.01, 0.3]])
    base = torch.zeros_like(targets)
    loss = weighted_direct_policy_smooth_l1(
        predictions,
        targets,
        base,
        material_delta_rad=0.02,
        material_weight=10.0,
        burn_in_steps=1,
    )
    anchor = torch.nn.functional.smooth_l1_loss(
        torch.tensor(0.0), torch.tensor(0.01), reduction="none"
    )
    material = torch.nn.functional.smooth_l1_loss(
        torch.tensor(0.0), torch.tensor(0.3), reduction="none"
    )
    assert loss == pytest.approx(float((anchor + material * 10.0) / 11.0))
