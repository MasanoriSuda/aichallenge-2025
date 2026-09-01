import json
from pathlib import Path

import numpy as np
import pytest
import torch

from build_recurrent_dataset import (
    iter_source_sequences,
    load_physical_source_scans,
    longest_true_run,
    recurrent_sequence_id,
)
from lib.recurrent_policy import (
    FrozenTinyLidarRecurrentAdapter,
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
        "scan_unit": "m",
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


def write_source_identity(root: Path, split: str, sequence_id: str) -> Path:
    sequence = root / split / sequence_id
    sequence.mkdir(parents=True)
    (sequence / "metadata.json").write_text(
        json.dumps({"sequence_id": sequence_id}), encoding="utf-8"
    )
    return sequence


def test_multiple_source_roots_preserve_unique_sequence_identity(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"
    for root, prefix in ((first, "a"), (second, "b")):
        write_source_identity(root, "train", f"{prefix}-train")
        write_source_identity(root, "val", f"{prefix}-val")

    discovered = list(iter_source_sequences([first, second]))

    assert [item[2].name for item in discovered] == [
        "a-train",
        "a-val",
        "b-train",
        "b-val",
    ]


def test_additional_source_may_be_train_only_when_explicitly_allowed(
    tmp_path: Path,
) -> None:
    primary = tmp_path / "primary"
    additional = tmp_path / "additional"
    write_source_identity(primary, "train", "primary-train")
    write_source_identity(primary, "val", "primary-val")
    write_source_identity(additional, "train", "additional-train")

    discovered = list(
        iter_source_sequences(
            [primary, additional], allow_partial_additional_roots=True
        )
    )

    assert [item[2].name for item in discovered] == [
        "primary-train",
        "primary-val",
        "additional-train",
    ]


def test_primary_source_still_requires_both_splits_in_partial_mode(
    tmp_path: Path,
) -> None:
    primary = tmp_path / "primary"
    write_source_identity(primary, "train", "primary-train")

    with pytest.raises(FileNotFoundError, match="missing source split"):
        list(
            iter_source_sequences(
                [primary], allow_partial_additional_roots=True
            )
        )


def test_multiple_source_roots_reject_duplicate_sequence_identity(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"
    for root in (first, second):
        write_source_identity(root, "train", "shared")
        write_source_identity(root, "val", f"unique-{root.name}")

    with pytest.raises(ValueError, match="duplicate source sequence identity shared"):
        list(iter_source_sequences([first, second]))


def test_source_exclusion_is_explicit_and_must_match(tmp_path: Path) -> None:
    root = tmp_path / "source"
    write_source_identity(root, "train", "keep-train")
    write_source_identity(root, "val", "exclude-val")

    discovered = list(iter_source_sequences([root], ["exclude-val"]))
    assert [item[2].name for item in discovered] == ["keep-train"]

    with pytest.raises(ValueError, match="was not found"):
        list(iter_source_sequences([root], ["missing"]))

    duplicate = tmp_path / "duplicate-source"
    write_source_identity(duplicate, "val", "exclude-val")
    with pytest.raises(ValueError, match="is ambiguous"):
        list(
            iter_source_sequences(
                [root, duplicate],
                ["exclude-val"],
                allow_partial_additional_roots=True,
            )
        )


def test_physical_scan_loader_rejects_normalization_mismatch(tmp_path: Path) -> None:
    physical = np.array([[0.0, 15.0, 30.0]], dtype=np.float32)
    np.save(tmp_path / "scans.npy", physical)
    loaded = load_physical_source_scans(tmp_path, physical / 30.0, 30.0)
    np.testing.assert_array_equal(loaded, physical)
    with pytest.raises(ValueError, match="physical/normalized scan identity"):
        load_physical_source_scans(tmp_path, physical, 30.0)


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


def test_sequence_dataset_rejects_unproven_scan_units(tmp_path: Path) -> None:
    sequence = write_sequence(tmp_path, "train", "sequence-a")
    metadata_path = sequence / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    metadata.pop("scan_unit")
    metadata_path.write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="scan unit must be metres"):
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


def test_frozen_adapter_starts_exactly_at_base_policy() -> None:
    torch.manual_seed(7)
    model = FrozenTinyLidarRecurrentAdapter(
        input_dim=750, speed_embedding_dim=4, hidden_dim=8
    )
    scans = torch.rand(2, 3, 750) * 30.0
    speeds = torch.rand(2, 3, 1) * 5.0
    predictions, corrections, base, hidden = model.forward_components(
        scans, speeds
    )
    torch.testing.assert_close(corrections, torch.zeros_like(corrections))
    torch.testing.assert_close(predictions, base, rtol=0.0, atol=0.0)
    torch.testing.assert_close(model.base_steering(scans), base)
    assert hidden.shape == (1, 2, 8)
    assert all(not parameter.requires_grad for parameter in model.base.parameters())


def test_pressure_adapter_preserves_initial_base_identity() -> None:
    model = FrozenTinyLidarRecurrentAdapter(
        input_dim=750,
        speed_embedding_dim=2,
        hidden_dim=4,
        include_pressure_tokens=True,
    )
    scans = torch.linspace(0.0, 30.0, 750).view(1, 1, 750)
    speeds = torch.tensor([[[2.0]]])
    predictions, corrections, base, _ = model.forward_components(scans, speeds)
    torch.testing.assert_close(corrections, torch.zeros_like(corrections))
    torch.testing.assert_close(predictions, base, rtol=0.0, atol=0.0)
    pressure = model.pressure_tokens(scans)
    assert torch.all(pressure[..., :-1] >= pressure[..., 1:])
