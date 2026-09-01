import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from evaluate_recurrent_policy import infer_sequence
from build_recurrent_dataset import (
    executed_teacher_mode,
    iter_source_sequences,
    load_embedded_causal_speed,
    load_physical_source_scans,
    longest_true_run,
    recurrent_label_source,
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


def write_sequence(
    root: Path,
    split: str,
    sequence_id: str,
    count: int = 9,
    label_source: str = "lidar_precontact_teacher_recurrent_direct",
) -> Path:
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
        "label_source": label_source,
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
    assert first != recurrent_sequence_id(
        "run-a", "/speed", 0.05, "a" * 64
    )


def test_recurrent_loader_requires_explicit_looser_speed_contract(
    tmp_path: Path,
) -> None:
    sequence = write_sequence(tmp_path, "train", "runtime-speed-contract")
    metadata_path = sequence / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    metadata["max_speed_sync_delta_sec"] = 0.1
    metadata_path.write_text(json.dumps(metadata))

    with pytest.raises(ValueError, match="speed sync contract too loose"):
        MultiSeqRecurrentPolicyDataset(tmp_path / "train", "train")

    loaded = MultiSeqRecurrentPolicyDataset(
        tmp_path / "train",
        "train",
        max_speed_sync_delta_sec=0.1,
    )
    assert loaded.sequence_ids == ["runtime-speed-contract"]


def test_recurrent_identity_preserves_teacher_provenance() -> None:
    assert recurrent_label_source("lidar_precontact_teacher_dagger") == (
        "lidar_precontact_teacher_recurrent_direct"
    )
    assert recurrent_label_source("lidar_speed_committed_teacher_dagger") == (
        "lidar_speed_committed_teacher_recurrent_direct"
    )
    assert executed_teacher_mode("lidar_speed_committed_teacher_dagger") == (
        "speed_committed_teacher"
    )
    with pytest.raises(ValueError, match="unsupported recurrent source"):
        recurrent_label_source("lidar_gap_teacher_dagger")


class EmbeddedSpeedSource:
    def __init__(
        self,
        seq_dir: Path,
        active_only: bool = False,
        novel_policy_only: bool = False,
    ):
        self.seq_dir = seq_dir
        self.scan_timestamps_ns = np.array(
            [100_000_000, 200_000_000, 300_000_000], dtype=np.int64
        )
        self.metadata = {
            "relabeling": {
                "control_mode": "speed_committed_teacher",
                "active_only": active_only,
                "novel_policy_only": novel_policy_only,
            },
            "speed_sync": {
                "policy": "latest_preceding",
                "max_delta_sec": 0.05,
            },
        }

    def __len__(self) -> int:
        return len(self.scan_timestamps_ns)


def write_embedded_speed(source: EmbeddedSpeedSource) -> None:
    speed_times = source.scan_timestamps_ns - 10_000_000
    np.save(source.seq_dir / "speeds.npy", np.array([1.0, 2.0, 3.0]))
    np.save(source.seq_dir / "speed_timestamps_ns.npy", speed_times)
    np.save(
        source.seq_dir / "speed_sync_deltas_sec.npy",
        np.full(3, 0.01),
    )


def test_embedded_causal_speed_preserves_executed_sequence(tmp_path: Path) -> None:
    source = EmbeddedSpeedSource(tmp_path)
    write_embedded_speed(source)
    speeds, speed_times, deltas = load_embedded_causal_speed(source, 0.05)
    np.testing.assert_array_equal(speeds, [1.0, 2.0, 3.0])
    assert np.all(speed_times <= source.scan_timestamps_ns)
    np.testing.assert_allclose(deltas, 0.01)


def test_embedded_causal_speed_rejects_temporally_filtered_source(
    tmp_path: Path,
) -> None:
    source = EmbeddedSpeedSource(tmp_path, active_only=True)
    write_embedded_speed(source)
    with pytest.raises(ValueError, match="retain every temporal sample"):
        load_embedded_causal_speed(source, 0.05)

    source = EmbeddedSpeedSource(tmp_path, novel_policy_only=True)
    with pytest.raises(ValueError, match="retain every temporal sample"):
        load_embedded_causal_speed(source, 0.05)


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


def test_partial_roots_must_be_split_complete_in_aggregate(
    tmp_path: Path,
) -> None:
    primary = tmp_path / "primary"
    write_source_identity(primary, "train", "primary-train")

    with pytest.raises(FileNotFoundError, match="aggregate source dataset"):
        list(
            iter_source_sequences(
                [primary], allow_partial_additional_roots=True
            )
        )

    validation = tmp_path / "validation"
    write_source_identity(validation, "val", "validation-val")
    discovered = list(
        iter_source_sequences(
            [primary, validation], allow_partial_additional_roots=True
        )
    )
    assert [(split, sequence.name) for _, split, sequence in discovered] == [
        ("train", "primary-train"),
        ("val", "validation-val"),
    ]


def test_validation_only_source_requires_explicit_split_selection(
    tmp_path: Path,
) -> None:
    validation = tmp_path / "validation"
    write_source_identity(validation, "val", "validation-val")

    with pytest.raises(FileNotFoundError, match="aggregate source dataset"):
        list(
            iter_source_sequences(
                [validation], allow_partial_additional_roots=True
            )
        )

    discovered = list(
        iter_source_sequences(
            [validation],
            allow_partial_additional_roots=True,
            required_splits=("val",),
        )
    )
    assert [(split, sequence.name) for _, split, sequence in discovered] == [
        ("val", "validation-val"),
    ]


def test_required_split_selection_rejects_invalid_or_duplicate_values(
    tmp_path: Path,
) -> None:
    root = tmp_path / "source"
    write_source_identity(root, "val", "validation-val")

    for required_splits in ((), ("val", "val"), ("test",)):
        with pytest.raises(ValueError, match="required splits"):
            list(iter_source_sequences([root], required_splits=required_splits))


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


def test_sequence_dataset_accepts_distinct_speed_teacher_source(
    tmp_path: Path,
) -> None:
    sequence = write_sequence(
        tmp_path,
        "val",
        "speed-sequence",
        label_source="lidar_speed_committed_teacher_recurrent_direct",
    )
    dataset = RecurrentPolicySequenceDataset(sequence, expected_split="val")
    assert dataset.label_source == "lidar_speed_committed_teacher_recurrent_direct"


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


def test_recurrent_deadband_decodes_raw_correction_before_output_clamp() -> None:
    model = FrozenTinyLidarRecurrentAdapter(
        input_dim=750,
        speed_embedding_dim=2,
        hidden_dim=4,
    )
    with torch.no_grad():
        model.base.fc4.weight.zero_()
        model.base.fc4.bias.zero_()
        model.base.fc4.bias[1] = 4.0
        model.correction_output.weight.zero_()
        model.correction_output.bias.fill_(
            float(np.arctanh(0.01 / model.max_abs_correction_rad))
        )
    sequence = SimpleNamespace(
        sequence_id="deadband-clamp",
        scans=np.full((2, 750), 30.0, dtype=np.float32),
        speeds=np.ones(2, dtype=np.float32),
        steers=np.ones(2, dtype=np.float32),
        base_steers=np.zeros(2, dtype=np.float32),
    )

    retained, base, classes = infer_sequence(
        model, sequence, torch.device("cpu"), correction_deadband_rad=0.005
    )
    suppressed, _, _ = infer_sequence(
        model, sequence, torch.device("cpu"), correction_deadband_rad=0.02
    )

    np.testing.assert_allclose(retained, 1.0, rtol=0.0, atol=0.0)
    np.testing.assert_allclose(suppressed, base, rtol=0.0, atol=0.0)
    assert classes is None


def test_projected_conv5_adapter_preserves_base_and_spatial_geometry() -> None:
    torch.manual_seed(11)
    model = FrozenTinyLidarRecurrentAdapter(
        input_dim=750,
        speed_embedding_dim=4,
        hidden_dim=8,
        spatial_features="projected_conv5",
        spatial_projection_dim=16,
        spatial_projection_seed=2034,
        spatial_normalization="fixed_train_statistics",
        use_speed=False,
    )
    scans = torch.rand(2, 3, 750) * 30.0
    speeds = torch.rand(2, 3, 1) * 5.0
    projected = model.projected_spatial_features(scans)
    assert projected.shape == (2, 3, 16)
    scale = torch.std(projected.reshape(-1, 16), dim=0).clamp(min=1e-4)
    model.set_spatial_statistics(
        torch.mean(projected.reshape(-1, 16), dim=0), scale
    )

    predictions, corrections, base, hidden = model.forward_components(
        scans, speeds
    )

    torch.testing.assert_close(corrections, torch.zeros_like(corrections))
    torch.testing.assert_close(predictions, base, rtol=0.0, atol=0.0)
    assert hidden.shape == (1, 2, 8)
    assert model.gru.input_size == 16


def test_projected_conv5_adapter_checkpoint_round_trip() -> None:
    config = {
        "input_dim": 750,
        "speed_embedding_dim": 3,
        "hidden_dim": 7,
        "spatial_features": "projected_conv5",
        "spatial_projection_dim": 12,
        "spatial_projection_seed": 99,
        "spatial_normalization": "fixed_train_statistics",
        "use_speed": True,
    }
    source = FrozenTinyLidarRecurrentAdapter(**config)
    source.set_spatial_statistics(torch.arange(12.0), torch.arange(1.0, 13.0))
    restored = FrozenTinyLidarRecurrentAdapter(**config)
    restored.load_state_dict(source.state_dict(), strict=True)

    scans = torch.rand(1, 2, 750) * 30.0
    speeds = torch.rand(1, 2, 1)
    expected, _ = source(scans, speeds)
    actual, _ = restored(scans, speeds)
    torch.testing.assert_close(actual, expected, rtol=0.0, atol=0.0)
    torch.testing.assert_close(restored.spatial_mean, torch.arange(12.0))


def test_recurrent_zero_correction_preserves_full_spatial_baseline() -> None:
    spatial_config = {
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
    }
    model = FrozenTinyLidarRecurrentAdapter(
        input_dim=750,
        speed_embedding_dim=3,
        hidden_dim=7,
        spatial_features="projected_conv5",
        spatial_projection_dim=12,
        spatial_normalization="fixed_train_statistics",
        frozen_spatial_baseline_config=spatial_config,
    )
    scans = torch.rand(1, 2, 750) * 30.0
    speeds = torch.rand(1, 2, 1) * 4.0
    predictions, corrections, full_base, _ = model.forward_components(
        scans, speeds
    )

    with torch.no_grad():
        raw = model.base((scans / 30.0).reshape(2, 1, 750))[:, 1]
        spatial = model.spatial_baseline(
            scans.reshape(2, 750), speeds.reshape(2)
        )
        expected = torch.clamp(raw + spatial, -1.0, 1.0).reshape(1, 2)
    torch.testing.assert_close(full_base, expected)
    torch.testing.assert_close(model.base_steering(scans, speeds), expected)
    torch.testing.assert_close(corrections, torch.zeros_like(corrections))
    torch.testing.assert_close(predictions, full_base, rtol=0.0, atol=0.0)
    assert all(
        not parameter.requires_grad
        for parameter in model.spatial_baseline.parameters()
    )


def test_projected_conv5_rejects_duplicate_pressure_representation() -> None:
    with pytest.raises(ValueError, match="must not duplicate"):
        FrozenTinyLidarRecurrentAdapter(
            spatial_features="projected_conv5",
            spatial_normalization="fixed_train_statistics",
            include_pressure_tokens=True,
        )


def test_signed_expert_adapter_starts_with_exact_neutral_correction() -> None:
    model = FrozenTinyLidarRecurrentAdapter(
        input_dim=750,
        speed_embedding_dim=3,
        hidden_dim=7,
        spatial_features="projected_conv5",
        spatial_projection_dim=12,
        spatial_normalization="fixed_train_statistics",
        correction_head="signed_expert",
    )
    scans = torch.rand(2, 3, 750) * 30.0
    speeds = torch.rand(2, 3, 1)
    (
        predictions,
        corrections,
        base,
        magnitudes,
        direction_logits,
        hidden,
    ) = model.forward_correction_components(scans, speeds)

    assert magnitudes.shape == (2, 3, 2)
    assert direction_logits.shape == (2, 3, 3)
    assert torch.all(torch.argmax(direction_logits, dim=-1) == 1)
    torch.testing.assert_close(corrections, torch.zeros_like(corrections))
    torch.testing.assert_close(predictions, base, rtol=0.0, atol=0.0)
    assert hidden.shape == (1, 2, 7)
