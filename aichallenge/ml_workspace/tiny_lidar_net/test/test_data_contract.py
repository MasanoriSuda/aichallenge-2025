import json
from pathlib import Path

import numpy as np
import pytest

from lib.data import (
    MultiSeqConcatDataset,
    ScanControlSequenceDataset,
    assert_disjoint_sequence_ids,
)


def write_sequence(
    root: Path,
    sequence_id: str = "teacher-run",
    split: str = "train",
    label_source: str = "mpcc",
    input_dim: int = 750,
    max_delta_sec: float = 0.01,
) -> Path:
    seq_dir = root / sequence_id
    seq_dir.mkdir(parents=True)
    sample_count = 3
    scans = np.full((sample_count, input_dim), 2.0, dtype=np.float32)
    steers = np.array([0.1, 0.2, 0.3], dtype=np.float32)
    accelerations = np.array([0.6, 0.6, 0.6], dtype=np.float32)
    deltas = np.array([0.0, max_delta_sec / 2.0, max_delta_sec], dtype=np.float64)
    scan_timestamps = np.array([1, 2, 3], dtype=np.int64)
    control_timestamps = np.array([1, 2, 3], dtype=np.int64)

    np.save(seq_dir / "scans.npy", scans)
    np.save(seq_dir / "steers.npy", steers)
    np.save(seq_dir / "accelerations.npy", accelerations)
    np.save(seq_dir / "delta_times.npy", deltas)
    np.save(seq_dir / "scan_timestamps_ns.npy", scan_timestamps)
    np.save(seq_dir / "control_timestamps_ns.npy", control_timestamps)
    metadata = {
        "schema_version": 1,
        "sequence_id": sequence_id,
        "split": split,
        "label_source": label_source,
        "scan_shape": [input_dim],
        "max_scan_range_m": 30.0,
        "max_sync_delta_sec": max_delta_sec,
        "counts": {"accepted_samples": sample_count},
    }
    (seq_dir / "metadata.json").write_text(
        json.dumps(metadata), encoding="utf-8"
    )
    return seq_dir


def test_accepts_valid_teacher_sequence(tmp_path: Path) -> None:
    seq_dir = write_sequence(tmp_path)
    dataset = ScanControlSequenceDataset(seq_dir, expected_split="train")
    assert dataset.sequence_id == "teacher-run"
    assert len(dataset) == 3
    scan, target = dataset[0]
    assert scan.shape == (750,)
    assert scan.dtype == np.float32
    np.testing.assert_allclose(target, [0.6, 0.1])


def test_rejects_student_command_as_teacher_by_default(tmp_path: Path) -> None:
    seq_dir = write_sequence(tmp_path, label_source="student")
    with pytest.raises(ValueError, match="Disallowed label source"):
        ScanControlSequenceDataset(seq_dir)


def test_rejects_sync_delta_beyond_training_contract(tmp_path: Path) -> None:
    seq_dir = write_sequence(tmp_path, max_delta_sec=0.06)
    with pytest.raises(ValueError, match="sync contract is too loose"):
        ScanControlSequenceDataset(seq_dir, max_sync_delta_sec=0.05)


def test_rejects_wrong_scan_shape(tmp_path: Path) -> None:
    seq_dir = write_sequence(tmp_path, input_dim=749)
    with pytest.raises(ValueError, match="Metadata scan shape mismatch"):
        ScanControlSequenceDataset(seq_dir, expected_input_dim=750)


def test_concat_dataset_exposes_auditable_sequence_ids(tmp_path: Path) -> None:
    train_root = tmp_path / "train"
    write_sequence(train_root, sequence_id="run-a")
    write_sequence(train_root, sequence_id="run-b")
    dataset = MultiSeqConcatDataset(train_root, expected_split="train")
    assert dataset.sequence_ids == ["run-a", "run-b"]
    assert len(dataset) == 6


def test_rejects_train_validation_run_leakage() -> None:
    with pytest.raises(ValueError, match="sequence leakage"):
        assert_disjoint_sequence_ids(["run-a", "run-b"], ["run-b", "run-c"])
