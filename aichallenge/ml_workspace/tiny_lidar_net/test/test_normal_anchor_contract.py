import json
from pathlib import Path

import numpy as np
import pytest

from build_normal_anchor_recurrent_dataset import normal_anchor_sequence_id
from lib.normal_anchor import (
    NORMAL_ANCHOR_LABEL_SOURCE,
    NORMAL_ANCHOR_SCHEMA_VERSION,
    NormalAnchorSequenceDataset,
)


def write_sequence(root: Path, label_source: str = NORMAL_ANCHOR_LABEL_SOURCE):
    sequence_id = "normal-sequence"
    sequence = root / sequence_id
    sequence.mkdir()
    metadata = {
        "schema_version": NORMAL_ANCHOR_SCHEMA_VERSION,
        "sequence_id": sequence_id,
        "split": "train",
        "label_source": label_source,
        "scan_unit": "m",
        "scan_shape": [750],
        "max_scan_range_m": 30.0,
        "max_speed_sync_delta_sec": 0.05,
    }
    (sequence / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    np.save(sequence / "scans.npy", np.full((2, 750), 12.0, dtype=np.float32))
    np.save(sequence / "speeds.npy", np.asarray([2.0, 3.0], dtype=np.float32))
    np.save(
        sequence / "scan_timestamps_ns.npy",
        np.asarray([1_000_000_000, 1_100_000_000], dtype=np.int64),
    )
    np.save(
        sequence / "speed_timestamps_ns.npy",
        np.asarray([1_001_000_000, 1_101_000_000], dtype=np.int64),
    )
    np.save(
        sequence / "speed_sync_deltas_sec.npy",
        np.asarray([0.001, 0.001], dtype=np.float64),
    )
    return sequence


def test_normal_anchor_loader_emits_physical_state_and_zero_correction(tmp_path):
    dataset = NormalAnchorSequenceDataset(write_sequence(tmp_path), "train")

    scan, speed, teacher, base = dataset[1]

    assert scan.tolist() == pytest.approx([12.0] * 750)
    assert speed == pytest.approx(3.0)
    assert (teacher, base) == (0.0, 0.0)


def test_normal_anchor_loader_rejects_teacher_label_source(tmp_path):
    sequence = write_sequence(tmp_path, "lidar_precontact_teacher_recurrent_direct")

    with pytest.raises(ValueError, match="label source"):
        NormalAnchorSequenceDataset(sequence, "train")


def test_normal_anchor_identity_binds_speed_contract():
    first = normal_anchor_sequence_id("run", "/speed", 0.05)

    assert first == normal_anchor_sequence_id("run", "/speed", 0.05)
    assert first != normal_anchor_sequence_id("run", "/speed", 0.04)
