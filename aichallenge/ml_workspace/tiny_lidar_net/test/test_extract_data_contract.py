from pathlib import Path

import numpy as np
import pytest

from extract_data_from_bag import (
    choose_split,
    clean_scan_array,
    discover_tasks,
    make_sequence_id,
    sync_acceptance_mask,
    synchronize_data,
)


def test_clean_scan_matches_runtime_range_contract() -> None:
    values = np.array([np.nan, -np.inf, -1.0, 2.0, np.inf, 31.0])
    cleaned = clean_scan_array(values, 30.0)
    np.testing.assert_array_equal(cleaned, [0.0, 0.0, 0.0, 2.0, 30.0, 30.0])
    assert cleaned.dtype == np.float32


def test_nearest_control_synchronization_is_deterministic() -> None:
    indices, deltas = synchronize_data(
        np.array([0, 50, 100], dtype=np.int64),
        np.array([10, 90], dtype=np.int64),
    )
    np.testing.assert_array_equal(indices, [0, 1, 1])
    np.testing.assert_array_equal(deltas, [10, 40, 10])


def test_sync_limit_accepts_boundary_and_rejects_beyond() -> None:
    accepted = sync_acceptance_mask(
        np.array([0, 49_999_999, 50_000_000, 50_000_001]),
        0.05,
    )
    np.testing.assert_array_equal(accepted, [True, True, True, False])


def test_sequence_identity_does_not_collapse_same_bag_basename() -> None:
    first = make_sequence_id("20260901-010101/d1/rosbag2_autoware")
    second = make_sequence_id("20260901-020202/d1/rosbag2_autoware")
    assert first != second
    assert first == make_sequence_id("20260901-010101/d1/rosbag2_autoware")


def test_split_is_stable_and_run_level() -> None:
    sequence_id = make_sequence_id("run/d1/rosbag2_autoware")
    assert choose_split(sequence_id, 0.0, 2026) == "train"
    assert choose_split(sequence_id, 1.0, 2026) == "val"
    assert choose_split(sequence_id, 0.2, 2026) == choose_split(
        sequence_id, 0.2, 2026
    )


def test_discovery_assigns_unique_run_directories(tmp_path: Path) -> None:
    for run_id in ("run-a", "run-b"):
        bag = tmp_path / run_id / "d1" / "rosbag2_autoware"
        bag.mkdir(parents=True)
        (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")

    tasks = discover_tasks(tmp_path, None, val_fraction=0.2, split_seed=2026)
    assert len(tasks) == 2
    assert len({task.sequence_id for task in tasks}) == 2
    assert all(task.split in {"train", "val"} for task in tasks)


def test_discovery_rejects_missing_explicit_bag(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="not a ROS 2 bag"):
        discover_tasks(None, [tmp_path / "missing"], 0.2, 2026)


def test_discovery_rejects_duplicate_explicit_bag(tmp_path: Path) -> None:
    bag = tmp_path / "run" / "rosbag2_autoware"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    with pytest.raises(ValueError, match="duplicate ROS 2 bag"):
        discover_tasks(None, [bag, bag], 0.2, 2026)
