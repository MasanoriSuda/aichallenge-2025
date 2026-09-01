import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "relabel_gap_teacher_bag.py"
SPEC = importlib.util.spec_from_file_location("relabel_gap_teacher_bag", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_first_confirmed_breach_rejects_single_sample_noise():
    minima = np.array([2.0, 0.4, 2.0, 0.4, 0.3, 0.2, 2.0])
    assert MODULE.first_confirmed_breach(minima, 0.5, 3) == 3


def test_first_confirmed_breach_returns_none_for_safe_sequence():
    assert MODULE.first_confirmed_breach(np.array([1.0, 0.9]), 0.5, 2) is None


def test_cutoff_excludes_pre_contact_margin():
    timestamps = np.arange(10, dtype=np.int64) * 100_000_000
    assert MODULE.cutoff_before_margin(timestamps, 8, 0.2) == 6
    assert MODULE.cutoff_before_margin(timestamps, None, 0.2) == 10


def test_duration_cutoff_is_exclusive_from_first_scan():
    timestamps = 1_000_000_000 + np.arange(10, dtype=np.int64) * 100_000_000
    assert MODULE.cutoff_before_duration(timestamps, 0.35) == 4
    assert MODULE.cutoff_before_duration(timestamps, None) == 10


@pytest.mark.parametrize("duration", [0.0, -1.0, np.inf, np.nan])
def test_duration_cutoff_rejects_invalid_limit(duration):
    timestamps = np.arange(3, dtype=np.int64) * 100_000_000
    with pytest.raises(ValueError, match="max_duration_sec"):
        MODULE.cutoff_before_duration(timestamps, duration)


def test_minimum_observed_ranges_treats_empty_scan_as_blocked():
    scans = np.array([[0.0, np.nan, np.inf], [0.0, 1.2, 0.8]])
    minima = MODULE.minimum_observed_ranges(scans)
    assert minima.tolist() == pytest.approx([0.0, 0.8])


def test_precontact_teacher_has_distinct_provenance():
    historical = MODULE.teacher_identity("gap_teacher")
    precontact = MODULE.teacher_identity("precontact_teacher")

    assert historical.label_source == "lidar_gap_teacher_dagger"
    assert historical.teacher_class == "LidarGapTeacher"
    assert precontact.label_source == "lidar_precontact_teacher_dagger"
    assert precontact.teacher_class == "LidarPrecontactTeacher"
    assert precontact.control_mode == "precontact_teacher"
    assert precontact.generated_control_type != historical.generated_control_type


def test_speed_committed_teacher_has_distinct_provenance():
    precontact = MODULE.teacher_identity("precontact_teacher")
    successor = MODULE.teacher_identity("speed_committed_teacher")

    assert successor.label_source == "lidar_speed_committed_teacher_dagger"
    assert successor.teacher_class == "LidarSpeedCommittedTeacher"
    assert successor.control_mode == "speed_committed_teacher"
    assert successor.generated_control_type != precontact.generated_control_type


def test_unknown_teacher_mode_is_rejected():
    with pytest.raises(ValueError, match="unsupported teacher mode"):
        MODULE.teacher_identity("unknown")


@pytest.mark.parametrize(
    ("teacher", "reference", "minimum", "expected"),
    [
        (0.10, 0.08, 0.02, True),
        (0.10, 0.081, 0.02, False),
        (-0.20, 0.10, 0.02, True),
    ],
)
def test_novel_policy_sample_uses_material_steering_delta(
    teacher, reference, minimum, expected
):
    assert MODULE.is_novel_policy_sample(teacher, reference, minimum) is expected


def test_novel_policy_sample_rejects_invalid_threshold():
    with pytest.raises(ValueError):
        MODULE.is_novel_policy_sample(0.0, 0.0, -0.1)


@pytest.mark.parametrize("confirmation", [0, -1])
def test_first_confirmed_breach_rejects_invalid_confirmation(confirmation):
    with pytest.raises(ValueError):
        MODULE.first_confirmed_breach(np.array([0.1]), 0.5, confirmation)
