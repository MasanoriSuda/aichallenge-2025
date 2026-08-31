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


def test_minimum_observed_ranges_treats_empty_scan_as_blocked():
    scans = np.array([[0.0, np.nan, np.inf], [0.0, 1.2, 0.8]])
    minima = MODULE.minimum_observed_ranges(scans)
    assert minima.tolist() == pytest.approx([0.0, 0.8])


@pytest.mark.parametrize("confirmation", [0, -1])
def test_first_confirmed_breach_rejects_invalid_confirmation(confirmation):
    with pytest.raises(ValueError):
        MODULE.first_confirmed_breach(np.array([0.1]), 0.5, confirmation)
