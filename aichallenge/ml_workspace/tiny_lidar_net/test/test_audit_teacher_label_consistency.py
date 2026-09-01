import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "audit_teacher_label_consistency.py"
)
SPEC = importlib.util.spec_from_file_location(
    "audit_teacher_label_consistency", MODULE_PATH
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_correction_summary_separates_material_directions():
    report = MODULE.correction_summary(
        np.asarray([-0.03, -0.01, 0.0, 0.02, 0.4]), 0.02
    )

    assert report["samples"] == 5
    assert report["material_samples"] == 3
    assert report["material_fraction"] == pytest.approx(0.6)
    assert report["left_samples"] == 1
    assert report["right_samples"] == 2


def test_reason_counts_merge_without_losing_sequences():
    assert MODULE.merge_reason_counts(
        [{"front-clear": 3}, {"front-clear": 2, "side-clearance": 4}]
    ) == {"front-clear": 5, "side-clearance": 4}
