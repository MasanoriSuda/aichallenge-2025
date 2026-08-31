import importlib.util
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "evaluate_checkpoint.py"
SPEC = importlib.util.spec_from_file_location("evaluate_checkpoint", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_summarize_errors_reports_known_values():
    metrics = MODULE.summarize_errors(
        np.array([0.0, 0.2, -0.1]),
        np.array([0.0, 0.1, 0.1]),
    )

    assert metrics["mae_rad"] == pytest.approx(0.1)
    assert metrics["rmse_rad"] == pytest.approx(np.sqrt(0.05 / 3.0))
    assert metrics["max_absolute_error_rad"] == pytest.approx(0.2)
    assert metrics["bias_rad"] == pytest.approx(-1.0 / 30.0)


@pytest.mark.parametrize(
    "predictions,targets",
    [
        (np.array([]), np.array([])),
        (np.array([[0.0]]), np.array([[0.0]])),
        (np.array([np.nan]), np.array([0.0])),
        (np.array([0.0]), np.array([0.0, 1.0])),
    ],
)
def test_summarize_errors_rejects_invalid_inputs(predictions, targets):
    with pytest.raises(ValueError):
        MODULE.summarize_errors(predictions, targets)


def test_correction_subset_reports_teacher_intervention_samples():
    report = MODULE.correction_subset_report(
        candidate_predictions=np.array([0.0, 0.18, -0.1]),
        baseline_predictions=np.array([0.0, 0.0, 0.0]),
        targets=np.array([0.0, 0.2, -0.1]),
        threshold_rad=0.05,
    )

    assert report["sample_count"] == 2
    assert report["sample_fraction"] == pytest.approx(2.0 / 3.0)
    assert report["candidate"]["mae_rad"] == pytest.approx(0.01)
    assert report["baseline"]["mae_rad"] == pytest.approx(0.15)


def test_correction_subset_rejects_invalid_threshold():
    with pytest.raises(ValueError):
        MODULE.correction_subset_report(
            np.array([0.0]), np.array([0.0]), np.array([0.0]), 0.0
        )
