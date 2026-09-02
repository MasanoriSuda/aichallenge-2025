import numpy as np

from audit_swept_maneuver_certificate import (
    latest_preceding_indices,
    summarize_maneuver_evidence,
)


def test_latest_preceding_indices_are_causal() -> None:
    indices, ages, valid = latest_preceding_indices(
        np.asarray([5, 10, 16]),
        np.asarray([6, 9, 15]),
    )

    assert indices.tolist() == [0, 1, 2]
    assert ages.tolist() == [-1, 1, 1]
    assert valid.tolist() == [False, True, True]


def test_summary_exposes_opposite_escape_and_unproven_forward() -> None:
    report = summarize_maneuver_evidence(
        [
            {
                "selected_side_available": False,
                "opposite_side_available": True,
                "any_candidate_available": True,
                "teacher_forward": True,
                "best_side_sign": -1,
                "teacher_side_sign": 1,
                "best_minimum_clearance_m": 0.4,
                "speed_mps": 4.0,
                "front_distance_m": 5.0,
            },
            {
                "selected_side_available": True,
                "opposite_side_available": False,
                "any_candidate_available": True,
                "teacher_forward": True,
                "best_side_sign": 1,
                "teacher_side_sign": 1,
                "best_minimum_clearance_m": 0.3,
                "speed_mps": 3.0,
                "front_distance_m": 6.0,
            },
        ]
    )

    assert report["evaluated_samples"] == 2
    assert report["selected_infeasible_opposite_feasible"]["samples"] == 1
    assert report["teacher_forward_without_selected_certificate"]["samples"] == 1
    assert report["selected_side_available_fraction"] == 0.5


def test_empty_summary_is_explicit() -> None:
    assert summarize_maneuver_evidence([]) == {"evaluated_samples": 0}

