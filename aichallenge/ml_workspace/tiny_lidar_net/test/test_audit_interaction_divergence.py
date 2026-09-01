import numpy as np

from audit_interaction_divergence import (
    contiguous_episode_count,
    count_sign_flips,
    first_sustained_interval,
    summarize_window,
)


def test_first_sustained_interval_rejects_short_and_split_stalls():
    times = np.asarray([0.0, 0.1, 0.2, 1.0, 1.1, 1.2])
    mask = np.asarray([False, True, True, True, True, True])
    assert first_sustained_interval(times, mask, 0.25) is None
    assert first_sustained_interval(times, mask, 0.1) == (1, 2)


def test_episode_and_flip_counts_do_not_join_false_samples():
    times = np.arange(6, dtype=np.float64) * 0.1
    mask = np.asarray([True, True, False, True, True, True])
    signs = np.asarray([1, 1, 0, -1, -1, 1])
    assert contiguous_episode_count(times, mask) == 2
    assert count_sign_flips(times, signs, mask) == 1


def test_flip_count_does_not_join_recorder_gaps():
    times = np.asarray([0.0, 0.1, 1.0, 1.1])
    mask = np.ones(4, dtype=bool)
    signs = np.asarray([1, 1, -1, -1])
    assert count_sign_flips(times, signs, mask) == 0


def test_window_reports_escape_deficit_without_claiming_causality():
    trace = {
        "time_sec": np.asarray([0.0, 0.1, 0.2]),
        "speed_mps": np.asarray([2.0, 2.0, 2.0]),
        "published_steering_rad": np.asarray([-0.1, 0.1, 0.3]),
        "predicted_residual_rad": np.asarray([0.1, 0.2, 0.3]),
        "teacher_residual_rad": np.asarray([0.5, 0.5, 0.5]),
        "teacher_steering_rad": np.asarray([0.4, 0.4, 0.4]),
        "teacher_reason": np.asarray(
            ["side-clearance", "side-clearance", "front-clear"]
        ),
        "left_side_m": np.asarray([4.0, 4.0, 4.0]),
        "right_side_m": np.asarray([0.8, 0.8, 4.0]),
        "escape_sign": np.asarray([1, 1, 0]),
        "side_hazard": np.asarray([True, True, False]),
    }
    report = summarize_window(trace, np.ones(3, dtype=bool))
    assert report["side_hazard_samples"] == 2
    assert report["toward_obstacle_fraction"] == 0.5
    assert report["teacher_projection_deficit_fraction"] == 1.0
    assert np.isclose(report["teacher_residual_mae_rad"], 0.35)
