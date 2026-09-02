"""Focused tests for the speed-committed teacher escape audit."""

import numpy as np
import pytest

from audit_speed_committed_escape_contract import (
    latest_preceding_indices,
    longest_episode_sec,
    summarize_replay,
)


def test_latest_preceding_sync_never_uses_future_speed() -> None:
    indices, ages, valid = latest_preceding_indices(
        np.asarray([5, 10, 16, 30]),
        np.asarray([10, 15, 25]),
    )
    assert valid.tolist() == [False, True, True, True]
    assert indices.tolist() == [0, 0, 1, 2]
    assert ages.tolist() == [-5, 0, 1, 5]


def test_longest_episode_does_not_join_recorder_gap() -> None:
    times = np.asarray([0.0, 0.1, 0.2, 1.0, 1.1, 1.2])
    mask = np.ones(6, dtype=bool)
    assert longest_episode_sec(times, mask) == pytest.approx(0.2)


def test_summary_separates_envelope_exposure_from_committed_forward() -> None:
    times = np.arange(5, dtype=np.float64) * 0.1
    admitted = np.asarray([True, True, True, True, False])
    inside = np.asarray([False, True, True, True, False])
    committed = np.asarray([False, True, True, False, False])
    forward = np.asarray([True, True, True, False, False])
    braking = np.asarray([False, False, False, True, False])
    speed = np.asarray([3.0, 3.0, 2.9, 2.8, np.nan])
    front = np.asarray([8.0, 5.0, 4.0, 1.0, np.nan])
    stop = np.asarray([6.0, 6.0, 6.0, 6.0, np.nan])
    replay_acceleration = np.asarray([0.8, 0.8, 0.8, -1.0, np.nan])
    published_acceleration = replay_acceleration.copy()
    report = summarize_replay(
        times_sec=times,
        admitted_mask=admitted,
        inside_stop_envelope=inside,
        committed_gap=committed,
        forward_command=forward,
        braking_command=braking,
        speeds_mps=speed,
        front_distances_m=front,
        required_stop_distances_m=stop,
        reasons=["front-clear", "gap-selected", "gap-selected", "no-gap"],
        supervisor_reasons=["clear", "side-acquired", "side-maintained", "no-gap"],
        replay_acceleration_mps2=replay_acceleration,
        published_acceleration_mps2=published_acceleration,
        failure_start_sec=0.4,
    )
    assert report["inside_dynamic_stop_envelope"]["samples"] == 3
    committed_report = report[
        "committed_gap_forward_inside_dynamic_stop_envelope"
    ]
    assert committed_report["samples"] == 2
    assert committed_report["longest_sec"] == pytest.approx(0.1)
    assert report["braking_inside_dynamic_stop_envelope"]["samples"] == 1
    assert report["published_acceleration_parity"]["mae_mps2"] == 0.0
