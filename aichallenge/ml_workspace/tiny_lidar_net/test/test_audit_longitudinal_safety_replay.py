import numpy as np
import pytest

from audit_longitudinal_safety_replay import (
    longest_duration_sec,
    production_pace_request,
    safe_speed_acceleration,
    safe_speed_from_clearance,
)


def test_safe_speed_inverts_reaction_and_braking_distance():
    speed = safe_speed_from_clearance(
        np.asarray([1.5, 3.0, 6.0]),
        minimum_clearance_m=1.5,
        effective_deceleration_mps2=2.0,
        reaction_time_sec=0.25,
    )
    assert speed[0] == pytest.approx(0.0)
    assert np.all(np.diff(speed) > 0.0)
    room = 6.0 - 1.5
    assert speed[2] * 0.25 + speed[2] ** 2 / 4.0 == pytest.approx(room)


def test_safe_speed_policy_brakes_fast_and_allows_bounded_creep():
    front = np.asarray([1.4, 1.7, 14.0])
    speed = np.asarray([0.0, 0.0, 4.0])
    requested = np.asarray([0.8, 0.8, 0.6])
    acceleration, safe_speed = safe_speed_acceleration(
        front,
        speed,
        requested,
        minimum_clearance_m=1.5,
        effective_deceleration_mps2=1.0,
        reaction_time_sec=0.25,
        brake_acceleration_mps2=-1.0,
        speed_error_gain=1.0,
    )
    assert acceleration[0] == -1.0
    assert 0.0 < acceleration[1] < requested[1]
    assert safe_speed[2] > speed[2]
    assert acceleration[2] == requested[2]


def test_slow_zone_gate_does_not_expand_existing_obstacle_exposure():
    front = np.asarray([1.7, 2.8, 3.1])
    speed = np.asarray([0.0, 2.0, 4.0])
    requested = np.asarray([0.8, 0.8, 0.6])
    acceleration, _ = safe_speed_acceleration(
        front,
        speed,
        requested,
        minimum_clearance_m=1.5,
        effective_deceleration_mps2=1.0,
        reaction_time_sec=0.25,
        brake_acceleration_mps2=-1.0,
        speed_error_gain=1.0,
        activation_distance_m=3.0,
    )
    assert 0.0 < acceleration[0] < requested[0]
    assert acceleration[1] < 0.0
    assert acceleration[2] == requested[2]


def test_production_pace_request_matches_bounded_positive_authority():
    request = production_pace_request(
        np.asarray([0.0, 4.2, 4.8]),
        requested_acceleration_mps2=0.8,
        maximum_forward_speed_mps=4.6,
    )
    np.testing.assert_allclose(request, [0.8, 0.4, 0.0], atol=1e-7)


def test_longest_duration_does_not_bridge_recorder_gap():
    assert longest_duration_sec(
        np.asarray([0.0, 0.1, 1.0, 1.1]),
        np.asarray([True, True, True, True]),
    ) == pytest.approx(0.1)
