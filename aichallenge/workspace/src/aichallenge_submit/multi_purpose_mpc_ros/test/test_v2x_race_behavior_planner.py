"""Unit tests for the V2X race follow/yield/overtake planner."""

from dataclasses import dataclass
from typing import List

import pytest

from multi_purpose_mpc_ros.v2x_race_behavior_planner import (
    V2XRaceBehaviorConfig,
    V2XRaceBehaviorPlanner,
)


@dataclass
class _Stamp:
    sec: int
    nanosec: int


@dataclass
class _Header:
    stamp: _Stamp


@dataclass
class _Point:
    x: float
    y: float
    z: float = 0.0


@dataclass
class _V2XVehiclePosition:
    header: _Header
    vehicle_id: str
    position: _Point


@dataclass
class _V2XVehiclePositionArray:
    header: _Header
    vehicles: List[_V2XVehiclePosition]


def _stamp(stamp_sec: float) -> _Stamp:
    sec = int(stamp_sec)
    nanosec = int((stamp_sec - sec) * 1e9)
    return _Stamp(sec, nanosec)


def _msg(stamp_sec: float, vehicles):
    header = _Header(_stamp(stamp_sec))
    return _V2XVehiclePositionArray(
        header=header,
        vehicles=[
            _V2XVehiclePosition(
                header=header,
                vehicle_id=vehicle_id,
                position=_Point(x=x, y=y),
            )
            for vehicle_id, x, y in vehicles
        ],
    )


def _path(length=80):
    return [(float(x), 0.0) for x in range(length + 1)]


def _reverse_path(length=80):
    return [(float(-x), 0.0) for x in range(length + 1)]


def _widths(length=80, left=5.0, right=5.0):
    return [(left, right) for _ in range(length + 1)]


def _velocity(values):
    return lambda vehicle_id: values.get(vehicle_id, (0.0, 0.0))


def _planner(**kwargs):
    defaults = dict(
        detection_range_m=40.0,
        corridor_half_width_m=2.0,
        front_conflict_lateral_window_m=2.6,
        front_conflict_gap_m=10.0,
        self_ignore_radius_m=0.75,
        min_follow_gap_m=3.0,
        time_headway_sec=0.8,
        follow_kp_gap=0.4,
        follow_kd_closing=0.8,
        follow_speed_cap_mps=4.0,
        emergency_stop_gap_m=1.5,
        yield_detect_range_m=12.0,
        yield_start_gap_m=8.0,
        yield_release_gap_m=14.0,
        yield_min_closing_speed_mps=0.4,
        yield_speed_cap_mps=2.0,
        yield_timeout_sec=4.0,
        yield_side_hold_sec=1.0,
        catchup_detect_range_m=120.0,
        catchup_start_gap_m=18.0,
        catchup_release_gap_m=8.0,
        catchup_speed_cap_mps=2.8,
        catchup_timeout_sec=30.0,
        approach_start_gap_m=34.0,
        approach_speed_cap_mps=5.0,
        min_overtake_start_gap_m=4.0,
        max_overtake_start_gap_m=22.0,
        abort_gap_m=2.0,
        return_clearance_m=8.0,
        return_rear_clearance_m=4.0,
        preferred_side="right",
        side_selection_policy="largest_margin",
        side_margin_tie_threshold_m=0.3,
        lateral_offset_m=1.4,
        lateral_offset_rate_mps=1.2,
        constraint_half_width_m=0.55,
        min_lateral_clearance_m=1.6,
        min_wall_clearance_m=0.5,
        wall_safety_margin_m=0.2,
        wall_check_horizon_m=16.0,
        vehicle_width_m=1.45,
        overtake_speed_cap_mps=12.0 / 3.6,
        prepare_speed_cap_mps=9.0 / 3.6,
        max_overtake_target_speed_mps=12.0 / 3.6,
        min_closing_speed_mps=0.3,
        min_ttc_sec=0.8,
        stale_timeout_sec=1.0,
        target_lost_hold_sec=1.0,
        circular_path=False,
        return_offset_threshold_m=0.2,
        overtake_cooldown_sec=2.0,
        side_lock_min_sec=1.0,
        side_switch_center_threshold_m=0.45,
    )
    defaults.update(kwargs)
    return V2XRaceBehaviorPlanner(V2XRaceBehaviorConfig(**defaults))


def test_front_vehicle_follows_when_overtake_gap_is_not_ready():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", 3.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        4.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    assert result.active
    assert result.state == "follow"
    assert result.target_role == "front"
    assert result.speed_cap_mps < 4.0
    assert result.target_lateral_offset_m == pytest.approx(0.0)


def test_close_front_vehicle_can_start_overtake_after_min_gap():
    planner = _planner(preferred_side="right")
    planner.update_v2x(_msg(0.0, [("d2", 4.5, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    assert result.active
    assert result.state == "prepare_overtake"
    assert result.side == "right"
    assert result.target_role == "front"
    assert result.speed_cap_mps == pytest.approx(12.0 / 3.6)


def test_side_switch_is_blocked_until_near_center():
    planner = _planner(preferred_side="right")
    planner.update_v2x(_msg(0.0, [("d2", 4.5, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(left=5.0, right=2.0),
        0.1,
        8.0,
        current_lateral_offset_m=-1.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    assert result.active
    assert result.state == "follow"
    assert result.reason == "side_switch_blocked"
    assert result.side is None
    assert result.target_lateral_offset_m == pytest.approx(0.0)


def test_far_front_vehicle_does_not_cap_chase_speed():
    planner = _planner(max_overtake_start_gap_m=22.0)
    planner.update_v2x(_msg(0.0, [("d2", 35.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (2.8, 0.0)}),
    )

    assert result.active
    assert result.state == "follow"
    assert result.reason == "front_far_chase"
    assert result.speed_cap_mps == pytest.approx(8.0)


def test_approaching_front_vehicle_caps_before_overtake_start_range():
    planner = _planner(max_overtake_start_gap_m=22.0, approach_start_gap_m=34.0)
    planner.update_v2x(_msg(0.0, [("d2", 28.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        8.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (2.8, 0.0)}),
    )

    assert result.active
    assert result.state == "follow"
    assert result.reason == "front_approach"
    assert result.speed_cap_mps == pytest.approx(5.0)


def test_yaw_front_conflict_is_followed_when_path_relation_misses_front():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", 5.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        4.0,
        _reverse_path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    assert result.active
    assert result.state == "follow"
    assert result.reason == "front_conflict"
    assert result.gap_m == pytest.approx(5.0)
    assert result.speed_cap_mps < 4.0


def test_yaw_rear_is_not_followed_when_path_relation_looks_front():
    planner = _planner(
        yield_detect_range_m=0.0,
        catchup_start_gap_m=100.0,
    )
    planner.update_v2x(_msg(0.0, [("d2", -5.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        4.0,
        _reverse_path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    assert not result.active
    assert result.state == "clear"
    assert result.reason == "no_race_front_target"
    assert result.speed_cap_mps == pytest.approx(8.0)


def test_near_side_front_conflict_uses_wider_lateral_guard():
    planner = _planner(corridor_half_width_m=2.0, front_conflict_lateral_window_m=2.6)
    planner.update_v2x(_msg(0.0, [("d2", 4.0, 2.4)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        4.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    assert result.active
    assert result.state == "follow"
    assert result.reason == "front_conflict"
    assert result.lateral_offset_m == pytest.approx(2.4)


def test_slow_front_vehicle_starts_overtake_on_safe_right_side():
    planner = _planner(preferred_side="right")
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    assert result.active
    assert result.state == "prepare_overtake"
    assert result.side == "right"
    assert result.target_lateral_offset_m < 0.0


def test_side_blocker_prevents_overtake_and_keeps_following():
    planner = _planner(preferred_side="right")
    planner.update_v2x(_msg(0.0, [
        ("d2", 12.0, 0.0),
        ("d3", 12.0, -1.4),
    ]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(left=2.0, right=5.0),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0), "d3": (1.0, 0.0)}),
    )

    assert result.active
    assert result.state == "follow"
    assert result.reason == "no_safe_side"
    assert result.side is None
    assert result.target_lateral_offset_m == pytest.approx(0.0)


def test_rear_vehicle_triggers_yield_speed_cap_without_side_offset():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", -6.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        4.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (6.0, 0.0)}),
    )

    assert result.active
    assert result.state == "yield"
    assert result.yield_active
    assert result.target_role == "rear"
    assert result.speed_cap_mps == pytest.approx(2.0)
    assert result.target_lateral_offset_m == pytest.approx(0.0)


def test_yield_releases_after_target_passes_release_gap():
    planner = _planner(yield_side_hold_sec=1.0)
    planner.update_v2x(_msg(0.0, [("d2", -6.0, 0.0)]))
    first = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        4.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (6.0, 0.0)}),
    )
    assert first.state == "yield"

    planner.update_v2x(_msg(0.2, [("d2", 15.0, 0.0)]))
    released = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        4.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        velocity_lookup=_velocity({"d2": (6.0, 0.0)}),
    )

    assert released.state == "cooldown"
    assert released.reason == "yield_target_passed"
    assert released.target_lateral_offset_m == pytest.approx(0.0)


def test_far_rear_vehicle_triggers_catchup_wait_speed_cap():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", -35.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (5.0, 0.0)}),
    )

    assert result.active
    assert result.state == "catchup_wait"
    assert result.target_role == "rear"
    assert result.speed_cap_mps == pytest.approx(2.8)
    assert result.target_lateral_offset_m == pytest.approx(0.0)


def test_yaw_rear_vehicle_triggers_catchup_even_when_path_relation_is_front():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", -35.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _reverse_path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (5.0, 0.0)}),
    )

    assert result.active
    assert result.state == "catchup_wait"
    assert result.target_role == "rear"
    assert result.signed_gap_m > 0.0
    assert result.yaw_signed_gap_m < 0.0
    assert result.speed_cap_mps == pytest.approx(2.8)


def test_catchup_wait_releases_when_rear_gap_closes():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", -35.0, 0.0)]))
    first = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (5.0, 0.0)}),
    )
    assert first.state == "catchup_wait"

    planner.update_v2x(_msg(0.2, [("d2", -7.0, 0.0)]))
    released = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        velocity_lookup=_velocity({"d2": (5.0, 0.0)}),
    )

    assert released.state == "yield"
    assert released.yield_active
    assert released.speed_cap_mps == pytest.approx(2.0)


def test_yaw_rear_vehicle_triggers_yield_even_when_path_relation_is_front():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", -6.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        4.0,
        _reverse_path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (6.0, 0.0)}),
    )

    assert result.active
    assert result.state == "yield"
    assert result.target_role == "rear"
    assert result.signed_gap_m > 0.0
    assert result.yaw_signed_gap_m < 0.0
    assert result.speed_cap_mps == pytest.approx(2.0)


def test_return_blocker_keeps_overtake_side():
    planner = _planner(preferred_side="right")
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))
    first = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )
    assert first.state == "prepare_overtake"

    planner.update_v2x(_msg(0.2, [
        ("d2", 10.0, 0.0),
        ("d3", 20.0, 0.0),
    ]))
    blocked = planner.compute_behavior(
        22.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        current_lateral_offset_m=-1.4,
        velocity_lookup=_velocity({"d2": (1.0, 0.0), "d3": (1.0, 0.0)}),
    )

    assert blocked.state == "overtaking"
    assert blocked.reason == "return_blocked_by_d3"
    assert blocked.target_lateral_offset_m < 0.0


def test_returned_overtake_enters_cooldown():
    planner = _planner(preferred_side="right", overtake_cooldown_sec=2.0)
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))
    planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.1,
        8.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    planner.update_v2x(_msg(0.2, [("d2", 10.0, 0.0)]))
    returning = planner.compute_behavior(
        22.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        current_lateral_offset_m=-1.4,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )
    assert returning.state == "return_to_line"

    cooled = planner.compute_behavior(
        22.0,
        0.0,
        0.0,
        5.0,
        _path(),
        _widths(),
        0.3,
        8.0,
        current_lateral_offset_m=0.0,
        velocity_lookup=_velocity({"d2": (1.0, 0.0)}),
    )

    assert cooled.state == "cooldown"
    assert cooled.reason == "overtake_returned"
    assert cooled.target_lateral_offset_m == pytest.approx(0.0)
