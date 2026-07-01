"""Unit tests for the V2X Gate2 overtake planner."""

from dataclasses import dataclass
from types import SimpleNamespace
from typing import List

import pytest

from multi_purpose_mpc_ros.v2x_overtake_planner import (
    V2XOvertakeConfig,
    V2XOvertakePlanner,
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


def _widths(length=80, left=5.0, right=5.0):
    return [(left, right) for _ in range(length + 1)]


def _planner(**kwargs):
    cfg = V2XOvertakeConfig(
        detection_range_m=35.0,
        corridor_half_width_m=2.0,
        self_ignore_radius_m=0.75,
        min_overtake_start_gap_m=6.0,
        max_overtake_start_gap_m=22.0,
        abort_gap_m=3.5,
        return_clearance_m=8.0,
        preferred_side="left",
        lateral_offset_m=1.8,
        lateral_offset_rate_mps=0.8,
        constraint_half_width_m=0.55,
        min_lateral_clearance_m=1.2,
        min_wall_clearance_m=0.5,
        wall_safety_margin_m=0.2,
        wall_check_horizon_m=20.0,
        vehicle_width_m=1.4,
        overtake_speed_cap_mps=5.0,
        follow_speed_cap_mps=3.0,
        max_overtake_target_speed_mps=3.0,
        min_closing_speed_mps=0.3,
        min_ttc_sec=0.8,
        stale_timeout_sec=1.0,
        target_lost_hold_sec=1.2,
        circular_path=False,
        return_offset_threshold_m=0.2,
    )
    for key, value in kwargs.items():
        setattr(cfg, key, value)
    return V2XOvertakePlanner(cfg)


def test_config_factory_normalizes_values_and_speed_units():
    cfg = V2XOvertakeConfig.from_config(
        SimpleNamespace(
            preferred_side="invalid",
            side_selection_policy="invalid",
            overtake_speed_cap_kmph=18.0,
            follow_speed_cap_kmph=9.0,
            max_overtake_target_speed_kmph=3.6,
        ),
        vehicle_width_m=1.45,
    )

    assert cfg.preferred_side == "left"
    assert cfg.side_selection_policy == "largest_margin"
    assert cfg.overtake_speed_cap_mps == pytest.approx(5.0)
    assert cfg.follow_speed_cap_mps == pytest.approx(2.5)
    assert cfg.max_overtake_target_speed_mps == pytest.approx(1.0)
    assert cfg.vehicle_width_m == pytest.approx(1.45)
    assert cfg.circular_path is True


def test_force_abort_returns_one_shot_abort_result():
    planner = _planner()
    planner.force_abort("mpc_infeasible")

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)

    assert result.active
    assert result.state == "abort"
    assert result.speed_cap_mps == 0.0
    assert result.reason == "mpc_infeasible"

    next_result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.2, 8.0)
    assert not next_result.active
    assert next_result.state == "clear"


def test_front_vehicle_selects_preferred_left_side():
    planner = _planner(preferred_side="left")
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        ego_v_mps=6.0,
        reference_xy=_path(),
        reference_widths=_widths(),
        now_sec=0.1,
        base_speed_mps=8.0,
    )

    assert result.active
    assert result.state == "prepare_overtake"
    assert result.vehicle_id == "d2"
    assert result.side == "left"
    assert result.target_lateral_offset_m > 0.0
    assert result.speed_cap_mps == pytest.approx(5.0)


def test_behind_vehicle_is_ignored():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", -5.0, 0.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)

    assert not result.active
    assert result.state == "clear"


def test_default_circular_path_keeps_ahead_target_positive_across_boundary():
    planner = _planner(
        circular_path=True,
        preferred_side="right",
        min_overtake_start_gap_m=1.0,
        max_overtake_start_gap_m=3.0,
        abort_gap_m=0.5,
    )
    planner.update_v2x(_msg(0.0, [("d2", 0.0, 0.0)]))

    result = planner.compute_behavior(
        2.0, 0.0, 0.0, 6.0, _path(length=2), _widths(length=2), 0.1, 8.0)

    assert result.active
    assert result.state == "prepare_overtake"
    assert result.gap_m == pytest.approx(2.0)
    assert result.signed_gap_m == pytest.approx(2.0)


def test_lateral_vehicle_is_ignored():
    planner = _planner(corridor_half_width_m=1.5)
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 3.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)

    assert not result.active
    assert result.state == "clear"


def test_gap_outside_start_range_follows():
    planner = _planner(max_overtake_start_gap_m=22.0)
    planner.update_v2x(_msg(0.0, [("d2", 30.0, 0.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)

    assert result.active
    assert result.state == "follow"
    assert result.side is None


def test_both_sides_safe_uses_preferred_side():
    planner = _planner(preferred_side="right")
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)

    assert result.active
    assert result.side == "right"
    assert result.target_lateral_offset_m < 0.0


def test_largest_margin_uses_more_open_side_over_preferred():
    planner = _planner(
        preferred_side="left",
        side_selection_policy="largest_margin",
        side_margin_tie_threshold_m=0.1,
    )
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(left=4.0, right=6.0), 0.1, 8.0)

    assert result.active
    assert result.side == "right"
    assert result.reason == "right_larger_wall_margin"
    assert result.target_lateral_offset_m < 0.0


def test_largest_margin_tie_uses_preferred_side():
    planner = _planner(
        preferred_side="left",
        side_selection_policy="largest_margin",
        side_margin_tie_threshold_m=0.5,
    )
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(left=5.0, right=5.2), 0.1, 8.0)

    assert result.active
    assert result.side == "left"
    assert result.reason == "left_preferred_tie"


def test_preferred_policy_keeps_preferred_even_if_other_side_has_more_margin():
    planner = _planner(
        preferred_side="left",
        side_selection_policy="preferred",
    )
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(left=4.0, right=6.0), 0.1, 8.0)

    assert result.active
    assert result.side == "left"
    assert result.reason == "left_preferred"


def test_forced_side_is_not_switched_after_overtake_started():
    planner = _planner(
        preferred_side="left",
        side_selection_policy="largest_margin",
    )
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))
    first = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)

    planner.update_v2x(_msg(0.2, [("d2", 12.0, 0.0)]))
    second = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(left=2.0, right=6.0),
        0.2,
        8.0,
        current_lateral_offset_m=1.8,
    )

    assert first.side == "left"
    assert second.active
    assert second.state == "abort"
    assert second.reason == "forced_left_unsafe"
    assert second.side is None


def test_right_overtake_does_not_count_left_ey_as_lateral_progress():
    planner = _planner(preferred_side="right")
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))
    first = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    assert first.state == "prepare_overtake"
    assert first.side == "right"

    planner.update_v2x(_msg(0.2, [("d2", 12.0, 0.0)]))
    still_preparing = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        current_lateral_offset_m=1.8,
    )

    assert still_preparing.state == "prepare_overtake"
    assert still_preparing.side == "right"


def test_active_right_overtake_continues_near_abort_gap_after_escape_started():
    planner = _planner(
        preferred_side="right",
        abort_gap_m=3.5,
        lateral_offset_m=1.3,
    )
    planner.update_v2x(_msg(0.0, [("d2", 7.0, 0.0)]))
    first = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    assert first.state == "prepare_overtake"
    assert first.side == "right"

    planner.update_v2x(_msg(0.2, [("d2", 7.0, 0.0)]))
    continued = planner.compute_behavior(
        4.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        current_lateral_offset_m=-0.7,
    )

    assert continued.active
    assert continued.state != "abort"
    assert continued.side == "right"
    assert continued.target_lateral_offset_m < 0.0
    assert continued.speed_cap_mps > 0.0


def test_abort_gap_escape_threshold_is_not_scaled_by_large_overtake_offset():
    planner = _planner(
        preferred_side="right",
        abort_gap_m=3.5,
        abort_escape_lateral_threshold_m=0.45,
        lateral_offset_m=1.65,
    )
    planner.update_v2x(_msg(0.0, [("d2", 7.0, 0.0)]))
    first = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    assert first.state == "prepare_overtake"
    assert first.side == "right"

    planner.update_v2x(_msg(0.2, [("d2", 7.0, 0.0)]))
    continued = planner.compute_behavior(
        4.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        current_lateral_offset_m=-0.5,
    )

    assert continued.active
    assert continued.state != "abort"
    assert continued.side == "right"
    assert continued.target_lateral_offset_m < 0.0


def test_active_right_overtake_aborts_near_gap_without_escape_progress():
    planner = _planner(
        preferred_side="right",
        abort_gap_m=3.5,
        lateral_offset_m=1.3,
    )
    planner.update_v2x(_msg(0.0, [("d2", 7.0, 0.0)]))
    first = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    assert first.state == "prepare_overtake"
    assert first.side == "right"

    planner.update_v2x(_msg(0.2, [("d2", 7.0, 0.0)]))
    aborted = planner.compute_behavior(
        4.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        current_lateral_offset_m=-0.2,
    )

    assert aborted.state == "abort"
    assert aborted.reason == "abort_gap"
    assert aborted.speed_cap_mps == pytest.approx(0.0)


def test_preferred_side_blocked_uses_right_side():
    planner = _planner(preferred_side="left")
    planner.update_v2x(_msg(0.0, [
        ("d2", 12.0, 0.0),
        ("d3", 13.0, 1.8),
    ]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)

    assert result.active
    assert result.side == "right"
    assert result.target_lateral_offset_m < 0.0


def test_wall_clearance_blocks_unsafe_side():
    planner = _planner(preferred_side="left")
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(left=2.0, right=5.0),
        0.1,
        8.0,
    )

    assert result.active
    assert result.side == "right"
    assert result.left_wall_margin_m < 0.0
    assert result.right_wall_margin_m >= 0.0


def test_both_sides_unsafe_follows_instead_of_overtaking():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(left=2.0, right=2.0),
        0.1,
        8.0,
    )

    assert result.active
    assert result.state == "follow"
    assert result.side is None
    assert result.target_lateral_offset_m == pytest.approx(0.0)
    assert result.speed_cap_mps == pytest.approx(3.0)


def test_abort_gap_does_not_overtake():
    planner = _planner(abort_gap_m=3.5)
    planner.update_v2x(_msg(0.0, [("d2", 3.0, 0.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)

    assert result.active
    assert result.state == "abort"
    assert result.speed_cap_mps == pytest.approx(0.0)
    assert result.target_lateral_offset_m == pytest.approx(0.0)


def test_stale_sample_is_ignored():
    planner = _planner(stale_timeout_sec=0.5)
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))

    result = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.6, 8.0)

    assert not result.active
    assert result.state == "clear"


def test_active_overtake_holds_side_during_short_target_loss():
    planner = _planner(
        preferred_side="right",
        stale_timeout_sec=0.2,
        target_lost_hold_sec=1.0,
    )
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))
    first = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    assert first.state == "prepare_overtake"
    assert first.side == "right"

    held = planner.compute_behavior(
        0.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(),
        0.4,
        8.0,
        current_lateral_offset_m=-0.8,
    )

    assert held.active
    assert held.state == "prepare_overtake"
    assert held.reason == "target_lost_hold"
    assert held.side == "right"
    assert held.target_lateral_offset_m < 0.0
    assert held.speed_cap_mps == pytest.approx(3.0)


def test_return_to_line_after_target_is_passed():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))
    first = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    assert first.state == "prepare_overtake"

    planner.update_v2x(_msg(0.2, [("d2", 10.0, 0.0)]))
    passed = planner.compute_behavior(
        22.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        current_lateral_offset_m=1.8,
    )

    assert passed.state == "return_to_line"
    assert passed.target_lateral_offset_m == pytest.approx(0.0)


def test_overtake_switches_to_next_queue_target_before_returning():
    planner = _planner(
        preferred_side="right",
        circular_path=False,
        return_clearance_m=4.0,
    )
    planner.update_v2x(_msg(0.0, [
        ("d2", 12.0, 0.0),
        ("d3", 18.0, 0.0),
    ]))
    first = planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    assert first.state == "prepare_overtake"
    assert first.side == "right"

    planner.update_v2x(_msg(0.2, [
        ("d2", 12.0, 0.0),
        ("d3", 19.0, 0.0),
    ]))
    queued = planner.compute_behavior(
        16.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(),
        0.2,
        8.0,
        current_lateral_offset_m=-1.6,
    )

    assert queued.state == "overtaking"
    assert queued.vehicle_id == "d3"
    assert queued.side == "right"
    assert queued.reason == "queue_target_right_forced_clear"
    assert queued.target_lateral_offset_m < 0.0


def test_return_does_not_clear_until_actual_lateral_offset_returns():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))
    planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    planner.update_v2x(_msg(0.2, [("d2", 10.0, 0.0)]))
    planner.compute_behavior(
        22.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.2, 8.0, 1.8)

    returning = planner.compute_behavior(
        22.0,
        0.0,
        0.0,
        6.0,
        _path(),
        _widths(),
        0.3,
        8.0,
        current_lateral_offset_m=1.0,
    )

    assert returning.active
    assert returning.state == "return_to_line"
    assert returning.target_lateral_offset_m == pytest.approx(0.0)


def test_return_completion_clears_state():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", 12.0, 0.0)]))
    planner.compute_behavior(
        0.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.1, 8.0)
    planner.update_v2x(_msg(0.2, [("d2", 10.0, 0.0)]))
    planner.compute_behavior(
        22.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.2, 8.0, 1.8)

    cleared = planner.compute_behavior(
        22.0, 0.0, 0.0, 6.0, _path(), _widths(), 0.3, 8.0, 0.05)

    assert not cleared.active
    assert cleared.state == "clear"
