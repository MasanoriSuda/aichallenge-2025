"""Unit tests for the V2X Gate1 longitudinal stop planner."""

from dataclasses import dataclass
from typing import List

import pytest

from multi_purpose_mpc_ros.v2x_stop_planner import (
    V2XStopConfig,
    V2XStopPlanner,
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


def _path(length=60):
    return [(float(x), 0.0) for x in range(length + 1)]


def _planner(**kwargs):
    cfg = V2XStopConfig(
        detection_range_m=35.0,
        corridor_half_width_m=2.0,
        self_ignore_radius_m=0.75,
        target_stop_gap_m=1.0,
        stop_hold_gap_m=1.6,
        release_gap_m=3.0,
        comfortable_decel_mps2=1.4,
        stale_timeout_sec=0.8,
        max_speed_cap_mps=30.0 / 3.6,
        circular_path=False,
    )
    for key, value in kwargs.items():
        setattr(cfg, key, value)
    return V2XStopPlanner(cfg)


def test_front_vehicle_caps_speed():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", 10.0, 0.0)]))

    result = planner.compute_speed_cap(
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        ego_v_mps=8.0,
        reference_xy=_path(),
        now_sec=0.1,
        base_speed_mps=8.0,
    )

    assert result.active
    assert result.reason == "braking"
    assert result.vehicle_id == "d2"
    assert result.gap_m == pytest.approx(10.0)
    assert result.speed_cap_mps < 8.0
    assert result.speed_cap_mps == pytest.approx((2.0 * 1.4 * 9.0) ** 0.5)


def test_behind_vehicle_is_ignored():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", -5.0, 0.0)]))

    result = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 8.0, _path(), 0.1, 8.0)

    assert not result.active
    assert result.reason == "clear"
    assert result.speed_cap_mps == pytest.approx(8.0)


def test_lateral_vehicle_is_ignored():
    planner = _planner(corridor_half_width_m=1.5)
    planner.update_v2x(_msg(0.0, [("d2", 10.0, 3.0)]))

    result = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 8.0, _path(), 0.1, 8.0)

    assert not result.active


def test_self_echo_is_ignored():
    planner = _planner(self_ignore_radius_m=0.75)
    planner.update_v2x(_msg(0.0, [("d1", 0.5, 0.0)]))

    result = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 8.0, _path(), 0.1, 8.0)

    assert not result.active


def test_target_stop_gap_commands_zero_speed():
    planner = _planner(target_stop_gap_m=1.0)
    planner.update_v2x(_msg(0.0, [("d2", 0.8, 0.0)]))

    result = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 1.0, _path(), 0.1, 8.0)

    assert result.active
    assert result.holding_stop
    assert result.speed_cap_mps == pytest.approx(0.0)


def test_hold_stop_uses_release_hysteresis():
    planner = _planner(
        target_stop_gap_m=1.0,
        stop_hold_gap_m=1.6,
        release_gap_m=3.0,
    )
    planner.update_v2x(_msg(0.0, [("d2", 0.8, 0.0)]))
    first = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 0.0, _path(), 0.1, 8.0)
    assert first.holding_stop

    planner.update_v2x(_msg(0.2, [("d2", 2.5, 0.0)]))
    second = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 0.0, _path(), 0.2, 8.0)
    assert second.holding_stop
    assert second.speed_cap_mps == pytest.approx(0.0)

    planner.update_v2x(_msg(0.3, [("d2", 4.0, 0.0)]))
    third = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 0.0, _path(), 0.3, 8.0)
    assert not third.holding_stop
    assert third.reason == "braking"
    assert third.speed_cap_mps > 0.0


def test_stale_v2x_is_ignored_after_timeout():
    planner = _planner(stale_timeout_sec=0.5)
    planner.update_v2x(_msg(0.0, [("d2", 10.0, 0.0)]))

    fresh = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 8.0, _path(), 0.4, 8.0)
    stale = planner.compute_speed_cap(
        0.0, 0.0, 0.0, 8.0, _path(), 0.6, 8.0)

    assert fresh.active
    assert not stale.active
    assert stale.speed_cap_mps == pytest.approx(8.0)


def test_yaw_fallback_works_without_reference_path():
    planner = _planner()
    planner.update_v2x(_msg(0.0, [("d2", 8.0, 1.0)]))

    result = planner.compute_speed_cap(
        ego_x=0.0,
        ego_y=0.0,
        ego_yaw=0.0,
        ego_v_mps=8.0,
        reference_xy=[],
        now_sec=0.1,
        base_speed_mps=8.0,
    )

    assert result.active
    assert result.gap_m == pytest.approx(8.0)
    assert result.lateral_offset_m == pytest.approx(1.0)
