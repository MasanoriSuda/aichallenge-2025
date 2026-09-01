"""Tests for the production-candidate longitudinal safety envelope."""

import pytest

from tiny_lidar_net_controller.gap_teacher import GapTeacherConfig
from tiny_lidar_net_controller.longitudinal_safety import (
    LidarSpeedAwareLongitudinalSafety,
    SpeedAwareLongitudinalSafetyConfig,
    safe_speed_from_clearance,
)


def test_safe_speed_inverts_physical_distance() -> None:
    speed = safe_speed_from_clearance(6.0, 1.5, 2.0, 0.25)
    assert speed * 0.25 + speed * speed / 4.0 == pytest.approx(4.5)


def test_clear_scene_does_not_expand_existing_exposure() -> None:
    safety = LidarSpeedAwareLongitudinalSafety(GapTeacherConfig())
    decision = safety.decide_from_front_distance(3.01, 0.8, 4.0)
    assert not decision.active
    assert decision.reason == "clear"
    assert decision.acceleration_mps2 == pytest.approx(0.8)


def test_stopped_vehicle_receives_only_bounded_slow_zone_request() -> None:
    safety = LidarSpeedAwareLongitudinalSafety(GapTeacherConfig())
    decision = safety.decide_from_front_distance(1.7, 0.8, 0.0)
    assert decision.active
    assert decision.reason == "safe-speed-limit"
    assert 0.0 < decision.acceleration_mps2 < 0.8
    assert decision.acceleration_mps2 == pytest.approx(decision.safe_speed_mps)


def test_fast_vehicle_brakes_inside_slow_zone_and_hard_stop_is_preserved() -> None:
    safety = LidarSpeedAwareLongitudinalSafety(GapTeacherConfig())
    slow = safety.decide_from_front_distance(2.8, 0.8, 2.0)
    stop = safety.decide_from_front_distance(1.4, 0.8, 0.0)
    assert slow.acceleration_mps2 < 0.0
    assert slow.reason == "safe-speed-limit"
    assert stop.acceleration_mps2 == pytest.approx(-1.0)
    assert stop.reason == "stop-clearance"


def test_missing_speed_cannot_accelerate_inside_slow_zone() -> None:
    safety = LidarSpeedAwareLongitudinalSafety(GapTeacherConfig())
    decision = safety.decide_from_front_distance(2.0, 0.8, None)
    assert decision.active
    assert decision.reason == "missing-or-stale-speed"
    assert decision.acceleration_mps2 == pytest.approx(0.0)


def test_envelope_cannot_assume_stronger_deceleration_than_commanded() -> None:
    with pytest.raises(ValueError, match="may not exceed"):
        LidarSpeedAwareLongitudinalSafety(
            GapTeacherConfig(brake_acceleration_mps2=-1.0),
            SpeedAwareLongitudinalSafetyConfig(
                effective_deceleration_mps2=1.1
            ),
        )
