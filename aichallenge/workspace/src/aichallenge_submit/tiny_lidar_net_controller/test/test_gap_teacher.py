"""Unit tests for the teacher-only LiDAR gap residual."""

import numpy as np
import pytest

from tiny_lidar_net_controller.gap_teacher import (
    GapTeacherConfig,
    LidarGapTeacher,
    LidarLongitudinalSafety,
    LidarPrecontactTeacher,
)


def _angles(size: int = 750) -> np.ndarray:
    return np.linspace(-np.pi / 2.0, np.pi / 2.0, size)


def test_clear_front_preserves_network_command() -> None:
    teacher = LidarGapTeacher(GapTeacherConfig())
    decision = teacher.decide(np.full(750, 30.0), 0.17, 0.6)
    assert not decision.active
    assert decision.reason == "front-clear"
    assert decision.steering_rad == pytest.approx(0.17)
    assert decision.acceleration_mps2 == pytest.approx(0.6)


def test_teacher_steers_toward_side_with_larger_physical_reserve() -> None:
    teacher = LidarGapTeacher(GapTeacherConfig())
    angles = _angles()
    ranges = np.full(750, 10.0)
    ranges[(angles >= -0.18) & (angles <= 0.18)] = 3.0
    ranges[angles > 0.18] = 4.0
    decision = teacher.decide(ranges, 0.0, 0.6)
    assert decision.active
    assert decision.reason == "gap-selected"
    assert decision.target_angle_rad < 0.0
    assert decision.steering_rad < 0.0


def test_close_obstacle_commands_teacher_brake() -> None:
    teacher = LidarGapTeacher(GapTeacherConfig())
    angles = _angles()
    ranges = np.full(750, 10.0)
    ranges[np.abs(angles) <= 0.18] = 1.0
    decision = teacher.decide(ranges, 0.0, 0.6)
    assert decision.active
    assert decision.acceleration_mps2 == pytest.approx(-1.0)


def test_longitudinal_safety_has_no_lateral_output_and_preserves_clear_accel() -> None:
    safety = LidarLongitudinalSafety(GapTeacherConfig())
    decision = safety.decide(np.full(750, 30.0), 0.6)
    assert not decision.active
    assert decision.reason == "clear"
    assert decision.acceleration_mps2 == pytest.approx(0.6)


def test_longitudinal_safety_inhibits_and_brakes_at_shared_thresholds() -> None:
    safety = LidarLongitudinalSafety(GapTeacherConfig())
    slow = safety.decide(np.full(750, 2.0), 0.6)
    stop = safety.decide(np.full(750, 1.0), 0.6)
    assert slow.active
    assert slow.reason == "slow-clearance"
    assert slow.acceleration_mps2 == pytest.approx(0.0)
    assert stop.active
    assert stop.reason == "stop-clearance"
    assert stop.acceleration_mps2 == pytest.approx(-1.0)


def test_left_side_wall_retains_teacher_authority_and_steers_right() -> None:
    teacher = LidarGapTeacher(GapTeacherConfig())
    angles = _angles()
    ranges = np.full(750, 30.0)
    ranges[angles >= 1.0] = 1.2
    decision = teacher.decide(ranges, 0.45, 0.6)
    assert decision.active
    assert decision.reason == "side-clearance"
    assert decision.left_side_distance_m == pytest.approx(1.2)
    assert decision.right_side_distance_m == pytest.approx(30.0)
    assert decision.steering_rad < 0.0
    assert decision.acceleration_mps2 == pytest.approx(0.6)


def test_right_side_wall_steers_left() -> None:
    teacher = LidarGapTeacher(GapTeacherConfig())
    angles = _angles()
    ranges = np.full(750, 30.0)
    ranges[angles <= -1.0] = 1.2
    decision = teacher.decide(ranges, -0.45, 0.6)
    assert decision.active
    assert decision.reason == "side-clearance"
    assert decision.steering_rad > 0.0


def test_precontact_teacher_detects_narrow_supported_side_return() -> None:
    teacher = LidarPrecontactTeacher(GapTeacherConfig(side_cluster_points=3))
    angles = _angles()
    ranges = np.full(750, 30.0)
    right_side_indices = np.flatnonzero(angles <= -1.3)
    ranges[right_side_indices[-3:]] = 1.4
    decision = teacher.decide(ranges, -0.45, 0.6)
    assert decision.active
    assert decision.reason == "side-clearance"
    assert decision.right_side_distance_m == pytest.approx(1.4)
    assert decision.steering_rad > 0.0


def test_precontact_teacher_rejects_two_isolated_side_rays() -> None:
    teacher = LidarPrecontactTeacher(GapTeacherConfig(side_cluster_points=3))
    angles = _angles()
    ranges = np.full(750, 30.0)
    right_side_indices = np.flatnonzero(angles <= -1.3)
    ranges[right_side_indices[-2:]] = 0.5
    decision = teacher.decide(ranges, -0.45, 0.6)
    assert not decision.active
    assert decision.reason == "front-clear"
    assert decision.steering_rad == pytest.approx(-0.45)


def test_no_gap_keeps_lateral_base_but_does_not_accelerate() -> None:
    teacher = LidarGapTeacher(GapTeacherConfig())
    decision = teacher.decide(np.full(750, 2.0), 0.2, 0.6)
    assert decision.active
    assert decision.reason == "no-gap"
    assert decision.steering_rad == pytest.approx(0.2)
    assert decision.acceleration_mps2 == pytest.approx(0.0)


def test_invalid_distance_order_is_rejected() -> None:
    with pytest.raises(ValueError, match="stop < slow < trigger"):
        GapTeacherConfig(
            trigger_distance_m=3.0,
            slow_distance_m=2.0,
            stop_distance_m=2.5,
        )


def test_invalid_side_distance_order_is_rejected() -> None:
    with pytest.raises(ValueError, match="critical < trigger"):
        GapTeacherConfig(
            side_trigger_distance_m=1.0,
            side_critical_distance_m=1.2,
        )


@pytest.mark.parametrize("value", [0, -1, True, 1.5])
def test_invalid_side_cluster_points_are_rejected(value) -> None:
    with pytest.raises(ValueError, match="side_cluster_points"):
        GapTeacherConfig(side_cluster_points=value)
