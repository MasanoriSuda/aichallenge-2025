"""Unit tests for the teacher-only LiDAR gap residual."""

import numpy as np
import pytest

from tiny_lidar_net_controller.gap_teacher import (
    GapTeacherConfig,
    LidarGapTeacher,
    LidarLongitudinalSafety,
    LidarPrecontactTeacher,
)
from tiny_lidar_net_controller.speed_committed_teacher import (
    LidarSpeedCommittedTeacher,
    SpeedCommittedTeacherConfig,
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


def _front_obstacle_with_preferred_side(side_sign: int) -> np.ndarray:
    angles = _angles()
    ranges = np.full(750, 12.0)
    ranges[np.abs(angles) <= 0.18] = 5.0
    if side_sign > 0:
        ranges[angles < -0.18] = 4.0
    else:
        ranges[angles > 0.18] = 4.0
    return ranges


def test_speed_committed_distances_follow_braking_physics() -> None:
    teacher = LidarSpeedCommittedTeacher(GapTeacherConfig())
    stopped = teacher.dynamic_distances(0.0)
    moving = teacher.dynamic_distances(4.0)
    assert stopped == pytest.approx((1.5, 3.0, 7.0))
    assert moving == pytest.approx((10.5, 11.5, 13.5))
    assert moving[0] > stopped[0]
    assert moving[0] < moving[1] < moving[2]


def test_speed_committed_teacher_requires_speed() -> None:
    teacher = LidarSpeedCommittedTeacher(GapTeacherConfig())
    with pytest.raises(ValueError, match="requires fresh wheel speed"):
        teacher.decide(np.full(750, 30.0), 0.1, 0.6, None)


def test_speed_committed_clear_scene_preserves_base_command() -> None:
    teacher = LidarSpeedCommittedTeacher(GapTeacherConfig())
    decision = teacher.decide(np.full(750, 30.0), 0.17, 0.6, 4.0)
    assert not decision.active
    assert decision.steering_rad == pytest.approx(0.17)
    assert decision.acceleration_mps2 == pytest.approx(0.6)
    assert decision.committed_side_sign == 0
    assert decision.supervisor_reason == "clear"


def test_speed_committed_teacher_confirms_early_side_reversal() -> None:
    teacher = LidarSpeedCommittedTeacher(GapTeacherConfig())
    first = teacher.decide(
        _front_obstacle_with_preferred_side(1), 0.0, 0.6, 3.0
    )
    assert first.proposed_side_sign == 1
    assert first.committed_side_sign == 1

    pending = teacher.decide(
        _front_obstacle_with_preferred_side(-1), 0.0, 0.6, 3.0
    )
    assert pending.proposed_side_sign == -1
    assert pending.committed_side_sign == 1
    assert pending.supervisor_reason == "side-switch-pending"
    assert pending.steering_rad == pytest.approx(0.0)
    assert pending.acceleration_mps2 == pytest.approx(0.0)

    confirmed = teacher.decide(
        _front_obstacle_with_preferred_side(-1), 0.0, 0.6, 3.0
    )
    assert confirmed.committed_side_sign == -1
    assert confirmed.supervisor_reason == "side-switch-confirmed"
    assert confirmed.steering_rad < 0.0
    assert np.sign(confirmed.steering_rad) == confirmed.proposed_side_sign


def test_speed_committed_teacher_blocks_side_reversal_inside_slow_envelope() -> None:
    teacher = LidarSpeedCommittedTeacher(GapTeacherConfig())
    teacher.decide(_front_obstacle_with_preferred_side(1), 0.0, 0.6, 3.0)
    close_opposite = _front_obstacle_with_preferred_side(-1)
    close_opposite[np.abs(_angles()) <= 0.18] = 2.0
    blocked = teacher.decide(close_opposite, 0.0, 0.6, 3.0)
    assert blocked.supervisor_reason == "late-side-switch-blocked"
    assert blocked.steering_rad == pytest.approx(0.0)
    assert blocked.acceleration_mps2 == pytest.approx(-1.0)


def test_speed_committed_teacher_releases_only_after_confirmed_clear() -> None:
    config = SpeedCommittedTeacherConfig(release_confirmation_samples=3)
    teacher = LidarSpeedCommittedTeacher(GapTeacherConfig(), config)
    teacher.decide(_front_obstacle_with_preferred_side(1), 0.0, 0.6, 3.0)
    for expected_count in (1, 2):
        decision = teacher.decide(np.full(750, 30.0), 0.1, 0.6, 3.0)
        assert decision.committed_side_sign == 1
        assert decision.clear_confirmation_samples == expected_count
    released = teacher.decide(np.full(750, 30.0), 0.1, 0.6, 3.0)
    assert released.committed_side_sign == 0
    assert released.clear_confirmation_samples == 0


def test_speed_committed_teacher_brakes_for_bilateral_side_pinch() -> None:
    teacher = LidarSpeedCommittedTeacher(GapTeacherConfig())
    angles = _angles()
    ranges = np.full(750, 30.0)
    ranges[angles >= 1.3] = 1.5
    ranges[angles <= -1.3] = 1.5
    decision = teacher.decide(ranges, 0.0, 0.6, 3.0)
    assert decision.supervisor_reason == "bilateral-side-pinch"
    assert decision.acceleration_mps2 == pytest.approx(-1.0)


def test_speed_committed_teacher_extends_geometry_only_for_detected_hazard() -> None:
    teacher = LidarSpeedCommittedTeacher(GapTeacherConfig())
    angles = _angles()
    ranges = np.full(750, 12.0)
    ranges[np.abs(angles) <= 0.18] = 8.5
    ranges[angles >= 1.3] = 1.5
    decision = teacher.decide(ranges, 0.0, 0.6, 3.0)
    assert decision.front_distance_m < decision.dynamic_trigger_distance_m
    assert decision.reason == "gap-selected"
    assert decision.active


@pytest.mark.parametrize("value", [0, -1, True, 1.5])
def test_invalid_side_cluster_points_are_rejected(value) -> None:
    with pytest.raises(ValueError, match="side_cluster_points"):
        GapTeacherConfig(side_cluster_points=value)
