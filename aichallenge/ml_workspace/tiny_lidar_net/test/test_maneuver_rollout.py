import math

import numpy as np
import pytest

from lib.maneuver_rollout import (
    ManeuverRolloutConfig,
    evaluate_candidate,
    evaluate_candidate_against_time_indexed_points,
    point_clearance_to_footprint,
    rollout_lane_change_stop,
    scan_points_in_base,
    select_best_candidate,
)


def test_scan_points_apply_lidar_offset_and_ignore_max_range() -> None:
    config = ManeuverRolloutConfig()
    ranges = np.asarray([30.0, 2.0, 30.0])

    points = scan_points_in_base(ranges, config)

    assert points.shape == (1, 2)
    assert points[0] == pytest.approx([3.65, 0.0])


def test_rollout_finishes_stopped_and_countersteers() -> None:
    config = ManeuverRolloutConfig()
    states = rollout_lane_change_stop(4.0, 0.0, 0.20, config)

    assert states[-1, 3] == pytest.approx(0.0)
    assert states[-1, 0] > 8.0
    assert states[-1, 1] > 0.5
    assert abs(states[-1, 2]) < 0.15


def test_signed_rectangle_clearance() -> None:
    config = ManeuverRolloutConfig()
    state = np.asarray([0.0, 0.0, 0.0, 0.0])
    points = np.asarray(
        [
            [config.front_extent_m + 0.4, 0.0],
            [0.0, config.vehicle_half_width_m + 0.3],
            [0.0, 0.0],
        ]
    )

    clearance = point_clearance_to_footprint(points, state, config)

    assert clearance[0] == pytest.approx(0.4)
    assert clearance[1] == pytest.approx(0.3)
    assert clearance[2] < 0.0


def test_candidate_rejects_obstacle_inside_swept_footprint() -> None:
    config = ManeuverRolloutConfig(clearance_margin_m=0.15)
    obstacle = np.asarray([[4.0, 0.0]])

    candidate = evaluate_candidate(obstacle, 2.0, 0.0, 0.0, config)

    assert not candidate.feasible
    assert candidate.minimum_clearance_m < config.clearance_margin_m


def test_open_side_candidate_is_selected_over_blocked_side() -> None:
    config = ManeuverRolloutConfig(clearance_margin_m=0.15)
    # Points form a forward/right obstruction; the positive-offset lane change
    # moves left in this coordinate convention.
    points = np.asarray([[4.0, -0.4], [5.0, -0.8], [6.0, -1.0]])
    right = evaluate_candidate(points, 3.0, 0.0, -0.24, config)
    left = evaluate_candidate(points, 3.0, 0.0, 0.24, config)

    selected = select_best_candidate([right, left])

    assert left.feasible
    assert selected is left


def test_time_indexed_points_do_not_freeze_a_departing_obstacle() -> None:
    config = ManeuverRolloutConfig(clearance_margin_m=0.15)
    static_obstacle = np.asarray([[4.0, 0.0]])
    frozen = evaluate_candidate(static_obstacle, 2.0, 0.0, 0.0, config)
    states = rollout_lane_change_stop(2.0, 0.0, 0.0, config)
    time_indexed = [static_obstacle] + [np.empty((0, 2))] * (len(states) - 1)

    moving = evaluate_candidate_against_time_indexed_points(
        time_indexed,
        2.0,
        0.0,
        0.0,
        config,
    )

    assert not frozen.feasible
    assert moving.feasible


def test_invalid_physical_contract_is_rejected() -> None:
    with pytest.raises(ValueError, match="scan range bounds"):
        ManeuverRolloutConfig(
            minimum_scan_range_m=30.0,
            maximum_scan_range_m=30.0,
        )
    with pytest.raises(ValueError, match="speed_mps"):
        rollout_lane_change_stop(
            math.nan,
            0.0,
            0.0,
            ManeuverRolloutConfig(),
        )
