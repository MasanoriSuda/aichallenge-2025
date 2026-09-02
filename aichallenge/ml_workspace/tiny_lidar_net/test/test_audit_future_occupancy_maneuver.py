import math

import numpy as np
import pytest

from audit_future_occupancy_maneuver import (
    classify_future_occupancy_oracle,
    future_point_clouds_for_rollout,
    quaternion_yaw,
    transform_points_to_reference,
)
from lib.maneuver_rollout import ManeuverRolloutConfig


def test_quaternion_yaw() -> None:
    yaw = 0.7
    assert quaternion_yaw(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)) == pytest.approx(yaw)


def test_transform_points_through_world_into_reference() -> None:
    points = np.asarray([[1.0, 0.0]])
    future_pose = np.asarray([2.0, 0.0, math.pi / 2.0])
    reference_pose = np.asarray([0.0, 0.0, 0.0])

    transformed = transform_points_to_reference(points, future_pose, reference_pose)

    assert transformed[0] == pytest.approx([2.0, 1.0])


def test_future_cloud_builder_rejects_missing_horizon() -> None:
    config = ManeuverRolloutConfig()
    scans = np.full((2, 750), 30.0, dtype=np.float32)
    clouds = future_point_clouds_for_rollout(
        current_scan_index=1,
        state_count=3,
        scan_times_ns=np.asarray([0, 100_000_000]),
        scans_m=scans,
        pose_times_ns=np.asarray([0, 100_000_000]),
        poses_xyyaw=np.zeros((2, 3)),
        config=config,
        maximum_pose_age_sec=0.1,
    )

    assert clouds is None


def test_future_cloud_builder_aligns_pose_and_scan() -> None:
    config = ManeuverRolloutConfig(integration_step_sec=0.1)
    scans = np.full((3, 750), 30.0, dtype=np.float32)
    scans[:, 375] = 2.0
    times = np.asarray([0, 100_000_000, 200_000_000])
    poses = np.asarray([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])

    clouds = future_point_clouds_for_rollout(
        current_scan_index=0,
        state_count=3,
        scan_times_ns=times,
        scans_m=scans,
        pose_times_ns=times,
        poses_xyyaw=poses,
        config=config,
        maximum_pose_age_sec=0.1,
    )

    assert clouds is not None
    assert len(clouds) == 3
    assert clouds[1][0, 0] > clouds[0][0, 0] + 0.9


def test_oracle_rejects_signal_when_candidate_bank_misses_success() -> None:
    reports = {
        "success": {
            "any_candidate_available_fraction": 0.15,
            "selected_infeasible_opposite_feasible": {"fraction": 0.05},
        },
        "failed": {
            "any_candidate_available_fraction": 0.95,
            "selected_infeasible_opposite_feasible": {"fraction": 0.18},
        },
    }

    result = classify_future_occupancy_oracle(reports, "failed")

    assert result["temporal_signal_observed"]
    assert not result["candidate_bank_represents_success"]
    assert not result["future_occupancy_discriminates_failure"]
    assert result["classification"] == "inconclusive-candidate-bank-misses-success"


def test_oracle_accepts_signal_only_with_success_coverage() -> None:
    reports = {
        "success": {
            "any_candidate_available_fraction": 0.8,
            "selected_infeasible_opposite_feasible": {"fraction": 0.04},
        },
        "failed": {
            "any_candidate_available_fraction": 0.8,
            "selected_infeasible_opposite_feasible": {"fraction": 0.15},
        },
    }

    result = classify_future_occupancy_oracle(reports, "failed")

    assert result["candidate_bank_represents_success"]
    assert result["future_occupancy_discriminates_failure"]
