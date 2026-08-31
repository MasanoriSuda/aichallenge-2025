import importlib.util
from pathlib import Path

import numpy as np


MODULE_PATH = Path(__file__).parents[1] / "analyze_e2e_run.py"
SPEC = importlib.util.spec_from_file_location("analyze_e2e_run", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_nearest_values_selects_closest_sample():
    result = MODULE.nearest_values(
        np.array([4, 16, 29], dtype=np.int64),
        np.array([0, 10, 30], dtype=np.int64),
        np.array([1.0, 2.0, 3.0]),
    )
    np.testing.assert_array_equal(result, np.array([1.0, 2.0, 3.0]))


def test_longest_true_duration_does_not_bridge_recording_gap():
    times = np.array([0, 100, 200, 1000, 1100], dtype=np.int64) * 1_000_000
    mask = np.ones(5, dtype=bool)
    assert MODULE.longest_true_duration_sec(times, mask, 0.25) == 0.2


def test_longest_true_interval_reports_the_winning_segment():
    times = np.array([0, 100, 200, 1000, 1100, 1200], dtype=np.int64) * 1_000_000
    mask = np.array([True, True, False, True, True, True])
    assert MODULE.longest_true_interval_ns(times, mask, 0.25) == (
        1_000_000_000,
        1_200_000_000,
    )


def test_motion_summary_detects_positive_acceleration_stall_after_motion():
    times = np.arange(0, 81, dtype=np.int64) * 100_000_000
    speeds = np.concatenate(
        (np.array([0.0]), np.full(29, 1.2), np.full(51, 0.05))
    )
    result = MODULE.summarize_motion(
        times,
        speeds,
        times,
        np.full(times.shape, 0.6),
        moving_speed_mps=1.0,
        stall_speed_mps=0.15,
        positive_accel_mps2=0.2,
    )
    assert result["longest_positive_accel_stall_sec"] == 5.0
    assert result["positive_accel_stall_start_sec"] == 3.0
    assert result["positive_accel_stall_end_sec"] == 8.0
    assert result["positive_accel_stall_samples"] == 51


def test_failure_context_synchronizes_scan_control_and_pose():
    arrays = {
        "scan_times_ns": np.array([100, 300]),
        "scan_front_m": np.array([7.0, 2.0]),
        "scan_left_m": np.array([8.0, 3.0]),
        "scan_right_m": np.array([9.0, 4.0]),
        "scan_left_side_m": np.array([5.0, 1.0]),
        "scan_right_side_m": np.array([6.0, 2.0]),
        "scan_closest_m": np.array([5.0, 1.0]),
        "scan_closest_angle_rad": np.array([0.8, 1.2]),
        "control_times_ns": np.array([110, 310]),
        "accelerations_mps2": np.array([0.6, -1.0]),
        "steering_commands_rad": np.array([0.1, -0.2]),
        "pose_times_ns": np.array([90, 290]),
        "pose_x_m": np.array([1.0, 3.0]),
        "pose_y_m": np.array([2.0, 4.0]),
    }
    context = MODULE.failure_context(arrays, 280)
    assert context["scan"]["front_m"] == 2.0
    assert context["scan"]["closest_angle_rad"] == 1.2
    assert context["command"]["acceleration_mps2"] == -1.0
    assert context["pose"] == {"x_m": 3.0, "y_m": 4.0}


def test_scan_sector_minima_uses_physical_angles():
    angles = np.linspace(-1.0, 1.0, 9)
    ranges = np.full(9, 10.0)
    ranges[4] = 2.0
    ranges[5] = 3.0
    ranges[3] = 4.0
    front, left, right = MODULE.scan_sector_minima(
        ranges,
        angle_min=float(angles[0]),
        angle_increment=float(angles[1] - angles[0]),
        half_angle_rad=0.1,
        max_range_m=30.0,
    )
    assert front == 2.0
    assert left == 3.0
    assert right == 4.0


def test_scan_side_clearance_reports_closest_side_geometry():
    angles = np.linspace(-1.0, 1.0, 9)
    ranges = np.full(9, 10.0)
    ranges[0] = 1.5
    ranges[7] = 2.5
    left, right, closest, closest_angle = MODULE.scan_side_clearance(
        ranges,
        angle_min=float(angles[0]),
        angle_increment=float(angles[1] - angles[0]),
        max_range_m=30.0,
    )
    assert left == 2.5
    assert right == 1.5
    assert closest == 1.5
    assert closest_angle == -1.0
