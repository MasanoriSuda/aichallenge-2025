from __future__ import annotations

from concurrent.futures import CancelledError
import math
from pathlib import Path

import pytest

from multi_purpose_mpc_ros.tools.trajectory_clearance import AdjustmentParameters
from multi_purpose_mpc_ros.tools.trajectory_clearance import AdjustmentStatus
from multi_purpose_mpc_ros.tools.trajectory_clearance import CellState
from multi_purpose_mpc_ros.tools.trajectory_clearance import MapLoadOptions
from multi_purpose_mpc_ros.tools.trajectory_clearance import Pose2D
from multi_purpose_mpc_ros.tools.trajectory_clearance import ValidationOptions
from multi_purpose_mpc_ros.tools.trajectory_clearance import VehicleFootprintSpec
from multi_purpose_mpc_ros.tools.trajectory_clearance import adjust_clearance
from multi_purpose_mpc_ros.tools.trajectory_clearance import footprint_polygon
from multi_purpose_mpc_ros.tools.trajectory_clearance import load_occupancy_grid
from multi_purpose_mpc_ros.tools.trajectory_clearance import load_vehicle_footprint
from multi_purpose_mpc_ros.tools.trajectory_clearance import validate_clearance


def _write_map(
    directory: Path,
    *,
    width: int,
    height: int,
    pixels: list[int],
    resolution: float = 1.0,
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
    encoding: str = "P2",
) -> Path:
    assert len(pixels) == width * height
    pgm = directory / "map.pgm"
    if encoding == "P2":
        pgm.write_text(
            f"P2\n# synthetic fixture\n{width} {height}\n255\n"
            + " ".join(str(value) for value in pixels)
            + "\n",
            encoding="ascii",
        )
    else:
        pgm.write_bytes(
            f"P5\n# synthetic fixture\n{width} {height}\n255\n".encode("ascii")
            + bytes(pixels)
        )
    yaml_path = directory / "map.yaml"
    yaml_path.write_text(
        "\n".join(
            (
                "image: map.pgm",
                f"resolution: {resolution}",
                f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            )
        ),
        encoding="utf-8",
    )
    return yaml_path


def _vehicle(**margins: float) -> VehicleFootprintSpec:
    return VehicleFootprintSpec(
        reference_point="rear_axle",
        wheel_base_m=0.4,
        front_overhang_m=0.0,
        rear_overhang_m=0.4,
        wheel_tread_m=0.8,
        left_overhang_m=0.0,
        right_overhang_m=0.0,
        margin_front_m=margins.get("front", 0.0),
        margin_rear_m=margins.get("rear", 0.0),
        margin_left_m=margins.get("left", 0.0),
        margin_right_m=margins.get("right", 0.0),
    )


def _free_pixels(width: int, height: int) -> list[int]:
    return [255] * (width * height)


def _set_map_cell(
    pixels: list[int], width: int, height: int, column: int, map_y: int, value: int
) -> None:
    row = height - 1 - map_y
    pixels[row * width + column] = value


@pytest.mark.parametrize("encoding", ["P2", "P5"])
def test_map_parser_classifies_pixels_and_inverts_image_y(
    tmp_path: Path, encoding: str
) -> None:
    # Top-left is occupied; bottom-left is free.
    path = _write_map(
        tmp_path,
        width=3,
        height=2,
        pixels=[0, 180, 255, 255, 255, 255],
        origin=(10.0, 20.0, 0.0),
        encoding=encoding,
    )
    grid = load_occupancy_grid(
        path, MapLoadOptions(fill_free_holes_below_cells=0)
    )

    assert grid.state(0, 0) is CellState.OCCUPIED
    assert grid.state(0, 1) is CellState.UNKNOWN
    assert grid.state(1, 0) is CellState.FREE
    assert grid.cell_center_world(1, 0) == pytest.approx((10.0, 20.0))
    assert grid.world_to_cell(10.0, 20.0) == (1, 0)
    assert grid.world_to_cell(10.0, 21.0) == (0, 0)


def test_map_origin_yaw_round_trip(tmp_path: Path) -> None:
    path = _write_map(
        tmp_path,
        width=3,
        height=3,
        pixels=_free_pixels(3, 3),
        origin=(10.0, 20.0, math.pi / 2.0),
    )
    grid = load_occupancy_grid(path)
    world = grid.cell_center_world(1, 1)

    assert world == pytest.approx((9.0, 21.0))
    assert grid.world_to_cell(*world) == (1, 1)


def test_runtime_small_occupied_island_filter(tmp_path: Path) -> None:
    pixels = _free_pixels(5, 5)
    _set_map_cell(pixels, 5, 5, 2, 2, 0)
    path = _write_map(tmp_path, width=5, height=5, pixels=pixels)

    parity = load_occupancy_grid(path)
    raw = load_occupancy_grid(
        path, MapLoadOptions(fill_free_holes_below_cells=0)
    )

    assert parity.state(2, 2) is CellState.FREE
    assert raw.state(2, 2) is CellState.OCCUPIED
    assert parity.spec.signature != raw.spec.signature


def test_runtime_pixel_normalization_uses_image_maximum(tmp_path: Path) -> None:
    path = _write_map(
        tmp_path,
        width=2,
        height=2,
        pixels=[0, 100, 100, 100],
    )
    grid = load_occupancy_grid(
        path, MapLoadOptions(fill_free_holes_below_cells=0)
    )

    assert grid.state(0, 0) is CellState.OCCUPIED
    assert grid.state(0, 1) is CellState.FREE


def test_runtime_parity_filters_small_gray_wall_component(tmp_path: Path) -> None:
    pixels = _free_pixels(3, 3)
    _set_map_cell(pixels, 3, 3, 1, 1, 128)
    path = _write_map(tmp_path, width=3, height=3, pixels=pixels)

    raw = load_occupancy_grid(
        path, MapLoadOptions(fill_free_holes_below_cells=0)
    )
    filtered = load_occupancy_grid(path)

    assert raw.state(1, 1) is CellState.OCCUPIED
    assert filtered.state(1, 1) is CellState.FREE


def test_vehicle_yaml_and_footprint_extents(tmp_path: Path) -> None:
    vehicle_yaml = tmp_path / "vehicle.yaml"
    vehicle_yaml.write_text(
        """/**:
  ros__parameters:
    wheel_base: 1.087
    front_overhang: 0.467
    rear_overhang: 0.510
    wheel_tread: 1.12
    left_overhang: 0.09
    right_overhang: 0.09
""",
        encoding="utf-8",
    )
    vehicle = load_vehicle_footprint(vehicle_yaml, margin_left_m=0.1)
    polygon = footprint_polygon(Pose2D(0.0, 0.0, 0.0), vehicle)

    assert vehicle.front_extent_m == pytest.approx(1.554)
    assert vehicle.rear_extent_m == pytest.approx(0.510)
    assert vehicle.left_extent_m == pytest.approx(0.650)
    assert max(point[1] for point in polygon) == pytest.approx(0.750)


def test_margin_violation_is_distinct_from_body_collision(tmp_path: Path) -> None:
    width = height = 10
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 5, 5, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(fill_free_holes_below_cells=0),
    )
    vehicle = _vehicle(front=1.5)
    poses = (Pose2D(3.0, 5.0, 0.0), Pose2D(3.1, 5.0, 0.0))

    report = validate_clearance(
        grid, poses, vehicle, ValidationOptions(include_sweep=False)
    )
    codes = {issue.code for issue in report.issues}

    assert "CLEARANCE_MARGIN_VIOLATION" in codes
    assert "FOOTPRINT_COLLISION" not in codes
    assert not report.is_safe


def test_swept_collision_between_safe_endpoints(tmp_path: Path) -> None:
    width = height = 12
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 5, 5, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(fill_free_holes_below_cells=0),
    )
    poses = (Pose2D(3.0, 5.0, 0.0), Pose2D(7.0, 5.0, 0.0))

    report = validate_clearance(
        grid,
        poses,
        _vehicle(),
        ValidationOptions(sweep_step_m=0.25, include_sweep=True),
    )

    assert report.colliding_point_count == 0
    assert report.colliding_segment_count == 1
    assert "SWEPT_FOOTPRINT_COLLISION" in {issue.code for issue in report.issues}


def test_simultaneous_translation_and_rotation_uses_conservative_sweep(
    tmp_path: Path,
) -> None:
    width = height = 30
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 12, 15, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(fill_free_holes_below_cells=0),
    )
    long_vehicle = VehicleFootprintSpec(
        reference_point="rear_axle",
        wheel_base_m=2.0,
        front_overhang_m=0.0,
        rear_overhang_m=0.3,
        wheel_tread_m=0.4,
        left_overhang_m=0.0,
        right_overhang_m=0.0,
    )

    report = validate_clearance(
        grid,
        (
            Pose2D(14.0, 14.0, 0.0),
            Pose2D(12.0, 12.0, 2.0 * math.pi / 3.0),
        ),
        long_vehicle,
        ValidationOptions(sweep_step_m=0.5),
    )

    assert not report.is_safe
    assert report.colliding_segment_count == 1


def test_circular_duplicate_xy_with_different_yaw_checks_rotation_seam(
    tmp_path: Path,
) -> None:
    width = height = 30
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 15, 12, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(fill_free_holes_below_cells=0),
    )
    long_vehicle = VehicleFootprintSpec(
        reference_point="rear_axle",
        wheel_base_m=2.0,
        front_overhang_m=0.0,
        rear_overhang_m=0.3,
        wheel_tread_m=0.4,
        left_overhang_m=0.0,
        right_overhang_m=0.0,
    )
    poses = (
        Pose2D(14.0, 14.0, 0.0),
        Pose2D(20.0, 14.0, 0.0),
        Pose2D(20.0, 20.0, math.pi / 2.0),
        Pose2D(14.0, 20.0, math.pi),
        Pose2D(14.0, 14.0, -math.pi / 2.0),
    )

    report = validate_clearance(
        grid,
        poses,
        long_vehicle,
        ValidationOptions(circular=True, sweep_step_m=0.25),
    )

    assert not report.is_safe
    assert any(issue.segment_index == 4 for issue in report.issues)


def test_minimum_clearance_scans_inside_rotated_footprint_aabb(
    tmp_path: Path,
) -> None:
    width = height = 30
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 15, 13, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(fill_free_holes_below_cells=0),
    )
    long_vehicle = VehicleFootprintSpec(
        reference_point="rear_axle",
        wheel_base_m=4.0,
        front_overhang_m=0.0,
        rear_overhang_m=0.4,
        wheel_tread_m=0.8,
        left_overhang_m=0.0,
        right_overhang_m=0.0,
    )

    report = validate_clearance(
        grid,
        (
            Pose2D(15.0, 15.0, math.pi / 4.0),
            Pose2D(15.1, 15.1, math.pi / 4.0),
        ),
        long_vehicle,
        ValidationOptions(include_sweep=False),
    )

    assert report.minimum_clearance_m is not None
    assert 0.0 < report.minimum_clearance_m < 1.0


def test_unknown_policy_is_explicit_and_safe_side_by_default(tmp_path: Path) -> None:
    width = height = 8
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 4, 4, 180)
    path = _write_map(tmp_path, width=width, height=height, pixels=pixels)
    poses = (Pose2D(4.0, 4.0, 0.0), Pose2D(4.1, 4.0, 0.0))
    blocked = validate_clearance(
        load_occupancy_grid(path, MapLoadOptions(True, 0)),
        poses,
        _vehicle(),
        ValidationOptions(include_sweep=False),
    )
    overridden = validate_clearance(
        load_occupancy_grid(path, MapLoadOptions(False, 0)),
        poses,
        _vehicle(),
        ValidationOptions(include_sweep=False),
    )

    assert not blocked.is_safe
    assert overridden.is_safe
    assert {issue.severity.value for issue in overridden.issues} == {"warning"}


def test_unknown_warning_does_not_mask_later_swept_wall_collision(
    tmp_path: Path,
) -> None:
    width = height = 12
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 3, 5, 180)
    _set_map_cell(pixels, width, height, 7, 5, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(
            unknown_is_occupied=False,
            fill_free_holes_below_cells=0,
        ),
    )

    report = validate_clearance(
        grid,
        (Pose2D(1.0, 5.0, 0.0), Pose2D(9.0, 5.0, 0.0)),
        _vehicle(),
        ValidationOptions(sweep_step_m=0.25),
    )
    codes = {issue.code for issue in report.issues}

    assert "UNKNOWN_CELL_CONTACT" in codes
    assert "SWEPT_FOOTPRINT_COLLISION" in codes
    assert not report.is_safe


def test_outside_map_is_never_clamped_safe(tmp_path: Path) -> None:
    grid = load_occupancy_grid(
        _write_map(
            tmp_path, width=5, height=5, pixels=_free_pixels(5, 5)
        )
    )
    report = validate_clearance(
        grid,
        (Pose2D(-1.0, 2.0, 0.0), Pose2D(-0.9, 2.0, 0.0)),
        _vehicle(),
        ValidationOptions(include_sweep=False),
    )

    assert not report.is_safe
    assert report.outside_map_count == 2
    assert "FOOTPRINT_OUTSIDE_MAP" in {issue.code for issue in report.issues}


def test_adjustment_returns_detached_safe_candidate(tmp_path: Path) -> None:
    width = height = 12
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 5, 5, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(fill_free_holes_below_cells=0),
    )
    poses = (
        Pose2D(3.0, 5.0, 0.0),
        Pose2D(5.0, 5.0, 0.0),
        Pose2D(7.0, 5.0, 0.0),
    )
    before = tuple(poses)

    result = adjust_clearance(
        grid,
        poses,
        _vehicle(),
        AdjustmentParameters(
            max_lateral_shift_m=2.0,
            sampling_step_m=1.0,
            smoothness_weight=20.0,
            curvature_weight=5.0,
            sweep_step_m=0.25,
        ),
        source_revision=7,
    )

    assert result.status is AdjustmentStatus.FEASIBLE
    assert result.candidate is not None
    assert result.candidate.after_report.is_safe
    assert result.candidate.source_revision == 7
    assert result.candidate.max_shift_m <= 2.0
    assert tuple(poses) == before


def test_adjustment_rejects_cheapest_colliding_transition_and_uses_safe_one(
    tmp_path: Path,
) -> None:
    width = height = 12
    pixels = _free_pixels(width, height)
    _set_map_cell(pixels, width, height, 5, 5, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(fill_free_holes_below_cells=0),
    )

    result = adjust_clearance(
        grid,
        (Pose2D(3.0, 5.0, 0.0), Pose2D(7.0, 5.0, 0.0)),
        _vehicle(),
        AdjustmentParameters(
            max_lateral_shift_m=2.0,
            sampling_step_m=1.0,
            sweep_step_m=0.25,
        ),
    )

    assert result.status is AdjustmentStatus.FEASIBLE
    assert result.candidate is not None
    assert result.candidate.after_report.is_safe
    assert result.candidate.offsets != (0.0, 0.0)


def test_adjustment_reports_infeasible_without_candidate(tmp_path: Path) -> None:
    width = height = 12
    pixels = _free_pixels(width, height)
    for map_y in range(height):
        _set_map_cell(pixels, width, height, 5, map_y, 0)
    grid = load_occupancy_grid(
        _write_map(tmp_path, width=width, height=height, pixels=pixels),
        MapLoadOptions(fill_free_holes_below_cells=0),
    )
    poses = (
        Pose2D(4.0, 5.0, 0.0),
        Pose2D(5.0, 5.0, 0.0),
        Pose2D(6.0, 5.0, 0.0),
    )

    result = adjust_clearance(
        grid,
        poses,
        _vehicle(),
        AdjustmentParameters(max_lateral_shift_m=1.0, sampling_step_m=0.5),
    )

    assert result.status is AdjustmentStatus.INFEASIBLE
    assert result.candidate is None
    assert result.issues


def test_adjustment_honours_cooperative_cancellation(tmp_path: Path) -> None:
    grid = load_occupancy_grid(
        _write_map(
            tmp_path,
            width=8,
            height=8,
            pixels=_free_pixels(8, 8),
        )
    )

    with pytest.raises(CancelledError):
        adjust_clearance(
            grid,
            (Pose2D(2.0, 2.0, 0.0), Pose2D(6.0, 2.0, 0.0)),
            _vehicle(),
            AdjustmentParameters(
                max_lateral_shift_m=1.0,
                sampling_step_m=0.5,
            ),
            cancel_requested=lambda: True,
        )
