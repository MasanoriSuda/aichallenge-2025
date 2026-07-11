from dataclasses import FrozenInstanceError
from enum import Enum
from pathlib import Path
from types import SimpleNamespace

import pytest

from multi_purpose_mpc_ros.tools.trajectory_clearance_dialog import (
    ClearanceDialogConfig,
)
from multi_purpose_mpc_ros.tools.trajectory_clearance_dialog import (
    UnknownCellPolicy,
)
from multi_purpose_mpc_ros.tools.trajectory_clearance_dialog import (
    clearance_issue_row,
)
from multi_purpose_mpc_ros.tools.trajectory_clearance_dialog import (
    clearance_report_summary,
)
from multi_purpose_mpc_ros.tools.trajectory_clearance_dialog import (
    load_provisional_vehicle_extents,
)
from multi_purpose_mpc_ros.tools.trajectory_clearance_dialog import (
    parse_clearance_dialog_config,
)


def _write_vehicle_yaml(path: Path, **overrides: object) -> None:
    values = {
        "wheel_base": 1.087,
        "front_overhang": 0.467,
        "rear_overhang": 0.510,
        "wheel_tread": 1.12,
        "left_overhang": 0.09,
        "right_overhang": 0.09,
        **overrides,
    }
    lines = ["/**:", "  ros__parameters:"]
    lines.extend(f"    {name}: {value}" for name, value in values.items())
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _valid_raw(map_path: Path, vehicle_path: Path) -> dict[str, object]:
    return {
        "map_yaml_path": str(map_path),
        "vehicle_yaml_path": str(vehicle_path),
        "wheel_base_m": "1.087",
        "wheel_tread_m": "1.12",
        "front_extent_m": "1.554",
        "rear_extent_m": "0.510",
        "left_extent_m": "0.650",
        "right_extent_m": "0.650",
        "margin_front_m": "0.1",
        "margin_rear_m": "0.2",
        "margin_left_m": "0.3",
        "margin_right_m": "0.4",
        "unknown_policy": "occupied",
        "sweep_step_m": "0.05",
        "max_lateral_shift_m": "0.5",
        "offset_step_m": "0.05",
        "smoothness_weight": "2.0",
    }


def test_vehicle_yaml_derives_provisional_rear_axle_extents(tmp_path: Path) -> None:
    path = tmp_path / "vehicle.yaml"
    _write_vehicle_yaml(path)

    extents = load_provisional_vehicle_extents(path)

    assert extents.front_extent_m == pytest.approx(1.554)
    assert extents.rear_extent_m == pytest.approx(0.510)
    assert extents.left_extent_m == pytest.approx(0.650)
    assert extents.right_extent_m == pytest.approx(0.650)


@pytest.mark.parametrize(
    ("override", "message"),
    (
        ({"wheel_base": "nan"}, "wheel_base must be finite"),
        ({"wheel_tread": -1}, "wheel_tread must be greater than"),
    ),
)
def test_vehicle_yaml_rejects_invalid_dimensions(
    tmp_path: Path,
    override: dict[str, object],
    message: str,
) -> None:
    path = tmp_path / "vehicle.yaml"
    _write_vehicle_yaml(path, **override)

    with pytest.raises(ValueError, match=message):
        load_provisional_vehicle_extents(path)


def test_vehicle_yaml_requires_all_geometry_fields(tmp_path: Path) -> None:
    path = tmp_path / "vehicle.yaml"
    path.write_text(
        "/**:\n  ros__parameters:\n    wheel_base: 1.0\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="front_overhang"):
        load_provisional_vehicle_extents(path)


def test_config_parser_returns_frozen_explicit_settings(tmp_path: Path) -> None:
    map_path = tmp_path / "map.yaml"
    vehicle_path = tmp_path / "vehicle.yaml"
    map_path.write_text("image: map.pgm\n", encoding="utf-8")
    _write_vehicle_yaml(vehicle_path)

    config = parse_clearance_dialog_config(
        **_valid_raw(map_path, vehicle_path), require_existing_paths=True
    )

    assert isinstance(config, ClearanceDialogConfig)
    assert config.unknown_policy is UnknownCellPolicy.OCCUPIED
    assert config.unknown_is_occupied
    assert config.body_length_m == pytest.approx(2.064)
    assert config.body_width_m == pytest.approx(1.3)
    assert config.envelope_length_m == pytest.approx(2.364)
    assert config.envelope_width_m == pytest.approx(2.0)
    assert config.vehicle_footprint_kwargs()["front_overhang_m"] == pytest.approx(
        0.467
    )
    assert config.adjustment_parameter_kwargs(circular=True) == {
        "max_lateral_shift_m": 0.5,
        "sampling_step_m": 0.05,
        "displacement_weight": 1.0,
        "smoothness_weight": 2.0,
        "curvature_weight": 1.0,
            "circular": True,
            "sweep_step_m": 0.05,
            "max_abs_curvature_radpm": 0.7,
        }
    with pytest.raises(FrozenInstanceError):
        config.front_extent_m = 4.0  # type: ignore[misc]


@pytest.mark.parametrize(
    ("field", "value", "message"),
    (
        ("map_yaml_path", "", "path is required"),
        ("margin_left_m", "-0.01", "left margin must be at least"),
        ("sweep_step_m", "0", "sweep step must be greater"),
        ("unknown_policy", "ignore", "unknown policy must be one of"),
        ("offset_step_m", "2", "offset step must not exceed"),
    ),
)
def test_config_parser_rejects_unsafe_or_invalid_input(
    tmp_path: Path,
    field: str,
    value: object,
    message: str,
) -> None:
    raw = _valid_raw(tmp_path / "map.yaml", tmp_path / "vehicle.yaml")
    raw[field] = value

    with pytest.raises(ValueError, match=message):
        parse_clearance_dialog_config(**raw)


class _Severity(Enum):
    ERROR = "error"


def test_generic_issue_and_report_adapters_are_headless() -> None:
    issue = SimpleNamespace(
        severity=_Severity.ERROR,
        code="FOOTPRINT_COLLISION",
        point_index=12,
        segment_index=None,
        s_m=3.5,
        clearance_m=-0.02,
        required_margin_m=0.1,
        grid_cell=(7, 8),
        message="vehicle envelope intersects an occupied cell",
    )
    report = SimpleNamespace(
        is_safe=False,
        minimum_clearance_m=-0.02,
        colliding_point_count=1,
        colliding_segment_count=0,
        unknown_contact_count=0,
        outside_map_count=0,
        issues=(issue,),
    )

    row = clearance_issue_row(issue)
    summary = clearance_report_summary(report)

    assert row[0] == "error"
    assert row[1] == "FOOTPRINT_COLLISION"
    assert row[2] == "12"
    assert row[-1].startswith("vehicle envelope")
    assert "status=UNSAFE" in summary
    assert "issues=1" in summary
    assert "minimum raw clearance=-0.02 m" in summary
