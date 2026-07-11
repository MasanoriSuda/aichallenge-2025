import copy
import math
from pathlib import Path

import pytest

from multi_purpose_mpc_ros.tools.trajectory_contract import MPC_COLUMNS
from multi_purpose_mpc_ros.tools.trajectory_contract import (
    PURE_PURSUIT_COLUMNS,
)
from multi_purpose_mpc_ros.tools.trajectory_contract import TrajectoryData
from multi_purpose_mpc_ros.tools.trajectory_contract import validate_trajectory
from multi_purpose_mpc_ros.tools.trajectory_plot import PlotDataError
from multi_purpose_mpc_ros.tools.trajectory_plot import build_comparison_plot
from multi_purpose_mpc_ros.tools.trajectory_plot import build_trajectory_plot
from multi_purpose_mpc_ros.tools.trajectory_plot import nearest_index_by_s


def _mpc_data(
    values: list[
        tuple[float, float, float, float, float, float, float]
    ],
) -> TrajectoryData:
    rows = [
        {
            "s_m": str(s_m),
            "x_m": str(x_m),
            "y_m": str(y_m),
            "psi_rad": str(psi),
            "kappa_radpm": str(kappa),
            "vx_mps": str(velocity),
            "ax_mps2": str(acceleration),
        }
        for s_m, x_m, y_m, psi, kappa, velocity, acceleration in values
    ]
    return TrajectoryData(
        path=Path("trajectory.csv"),
        fieldnames=list(MPC_COLUMNS),
        rows=rows,
        points=[(float(row["x_m"]), float(row["y_m"])) for row in rows],
        x_column="x_m",
        y_column="y_m",
        format_name="mpc",
    )


def _pure_pursuit_data(
    values: list[tuple[float, float, float, float]],
) -> TrajectoryData:
    rows = []
    for x_m, y_m, yaw, speed in values:
        rows.append(
            {
                "x": str(x_m),
                "y": str(y_m),
                "z": "0.0",
                "x_quat": "0.0",
                "y_quat": "0.0",
                "z_quat": str(math.sin(yaw / 2.0)),
                "w_quat": str(math.cos(yaw / 2.0)),
                "speed": str(speed),
            }
        )
    return TrajectoryData(
        path=Path("pure.csv"),
        fieldnames=list(PURE_PURSUIT_COLUMNS),
        rows=rows,
        points=[(float(row["x"]), float(row["y"])) for row in rows],
        x_column="x",
        y_column="y",
        format_name="pure_pursuit",
    )


def _report(data: TrajectoryData, *, circular: bool):
    return validate_trajectory(
        data.fieldnames,
        data.rows,
        data.format_name,
        circular,
    )


def test_mpc_builds_all_seven_plot_views_from_validated_data() -> None:
    data = _mpc_data(
        [
            (0, 0, 0, 0.0, 0.0, 2.0, 0.5),
            (1, 1, 0, 0.1, 0.25, 3.0, -0.5),
            (3, 3, 0, 0.2, -0.5, 4.0, 0.0),
        ]
    )
    before = copy.deepcopy(data)
    report = _report(data, circular=False)

    plot = build_trajectory_plot(data, report)

    assert data == before
    assert plot.s_axis_m == (0.0, 1.0, 3.0)
    assert tuple(point.x_m for point in plot.xy.points) == (0.0, 1.0, 3.0)
    assert tuple(sample.value for sample in plot.spacing.samples) == (1.0, 2.0)
    assert tuple(sample.value for sample in plot.psi.samples) == (
        0.0,
        0.1,
        0.2,
    )
    assert tuple(sample.value for sample in plot.kappa.samples) == (
        0.0,
        0.25,
        -0.5,
    )
    assert tuple(sample.value for sample in plot.velocity.samples) == (
        2.0,
        3.0,
        4.0,
    )
    assert tuple(sample.value for sample in plot.acceleration.samples) == (
        0.5,
        -0.5,
        0.0,
    )
    lateral_values = tuple(
        sample.value for sample in plot.lateral_acceleration.samples
    )
    assert lateral_values == pytest.approx(
        (0.0, 2.25, -8.0)
    )
    assert plot.path_length_m == report.metrics.total_distance_m == 3.0

    comparison = build_comparison_plot(data, report, data, report)
    summary = comparison.summary
    assert summary.series("spacing").before.minimum == (
        report.metrics.min_spacing_m
    )
    assert summary.series("spacing").before.maximum == (
        report.metrics.max_spacing_m
    )
    assert summary.series("kappa").before.maximum_absolute == (
        report.metrics.max_abs_kappa_radpm
    )
    assert summary.series("lateral_acceleration").before.maximum_absolute == (
        report.metrics.max_lateral_acceleration_mps2
    )


def test_different_counts_map_selection_by_nearest_s_with_stable_tie() -> None:
    before = _mpc_data(
        [
            (0, 0, 0, 0, 0, 2, 0),
            (2, 2, 0, 0, 0, 2, 0),
            (4, 4, 0, 0, 0, 2, 0),
        ]
    )
    candidate = _mpc_data(
        [
            (0, 0, 0, 0, 0, 2, 0),
            (1, 1, 0, 0, 0, 2, 0),
            (2, 2, 0, 0, 0, 2, 0),
            (3, 3, 0, 0, 0, 2, 0),
            (4, 4, 0, 0, 0, 2, 0),
        ]
    )
    comparison = build_comparison_plot(
        before,
        _report(before, circular=False),
        candidate,
        _report(candidate, circular=False),
    )

    from_before = comparison.selection_from_before(1)
    from_candidate = comparison.selection_from_candidate(1)

    assert (from_before.before_index, from_before.candidate_index) == (1, 2)
    assert (
        from_candidate.before_index,
        from_candidate.candidate_index,
    ) == (0, 1)
    assert comparison.summary.before_point_count == 3
    assert comparison.summary.candidate_point_count == 5
    assert comparison.summary.point_count_delta == 2
    assert comparison.summary.path_length_delta_m == 0.0
    assert comparison.summary.max_displacement_m == pytest.approx(0.0)
    assert comparison.summary.series("spacing").before.maximum == 2.0
    assert comparison.summary.series("spacing").candidate.maximum == 1.0


def test_circular_duplicate_endpoint_normalizes_and_plots_seam() -> None:
    data = _mpc_data(
        [
            (0, 0, 0, 0, 0, 2, 0),
            (1, 1, 0, 0, 0, 2, 0),
            (2, 1, 1, math.pi / 2, 0, 2, 0),
            (3, 0, 1, math.pi, 0, 2, 0),
            (4, 0, 0, -math.pi / 2, 0, 2, 0),
        ]
    )
    report = _report(data, circular=True)

    plot = build_trajectory_plot(data, report)

    assert report.metrics.duplicate_endpoint
    assert plot.point_count == 5
    assert plot.normalized_point_count == 4
    assert len(plot.xy.points) == 4
    assert len(plot.spacing.samples) == 4
    seam = plot.spacing.samples[-1]
    assert seam.segment_index == 3
    assert seam.point_index == 0
    assert seam.s_m == 4.0
    assert seam.value == 1.0
    assert nearest_index_by_s(plot, 3.9) == 0


def test_pure_pursuit_marks_missing_profiles_unavailable() -> None:
    data = _pure_pursuit_data(
        [
            (0, 0, 0.0, 2.0),
            (3, 0, math.pi / 2, 3.0),
            (3, 4, math.pi, 4.0),
        ]
    )
    report = _report(data, circular=False)

    plot = build_trajectory_plot(data, report)

    assert plot.s_axis_m == (0.0, 3.0, 7.0)
    assert tuple(sample.value for sample in plot.psi.samples) == pytest.approx(
        (0.0, math.pi / 2, math.pi)
    )
    assert tuple(sample.value for sample in plot.velocity.samples) == (
        2.0,
        3.0,
        4.0,
    )
    assert plot.psi.provenance == "derived:normalized_quaternion"
    assert not plot.kappa.available
    assert not plot.acceleration.available
    assert not plot.lateral_acceleration.available
    assert plot.kappa.samples == ()

    comparison = build_comparison_plot(
        data, report, copy.deepcopy(data), report
    )
    assert comparison.summary.max_displacement_m == 0.0
    assert comparison.summary.series("velocity").candidate.maximum == 4.0
    assert comparison.summary.series("kappa").candidate.maximum is None


def test_zero_quaternion_and_large_zero_curvature_are_safe() -> None:
    pure = _pure_pursuit_data([(0, 0, 0, 2), (1, 0, 0, 2)])
    for column in ("x_quat", "y_quat", "z_quat", "w_quat"):
        pure.rows[0][column] = "0"
    pure_plot = build_trajectory_plot(
        pure, _report(pure, circular=False)
    )
    assert not pure_plot.psi.available
    assert "no direction" in (pure_plot.psi.unavailable_reason or "")

    mpc = _mpc_data(
        [
            (0, 0, 0, 0, 0, 1e200, 0),
            (1, 1, 0, 0, 0, 1e200, 0),
        ]
    )
    mpc_plot = build_trajectory_plot(mpc, _report(mpc, circular=False))
    assert tuple(
        sample.value for sample in mpc_plot.lateral_acceleration.samples
    ) == (0.0, 0.0)


def test_plot_rejects_invalid_or_stale_reports_without_mutation() -> None:
    data = _mpc_data(
        [
            (0, 0, 0, 0, 0, 2, 0),
            (1, 1, 0, 0, 0, 2, 0),
        ]
    )
    invalid = copy.deepcopy(data)
    invalid.rows[1]["vx_mps"] = "nan"
    invalid_report = _report(invalid, circular=False)
    before = copy.deepcopy(invalid)

    with pytest.raises(PlotDataError, match="successful validation"):
        build_trajectory_plot(invalid, invalid_report)
    assert invalid == before

    valid_report = _report(data, circular=False)
    stale = copy.deepcopy(data)
    stale.rows.append(dict(stale.rows[-1]))
    stale.points.append((2.0, 0.0))
    with pytest.raises(PlotDataError, match="point count is stale"):
        build_trajectory_plot(stale, valid_report)


def test_repairable_geometry_errors_require_explicit_before_opt_in() -> None:
    before = _mpc_data(
        [
            (0, 0, 0, 0, 0, 2, 0),
            (1, 0, 0, 0, 0, 2, 0),
            (2, 1, 0, 0, 0, 2, 0),
        ]
    )
    candidate = _mpc_data(
        [
            (0, 0, 0, 0, 0, 2, 0),
            (1, 1, 0, 0, 0, 2, 0),
        ]
    )
    before_report = _report(before, circular=False)
    candidate_report = _report(candidate, circular=False)
    assert not before_report.is_valid

    with pytest.raises(PlotDataError, match="successful validation"):
        build_comparison_plot(
            before,
            before_report,
            candidate,
            candidate_report,
        )

    comparison = build_comparison_plot(
        before,
        before_report,
        candidate,
        candidate_report,
        allow_repairable_before=True,
    )
    assert comparison.summary.before_point_count == 3
    assert comparison.summary.candidate_point_count == 2
    assert tuple(
        sample.value for sample in comparison.before.spacing.samples
    ) == (0.0, 1.0)


def test_comparison_rejects_format_or_topology_mismatch() -> None:
    mpc = _mpc_data(
        [
            (0, 0, 0, 0, 0, 2, 0),
            (1, 1, 0, 0, 0, 2, 0),
            (2, 0, 1, 0, 0, 2, 0),
        ]
    )
    pure = _pure_pursuit_data([(0, 0, 0, 2), (1, 0, 0, 2)])

    with pytest.raises(PlotDataError, match="formats differ"):
        build_comparison_plot(
            mpc,
            _report(mpc, circular=False),
            pure,
            _report(pure, circular=False),
        )
    with pytest.raises(PlotDataError, match="topology differs"):
        build_comparison_plot(
            mpc,
            _report(mpc, circular=False),
            copy.deepcopy(mpc),
            _report(mpc, circular=True),
        )
