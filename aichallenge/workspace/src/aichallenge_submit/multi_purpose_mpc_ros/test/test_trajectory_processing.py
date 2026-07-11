import copy
import math
from pathlib import Path

import pytest

from multi_purpose_mpc_ros.tools import trajectory_contract as contract
from multi_purpose_mpc_ros.tools import trajectory_processing as processing


def _row(
    s: float,
    x: float,
    y: float,
    *,
    velocity: str = "2.500",
    acceleration: str = "0.1250",
) -> dict[str, str]:
    return {
        "s_m": str(s),
        "x_m": str(x),
        "y_m": str(y),
        "psi_rad": "99",
        "kappa_radpm": "99",
        "vx_mps": velocity,
        "ax_mps2": acceleration,
    }


def _data(
    points: list[contract.Point],
    rows: list[dict[str, str]] | None = None,
) -> contract.TrajectoryData:
    if rows is None:
        rows = [
            _row(float(index), *point)
            for index, point in enumerate(points)
        ]
    return contract.TrajectoryData(
        path=Path("source.csv"),
        fieldnames=list(contract.MPC_COLUMNS),
        rows=rows,
        points=list(points),
        x_column="x_m",
        y_column="y_m",
        format_name="mpc",
    )


def _options(
    *,
    circular: bool,
    metadata_mode: str = "interpolate",
    resample: bool = True,
    resolution_m: float = 0.25,
    remove_closure_duplicate: bool = True,
    remove_degenerate_points: bool = True,
) -> processing.NormalizeOptions:
    return processing.NormalizeOptions(
        circular=circular,
        metadata_mode=metadata_mode,
        resample=resample,
        resolution_m=resolution_m,
        remove_closure_duplicate=remove_closure_duplicate,
        remove_degenerate_points=remove_degenerate_points,
    )


def test_open_line_resamples_uniformly_and_regenerates_geometry() -> None:
    source = _data([(0.0, 0.0), (1.0, 0.0)])

    result = processing.normalize_geometry(
        source, _options(circular=False, resolution_m=0.3), source_revision=7
    )

    assert result.source_revision == 7
    assert result.operation == "normalize_geometry"
    assert result.validation.is_valid
    assert result.transformation.output_point_count == 5
    assert result.dataset.points[0] == (0.0, 0.0)
    assert result.dataset.points[-1] == (1.0, 0.0)
    assert result.transformation.output_max_spacing_m <= 0.3 * (1.0 + 1e-12)
    assert [float(row["s_m"]) for row in result.dataset.rows] == pytest.approx(
        [0.0, 0.25, 0.5, 0.75, 1.0]
    )
    assert [
        float(row["psi_rad"]) for row in result.dataset.rows
    ] == pytest.approx([0.0] * 5)
    assert [
        float(row["kappa_radpm"]) for row in result.dataset.rows
    ] == pytest.approx([0.0] * 5, abs=1e-12)


def test_circle_is_unique_periodic_and_has_expected_curvature() -> None:
    radius = 5.0
    count = 128
    points = [
        (
            radius * math.cos(2.0 * math.pi * index / count),
            radius * math.sin(2.0 * math.pi * index / count),
        )
        for index in range(count)
    ]
    result = processing.normalize_geometry(
        _data(points),
        _options(circular=True, resolution_m=0.2),
        source_revision=3,
    )

    assert result.validation.is_valid
    assert result.dataset.points[-1] != result.dataset.points[0]
    assert result.transformation.output_closing_spacing_m is not None
    assert (
        result.transformation.output_closing_spacing_m
        <= 0.2 * (1.0 + 1e-12)
    )
    assert result.transformation.output_max_spacing_m <= 0.2 * (1.0 + 1e-12)
    kappas = [float(row["kappa_radpm"]) for row in result.dataset.rows]
    assert sum(kappas) / len(kappas) == pytest.approx(1.0 / radius, rel=0.08)
    assert max(abs(value - 1.0 / radius) for value in kappas) < 0.08


def test_heading_pi_seam_is_compared_as_wrapped_angle() -> None:
    count = 96
    points = [
        (
            math.cos(2.0 * math.pi * index / count),
            math.sin(2.0 * math.pi * index / count),
        )
        for index in range(count)
    ]
    result = processing.normalize_geometry(
        _data(points),
        _options(circular=True, resample=False),
        source_revision=0,
    )
    headings = [float(row["psi_rad"]) for row in result.dataset.rows]

    assert min(headings) < -3.0
    assert max(headings) > 3.0
    maximum_heading_change = (
        result.validation.metrics.max_abs_psi_difference_rad
    )
    assert maximum_heading_change == pytest.approx(
        2.0 * math.pi / count, rel=0.02
    )


def test_periodic_metadata_interpolates_across_closing_seam() -> None:
    points = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
    rows = [
        _row(0, *points[0], velocity="0", acceleration="0"),
        _row(2, *points[1], velocity="2", acceleration="1"),
        _row(4, *points[2], velocity="4", acceleration="2"),
        _row(6, *points[3], velocity="6", acceleration="3"),
    ]
    result = processing.normalize_geometry(
        _data(points, rows),
        _options(circular=True, resolution_m=1.0),
        source_revision=1,
    )

    velocities = [float(row["vx_mps"]) for row in result.dataset.rows]
    assert velocities == pytest.approx([0, 1, 2, 3, 4, 5, 6, 3])
    assert result.dataset.points[-1] == pytest.approx((0.0, 1.0))


def test_preserve_keeps_metadata_text_when_point_count_is_unchanged() -> None:
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    rows = [
        _row(0, *points[0], velocity="2.500", acceleration="0.1250"),
        _row(1, *points[1], velocity="3.00", acceleration="-0.00"),
        _row(2, *points[2], velocity="4e0", acceleration="1.0e-1"),
    ]
    result = processing.normalize_geometry(
        _data(points, rows),
        _options(circular=False, metadata_mode="preserve", resample=False),
        source_revision=11,
    )

    assert [row["vx_mps"] for row in result.dataset.rows] == [
        "2.500",
        "3.00",
        "4e0",
    ]
    assert [row["ax_mps2"] for row in result.dataset.rows] == [
        "0.1250",
        "-0.00",
        "1.0e-1",
    ]


def test_preserve_rejects_cleanup_or_resampling_that_changes_count() -> None:
    duplicate = _data([(0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    with pytest.raises(ValueError, match="unchanged topology and point count"):
        processing.normalize_geometry(
            duplicate,
            _options(circular=False, metadata_mode="preserve", resample=False),
            source_revision=0,
        )

    line = _data([(0.0, 0.0), (1.0, 0.0)])
    with pytest.raises(ValueError, match="unchanged topology and point count"):
        processing.normalize_geometry(
            line,
            _options(
                circular=False,
                metadata_mode="preserve",
                resolution_m=0.25,
            ),
            source_revision=0,
        )


def test_cleanup_removes_endpoint_and_internal_degenerate_record() -> None:
    points = [
        (0.0, 0.0),
        (1.0, 0.0),
        (1.0 + contract.MIN_SEGMENT_LENGTH_M, 0.0),
        (1.0, 1.0),
        (0.0005, 0.0),
    ]
    result = processing.normalize_geometry(
        _data(points),
        _options(circular=True, resample=False),
        source_revision=2,
    )

    assert result.validation.is_valid
    assert result.transformation.removed_closure_indices == (4,)
    assert result.transformation.removed_degenerate_indices == (2,)
    assert result.transformation.retained_source_indices == (0, 1, 3)
    assert result.transformation.cleaned_point_count == 3
    assert len(result.dataset.rows) == 3
    assert result.dataset.points[-1] != result.dataset.points[0]


def test_cleanup_can_be_explicitly_refused() -> None:
    circular_duplicate = _data(
        [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (0.0005, 0.0)]
    )
    with pytest.raises(ValueError, match="enable remove_closure_duplicate"):
        processing.normalize_geometry(
            circular_duplicate,
            _options(circular=True, remove_closure_duplicate=False),
            source_revision=0,
        )

    internal_duplicate = _data(
        [(0.0, 0.0), (1.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    )
    with pytest.raises(ValueError, match="enable remove_degenerate_points"):
        processing.normalize_geometry(
            internal_duplicate,
            _options(circular=False, remove_degenerate_points=False),
            source_revision=0,
        )


def test_generation_is_non_mutating_and_deterministic() -> None:
    source = _data([(0.0, 0.0), (1.0, 0.5), (2.0, 0.0)])
    before = copy.deepcopy(source)
    options = _options(circular=False, resolution_m=0.2)

    first = processing.normalize_geometry(source, options, source_revision=9)
    second = processing.normalize_geometry(source, options, source_revision=9)

    assert source == before
    assert first.dataset == second.dataset
    assert first.validation == second.validation
    assert first.transformation == second.transformation
    assert dict(first.parameters) == dict(second.parameters)


@pytest.mark.parametrize("resolution", [0.0, -1.0, math.inf, math.nan])
def test_invalid_resolution_is_rejected(resolution: float) -> None:
    with pytest.raises(ValueError, match="resolution_m"):
        processing.normalize_geometry(
            _data([(0.0, 0.0), (1.0, 0.0)]),
            _options(circular=False, resolution_m=resolution),
            source_revision=0,
        )


def test_invalid_mode_and_revision_are_rejected() -> None:
    source = _data([(0.0, 0.0), (1.0, 0.0)])
    with pytest.raises(ValueError, match="metadata_mode"):
        processing.normalize_geometry(
            source,
            _options(circular=False, metadata_mode="copy-nearest"),
            source_revision=0,
        )
    with pytest.raises(ValueError, match="source_revision"):
        processing.normalize_geometry(
            source, _options(circular=False), source_revision=-1
        )


def test_noncanonical_source_number_is_not_silently_repaired() -> None:
    source = _data([(0.0, 0.0), (1.0, 0.0)])
    source.rows[0]["vx_mps"] = "1_0"

    with pytest.raises(ValueError, match="canonical finite numbers"):
        processing.normalize_geometry(
            source, _options(circular=False), source_revision=0
        )


def test_regenerated_source_columns_may_be_repaired() -> None:
    source = _data([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    source.rows[0]["s_m"] = "nan"
    source.rows[1]["psi_rad"] = "not-a-number"
    source.rows[2]["kappa_radpm"] = "inf"

    result = processing.normalize_geometry(
        source,
        _options(circular=False, resample=False),
        source_revision=0,
    )

    assert result.validation.is_valid
    assert [float(row["s_m"]) for row in result.dataset.rows] == [0.0, 1.0, 2.0]
    assert all(
        math.isfinite(float(row[column]))
        for row in result.dataset.rows
        for column in ("psi_rad", "kappa_radpm")
    )


def test_recompute_metadata_mode_marks_explicit_deferred_policy() -> None:
    source = _data([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    result = processing.normalize_geometry(
        source,
        _options(
            circular=False,
            metadata_mode="recompute",
            resolution_m=0.4,
        ),
        source_revision=0,
    )

    assert result.validation.is_valid
    assert result.transformation.metadata_mode is processing.MetadataMode.RECOMPUTE
    assert result.parameters["metadata_mode"] == "recompute"


def test_circular_spacing_must_exceed_duplicate_endpoint_tolerance() -> None:
    source = _data(
        [(0.0, 0.0), (0.04, 0.0), (0.04, 0.04), (0.0, 0.04)]
    )
    with pytest.raises(ValueError, match="duplicate endpoint"):
        processing.normalize_geometry(
            source,
            _options(circular=True, resolution_m=0.0009),
            source_revision=0,
        )


def test_generated_circular_chord_must_exceed_closure_tolerance() -> None:
    source = _data(
        [
            (0.0, 0.0),
            (0.02, 0.0),
            (-0.01, 0.005),
            (0.00105, 0.0),
        ]
    )
    with pytest.raises(ValueError, match="closing chord.*duplicate endpoint"):
        processing.normalize_geometry(
            source,
            _options(circular=True, resolution_m=0.0012),
            source_revision=0,
        )


def test_tiny_resolution_is_rejected_before_allocation() -> None:
    source = _data([(0.0, 0.0), (1.0, 0.0)])
    with pytest.raises(ValueError, match="resolution_m is too small"):
        processing.normalize_geometry(
            source,
            _options(circular=False, resolution_m=math.nextafter(0.0, 1.0)),
            source_revision=0,
        )


def test_pure_pursuit_has_clear_unsupported_error() -> None:
    source = contract.TrajectoryData(
        path=Path("pure.csv"),
        fieldnames=list(contract.PURE_PURSUIT_COLUMNS),
        rows=[],
        points=[(0.0, 0.0), (1.0, 0.0)],
        x_column="x",
        y_column="y",
        format_name="pure_pursuit",
    )
    with pytest.raises(
        ValueError, match="MPC trajectories only.*Pure Pursuit"
    ):
        processing.normalize_geometry(
            source, _options(circular=False), source_revision=0
        )
