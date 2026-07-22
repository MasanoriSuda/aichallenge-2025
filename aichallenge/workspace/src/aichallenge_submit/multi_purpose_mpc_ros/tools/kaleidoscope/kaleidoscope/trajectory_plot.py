"""GUI-independent trajectory plot and before/after comparison models.

The Tk editor can render these immutable models with a Canvas without pulling
in matplotlib or NumPy.  Plot construction intentionally requires a matching,
successful :class:`ValidationReport`: malformed trajectory rows must not be
silently skipped just to make a preview look complete.
"""

from __future__ import annotations

from bisect import bisect_right
from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple

from .trajectory_contract import TrajectoryData
from .trajectory_contract import TrajectoryMetrics
from .trajectory_contract import ValidationReport


SCALAR_SERIES_KEYS: Tuple[str, ...] = (
    "spacing",
    "psi",
    "kappa",
    "velocity",
    "acceleration",
    "lateral_acceleration",
)

# These geometry/topology errors contain fully parsed finite rows and are the
# exact defects Normalize Geometry can repair.  They may be rendered only as
# an explicitly opted-in *before* view; candidates still require zero errors.
REPAIRABLE_PLOT_ERROR_CODES = frozenset(
    {
        "NON_INCREASING_S",
        "DUPLICATE_POINT",
        "DEGENERATE_SEGMENT",
        "DUPLICATE_CLOSING_POINT",
        "DEGENERATE_CLOSING_SEGMENT",
        "DEGENERATE_GEOMETRY_DERIVATIVE",
        "NONFINITE_GEOMETRY_DERIVATIVE",
    }
)


class PlotDataError(ValueError):
    """Raised when plot input is invalid or does not match its validation."""


@dataclass(frozen=True)
class XYPlotPoint:
    """One selectable trajectory point in the XY overlay."""

    point_index: int
    s_m: float
    x_m: float
    y_m: float


@dataclass(frozen=True)
class XYPlotSeries:
    key: str
    label: str
    x_label: str
    x_unit: str
    y_label: str
    y_unit: str
    points: Tuple[XYPlotPoint, ...]


@dataclass(frozen=True)
class ScalarPlotSample:
    """One scalar sample, optionally associated with a path segment."""

    point_index: int
    s_m: float
    value: float
    segment_index: Optional[int] = None


@dataclass(frozen=True)
class ScalarPlotSeries:
    key: str
    label: str
    x_label: str
    x_unit: str
    y_label: str
    y_unit: str
    provenance: str
    samples: Tuple[ScalarPlotSample, ...]
    unavailable_reason: Optional[str] = None

    @property
    def available(self) -> bool:
        return self.unavailable_reason is None


@dataclass(frozen=True)
class SeriesExtrema:
    minimum: Optional[float]
    maximum: Optional[float]
    maximum_absolute: Optional[float]


@dataclass(frozen=True)
class SeriesComparison:
    key: str
    label: str
    unit: str
    before: SeriesExtrema
    candidate: SeriesExtrema


@dataclass(frozen=True)
class TrajectoryPlotData:
    """All renderable data for one validated trajectory state."""

    role: str
    label: str
    format_name: str
    circular: bool
    point_count: int
    normalized_point_count: int
    path_length_m: float
    metrics: TrajectoryMetrics
    xy: XYPlotSeries
    spacing: ScalarPlotSeries
    psi: ScalarPlotSeries
    kappa: ScalarPlotSeries
    velocity: ScalarPlotSeries
    acceleration: ScalarPlotSeries
    lateral_acceleration: ScalarPlotSeries

    def scalar_series(self, key: str) -> ScalarPlotSeries:
        if key not in SCALAR_SERIES_KEYS:
            raise KeyError(f"unknown trajectory plot series: {key}")
        return getattr(self, key)

    @property
    def s_axis_m(self) -> Tuple[float, ...]:
        return tuple(point.s_m for point in self.xy.points)


@dataclass(frozen=True)
class SelectionMapping:
    """Synchronized selection for a before/candidate preview."""

    source_role: str
    source_index: int
    source_s_m: float
    before_index: int
    before_s_m: float
    candidate_index: int
    candidate_s_m: float


@dataclass(frozen=True)
class ComparisonSummary:
    before_point_count: int
    candidate_point_count: int
    point_count_delta: int
    before_normalized_point_count: int
    candidate_normalized_point_count: int
    before_path_length_m: float
    candidate_path_length_m: float
    path_length_delta_m: float
    max_displacement_m: float
    extrema: Tuple[SeriesComparison, ...]

    def series(self, key: str) -> SeriesComparison:
        for item in self.extrema:
            if item.key == key:
                return item
        raise KeyError(f"unknown trajectory plot series: {key}")


@dataclass(frozen=True)
class ComparisonPlotData:
    before: TrajectoryPlotData
    candidate: TrajectoryPlotData
    legend: Tuple[Tuple[str, str], ...]
    summary: ComparisonSummary

    def selection_from_before(self, point_index: int) -> SelectionMapping:
        return map_selection(self, "before", point_index)

    def selection_from_candidate(self, point_index: int) -> SelectionMapping:
        return map_selection(self, "candidate", point_index)


def _finite_float(value: object, *, column: str, index: int) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as error:
        raise PlotDataError(
            f"row {index + 2} column {column} is not numeric"
        ) from error
    if not math.isfinite(parsed):
        raise PlotDataError(f"row {index + 2} column {column} is not finite")
    return parsed


def _validate_plot_input(
    data: TrajectoryData,
    report: ValidationReport,
    *,
    allow_repairable_errors: bool = False,
) -> int:
    if not report.is_valid:
        error_codes = {
            issue.code
            for issue in report.issues
            if issue.severity.value == "error"
        }
        if (
            not allow_repairable_errors
            or not error_codes
            or not error_codes.issubset(REPAIRABLE_PLOT_ERROR_CODES)
        ):
            raise PlotDataError(
                "plot data requires a successful validation report"
            )
    if data.format_name not in ("mpc", "pure_pursuit"):
        raise PlotDataError(
            f"unsupported trajectory format: {data.format_name}"
        )
    if report.format_name != data.format_name:
        raise PlotDataError(
            "validation report format does not match trajectory data"
        )
    if len(data.rows) != len(data.points):
        raise PlotDataError("trajectory row and point counts differ")
    if report.metrics.point_count != len(data.rows):
        raise PlotDataError("validation report point count is stale")

    count = report.metrics.normalized_point_count
    if count < 2 or count > len(data.points):
        raise PlotDataError(
            "validation report normalized point count is invalid"
        )
    if report.metrics.total_distance_m is None:
        raise PlotDataError("validation report has no finite path length")
    return count


def _axis_from_data(data: TrajectoryData, count: int) -> Tuple[float, ...]:
    if data.format_name == "mpc":
        return tuple(
            _finite_float(
                data.rows[index].get("s_m"), column="s_m", index=index
            )
            for index in range(count)
        )

    values = [0.0]
    for index in range(1, count):
        previous = data.points[index - 1]
        current = data.points[index]
        spacing = math.hypot(
            current[0] - previous[0],
            current[1] - previous[1],
        )
        if not math.isfinite(spacing):
            raise PlotDataError(f"segment {index - 1} length is not finite")
        values.append(values[-1] + spacing)
    return tuple(values)


def _xy_series(
    data: TrajectoryData,
    axis: Sequence[float],
    count: int,
    label: str,
) -> XYPlotSeries:
    points = []
    for index in range(count):
        x_m, y_m = data.points[index]
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            raise PlotDataError(f"trajectory point {index} is not finite")
        points.append(XYPlotPoint(index, axis[index], x_m, y_m))
    return XYPlotSeries(
        key="xy",
        label=label,
        x_label="X",
        x_unit="m",
        y_label="Y",
        y_unit="m",
        points=tuple(points),
    )


def _scalar_series(
    *,
    key: str,
    label: str,
    y_label: str,
    y_unit: str,
    provenance: str,
    samples: Sequence[ScalarPlotSample] = (),
    unavailable_reason: Optional[str] = None,
) -> ScalarPlotSeries:
    return ScalarPlotSeries(
        key=key,
        label=label,
        x_label="s",
        x_unit="m",
        y_label=y_label,
        y_unit=y_unit,
        provenance=provenance,
        samples=tuple(samples),
        unavailable_reason=unavailable_reason,
    )


def _point_samples(
    data: TrajectoryData,
    axis: Sequence[float],
    count: int,
    column: str,
) -> Tuple[ScalarPlotSample, ...]:
    return tuple(
        ScalarPlotSample(
            point_index=index,
            s_m=axis[index],
            value=_finite_float(
                data.rows[index].get(column), column=column, index=index
            ),
        )
        for index in range(count)
    )


def _spacing_samples(
    data: TrajectoryData,
    report: ValidationReport,
    axis: Sequence[float],
    count: int,
) -> Tuple[ScalarPlotSample, ...]:
    samples = []
    pairs = [(index, index + 1) for index in range(count - 1)]
    if report.circular:
        pairs.append((count - 1, 0))

    for segment_index, (start, end) in enumerate(pairs):
        spacing = math.hypot(
            data.points[end][0] - data.points[start][0],
            data.points[end][1] - data.points[start][1],
        )
        if not math.isfinite(spacing):
            raise PlotDataError(
                f"segment {segment_index} length is not finite"
            )
        sample_s = (
            axis[end]
            if end != 0
            else axis[-1] + spacing
        )
        samples.append(
            ScalarPlotSample(
                point_index=end,
                segment_index=segment_index,
                s_m=sample_s,
                value=spacing,
            )
        )
    return tuple(samples)


def _pure_pursuit_psi_samples(
    data: TrajectoryData,
    axis: Sequence[float],
    count: int,
) -> Tuple[Tuple[ScalarPlotSample, ...], Optional[str]]:
    samples = []
    for index in range(count):
        row = data.rows[index]
        quaternion = tuple(
            _finite_float(row.get(column), column=column, index=index)
            for column in ("x_quat", "y_quat", "z_quat", "w_quat")
        )
        norm = math.hypot(*quaternion)
        if norm == 0.0 or not math.isfinite(norm):
            return (
                (),
                f"Pure Pursuit quaternion at point {index} has no direction",
            )
        x, y, z, w = (value / norm for value in quaternion)
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        samples.append(ScalarPlotSample(index, axis[index], yaw))
    return tuple(samples), None


def build_trajectory_plot(
    data: TrajectoryData,
    report: ValidationReport,
    *,
    role: str = "working",
    label: str = "Working",
    allow_repairable_errors: bool = False,
) -> TrajectoryPlotData:
    """Build immutable series for one validated trajectory.

    MPC scalar values come directly from the seven-column CSV contract.  Pure
    Pursuit provides speed and quaternion heading, but not curvature or
    acceleration; those absent quantities stay explicitly unavailable rather
    than being presented as if they were stored metadata.
    """

    count = _validate_plot_input(
        data,
        report,
        allow_repairable_errors=allow_repairable_errors,
    )
    axis = _axis_from_data(data, count)
    xy = _xy_series(data, axis, count, label)
    spacing = _scalar_series(
        key="spacing",
        label=label,
        y_label="Waypoint spacing",
        y_unit="m",
        provenance="geometry",
        samples=_spacing_samples(data, report, axis, count),
    )

    if data.format_name == "mpc":
        psi = _scalar_series(
            key="psi",
            label=label,
            y_label="Heading",
            y_unit="rad",
            provenance="csv:psi_rad",
            samples=_point_samples(data, axis, count, "psi_rad"),
        )
        kappa_samples = _point_samples(data, axis, count, "kappa_radpm")
        velocity_samples = _point_samples(data, axis, count, "vx_mps")
        acceleration_samples = _point_samples(data, axis, count, "ax_mps2")
        kappa = _scalar_series(
            key="kappa",
            label=label,
            y_label="Curvature",
            y_unit="rad/m",
            provenance="csv:kappa_radpm",
            samples=kappa_samples,
        )
        velocity = _scalar_series(
            key="velocity",
            label=label,
            y_label="Velocity",
            y_unit="m/s",
            provenance="csv:vx_mps",
            samples=velocity_samples,
        )
        acceleration = _scalar_series(
            key="acceleration",
            label=label,
            y_label="Longitudinal acceleration",
            y_unit="m/s^2",
            provenance="csv:ax_mps2",
            samples=acceleration_samples,
        )
        lateral_samples = []
        for kappa_sample, velocity_sample in zip(
            kappa_samples, velocity_samples
        ):
            velocity_value = velocity_sample.value
            kappa_value = kappa_sample.value
            lateral_value = (
                0.0
                if velocity_value == 0.0 or kappa_value == 0.0
                else velocity_value * (velocity_value * kappa_value)
            )
            lateral_samples.append(
                ScalarPlotSample(
                    point_index=kappa_sample.point_index,
                    s_m=kappa_sample.s_m,
                    value=lateral_value,
                )
            )
        lateral_samples_tuple = tuple(lateral_samples)
        if not all(
            math.isfinite(sample.value) for sample in lateral_samples_tuple
        ):
            raise PlotDataError("derived lateral acceleration is not finite")
        lateral_acceleration = _scalar_series(
            key="lateral_acceleration",
            label=label,
            y_label="Lateral acceleration",
            y_unit="m/s^2",
            provenance="derived:vx_mps^2*kappa_radpm",
            samples=lateral_samples_tuple,
        )
    else:
        psi_samples, psi_reason = _pure_pursuit_psi_samples(
            data, axis, count
        )
        psi = _scalar_series(
            key="psi",
            label=label,
            y_label="Heading",
            y_unit="rad",
            provenance="derived:normalized_quaternion",
            samples=psi_samples,
            unavailable_reason=psi_reason,
        )
        velocity = _scalar_series(
            key="velocity",
            label=label,
            y_label="Velocity",
            y_unit="m/s",
            provenance="csv:speed",
            samples=_point_samples(data, axis, count, "speed"),
        )
        kappa = _scalar_series(
            key="kappa",
            label=label,
            y_label="Curvature",
            y_unit="rad/m",
            provenance="unavailable",
            unavailable_reason="Pure Pursuit CSV has no curvature column",
        )
        acceleration = _scalar_series(
            key="acceleration",
            label=label,
            y_label="Longitudinal acceleration",
            y_unit="m/s^2",
            provenance="unavailable",
            unavailable_reason="Pure Pursuit CSV has no acceleration column",
        )
        lateral_acceleration = _scalar_series(
            key="lateral_acceleration",
            label=label,
            y_label="Lateral acceleration",
            y_unit="m/s^2",
            provenance="unavailable",
            unavailable_reason=(
                "Pure Pursuit CSV has no curvature/acceleration profile"
            ),
        )

    path_length = report.metrics.total_distance_m
    assert path_length is not None
    return TrajectoryPlotData(
        role=role,
        label=label,
        format_name=data.format_name,
        circular=report.circular,
        point_count=report.metrics.point_count,
        normalized_point_count=count,
        path_length_m=path_length,
        metrics=report.metrics,
        xy=xy,
        spacing=spacing,
        psi=psi,
        kappa=kappa,
        velocity=velocity,
        acceleration=acceleration,
        lateral_acceleration=lateral_acceleration,
    )


def _series_extrema(series: ScalarPlotSeries) -> SeriesExtrema:
    if not series.available or not series.samples:
        return SeriesExtrema(None, None, None)
    values = tuple(sample.value for sample in series.samples)
    return SeriesExtrema(
        minimum=min(values),
        maximum=max(values),
        maximum_absolute=max(abs(value) for value in values),
    )


def nearest_index_by_s(plot: TrajectoryPlotData, s_m: float) -> int:
    """Return the deterministic nearest point index for an ``s_m`` cursor.

    Circular plots use periodic distance, so a cursor near total path length
    maps to the first point across the seam.  Equal-distance ties select the
    lower point index.
    """

    if not math.isfinite(s_m):
        raise ValueError("selection s_m must be finite")
    if not plot.xy.points:
        raise PlotDataError("cannot select from an empty plot")

    origin = plot.xy.points[0].s_m
    period = (
        plot.spacing.samples[-1].s_m - origin
        if plot.circular and plot.spacing.samples
        else plot.path_length_m
    )

    def distance(point: XYPlotPoint) -> float:
        direct = abs(point.s_m - s_m)
        if not plot.circular or period <= 0.0:
            return direct
        source_offset = (s_m - origin) % period
        point_offset = (point.s_m - origin) % period
        periodic = abs(source_offset - point_offset)
        return min(periodic, period - periodic)

    return min(
        range(len(plot.xy.points)),
        key=lambda index: (distance(plot.xy.points[index]), index),
    )


def _mapped_s(
    source: TrajectoryPlotData,
    target: TrajectoryPlotData,
    s_m: float,
) -> float:
    """Move an absolute source s into the target's coordinate origin."""

    source_origin = source.xy.points[0].s_m
    target_origin = target.xy.points[0].s_m
    return target_origin + (s_m - source_origin)


def map_selection(
    comparison: ComparisonPlotData,
    source_role: str,
    point_index: int,
) -> SelectionMapping:
    """Map a selected point to the other dataset using nearest ``s_m``."""

    if source_role == "before":
        source = comparison.before
        target = comparison.candidate
    elif source_role == "candidate":
        source = comparison.candidate
        target = comparison.before
    else:
        raise ValueError("source_role must be 'before' or 'candidate'")
    if point_index < 0 or point_index >= len(source.xy.points):
        raise IndexError(
            "selected point index is outside the source trajectory"
        )

    source_point = source.xy.points[point_index]
    target_s = _mapped_s(source, target, source_point.s_m)
    target_index = nearest_index_by_s(target, target_s)
    if source_role == "before":
        before_index = point_index
        candidate_index = target_index
    else:
        before_index = target_index
        candidate_index = point_index

    before_point = comparison.before.xy.points[before_index]
    candidate_point = comparison.candidate.xy.points[candidate_index]
    return SelectionMapping(
        source_role=source_role,
        source_index=point_index,
        source_s_m=source_point.s_m,
        before_index=before_index,
        before_s_m=before_point.s_m,
        candidate_index=candidate_index,
        candidate_s_m=candidate_point.s_m,
    )


def _max_mapped_displacement(
    first: TrajectoryPlotData,
    second: TrajectoryPlotData,
) -> float:
    maximum = 0.0
    second_starts = tuple(point.s_m for point in second.xy.points)
    for point in first.xy.points:
        target_s = _mapped_s(first, second, point.s_m)
        target_x, target_y = _interpolated_xy(
            second,
            target_s,
            starts=second_starts,
        )
        maximum = max(
            maximum,
            math.hypot(point.x_m - target_x, point.y_m - target_y),
        )
    return maximum


def _interpolated_xy(
    plot: TrajectoryPlotData,
    s_m: float,
    *,
    starts: Optional[Sequence[float]] = None,
) -> Tuple[float, float]:
    """Interpolate XY at an arc coordinate without waypoint-density bias."""

    points = plot.xy.points
    if len(points) == 1:
        return points[0].x_m, points[0].y_m
    if starts is None:
        starts = tuple(point.s_m for point in points)
    origin = starts[0]

    if plot.circular:
        period = (
            plot.spacing.samples[-1].s_m - origin
            if plot.spacing.samples
            else plot.path_length_m
        )
        if period <= 0.0:
            raise PlotDataError("circular plot has no positive interpolation period")
        query = origin + ((s_m - origin) % period)
        segment = max(0, bisect_right(starts, query) - 1)
        if segment >= len(points) - 1:
            following = 0
            segment_end = origin + period
        else:
            following = segment + 1
            segment_end = starts[following]
    else:
        if s_m <= starts[0]:
            return points[0].x_m, points[0].y_m
        if s_m >= starts[-1]:
            return points[-1].x_m, points[-1].y_m
        query = s_m
        segment = max(0, bisect_right(starts, query) - 1)
        following = segment + 1
        segment_end = starts[following]

    segment_start = starts[segment]
    length = segment_end - segment_start
    if length <= 0.0:
        # Repair previews can contain non-increasing source s. Deterministically
        # fall back to the segment start instead of inventing a displacement.
        return points[segment].x_m, points[segment].y_m
    ratio = (query - segment_start) / length
    first = points[segment]
    second = points[following]
    return (
        first.x_m + ratio * (second.x_m - first.x_m),
        first.y_m + ratio * (second.y_m - first.y_m),
    )


def build_comparison_plot(
    before_data: TrajectoryData,
    before_report: ValidationReport,
    candidate_data: TrajectoryData,
    candidate_report: ValidationReport,
    *,
    allow_repairable_before: bool = False,
    allow_candidate_errors: bool = False,
) -> ComparisonPlotData:
    """Build before/candidate overlays, selection mapping, and summary."""

    before = build_trajectory_plot(
        before_data,
        before_report,
        role="before",
        label="Before",
        allow_repairable_errors=allow_repairable_before,
    )
    candidate = build_trajectory_plot(
        candidate_data,
        candidate_report,
        role="candidate",
        label="Candidate",
        allow_repairable_errors=allow_candidate_errors,
    )
    if before.format_name != candidate.format_name:
        raise PlotDataError("before and candidate trajectory formats differ")
    if before.circular != candidate.circular:
        raise PlotDataError("before and candidate topology differs")

    extrema = tuple(
        SeriesComparison(
            key=key,
            label=before.scalar_series(key).y_label,
            unit=before.scalar_series(key).y_unit,
            before=_series_extrema(before.scalar_series(key)),
            candidate=_series_extrema(candidate.scalar_series(key)),
        )
        for key in SCALAR_SERIES_KEYS
    )
    maximum_displacement = max(
        _max_mapped_displacement(before, candidate),
        _max_mapped_displacement(candidate, before),
    )
    summary = ComparisonSummary(
        before_point_count=before.point_count,
        candidate_point_count=candidate.point_count,
        point_count_delta=candidate.point_count - before.point_count,
        before_normalized_point_count=before.normalized_point_count,
        candidate_normalized_point_count=candidate.normalized_point_count,
        before_path_length_m=before.path_length_m,
        candidate_path_length_m=candidate.path_length_m,
        path_length_delta_m=candidate.path_length_m - before.path_length_m,
        max_displacement_m=maximum_displacement,
        extrema=extrema,
    )
    return ComparisonPlotData(
        before=before,
        candidate=candidate,
        legend=(("before", "Before"), ("candidate", "Candidate")),
        summary=summary,
    )
