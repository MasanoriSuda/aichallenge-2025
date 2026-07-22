"""Pure trajectory candidate generation for the offline trajectory editor.

The functions in this module never write files and never mutate their input
``TrajectoryData``.  Geometry normalization is deliberately limited to the
strict seven-column MPC format; the Pure Pursuit editor keeps its existing,
format-specific editing path.
"""

from __future__ import annotations

from bisect import bisect_right
import copy
from dataclasses import dataclass
from enum import Enum
import math
from types import MappingProxyType
from typing import Mapping, Optional, Sequence, Tuple

from .trajectory_contract import CLOSURE_TOLERANCE_M
from .trajectory_contract import (
    MIN_SEGMENT_LENGTH_M,
)
from .trajectory_contract import MPC_COLUMNS
from .trajectory_contract import Point
from .trajectory_contract import Severity
from .trajectory_contract import TrajectoryData
from .trajectory_contract import ValidationIssue
from .trajectory_contract import ValidationReport
from .trajectory_contract import validate_trajectory


_DERIVATIVE_EPSILON_SQUARED = 1e-24
_MAX_OUTPUT_POINTS = 50_000


class MetadataMode(str, Enum):
    """How velocity and acceleration metadata follows normalized geometry."""

    PRESERVE = "preserve"
    INTERPOLATE = "interpolate"
    RECOMPUTE = "recompute"


@dataclass(frozen=True)
class NormalizeOptions:
    """Explicit options for one geometry-normalization candidate."""

    circular: bool
    metadata_mode: MetadataMode | str
    remove_closure_duplicate: bool = True
    remove_degenerate_points: bool = True
    resample: bool = True
    resolution_m: float = 0.25


@dataclass(frozen=True)
class TransformationReport:
    """Deterministic before/after facts for a normalization preview."""

    input_point_count: int
    cleaned_point_count: int
    output_point_count: int
    retained_source_indices: Tuple[int, ...]
    removed_closure_indices: Tuple[int, ...]
    removed_degenerate_indices: Tuple[int, ...]
    circular: bool
    resampled: bool
    requested_resolution_m: Optional[float]
    metadata_mode: MetadataMode
    input_path_length_m: float
    cleaned_path_length_m: float
    output_path_length_m: float
    output_min_spacing_m: Optional[float]
    output_max_spacing_m: Optional[float]
    output_mean_spacing_m: Optional[float]
    output_closing_spacing_m: Optional[float]

    @property
    def removed_point_count(self) -> int:
        return len(self.removed_closure_indices) + len(
            self.removed_degenerate_indices
        )


@dataclass(frozen=True)
class CandidateResult:
    """A detached normalization candidate tied to its source revision."""

    source_revision: int
    operation: str
    parameters: Mapping[str, object]
    dataset: TrajectoryData
    validation: ValidationReport
    transformation: TransformationReport


def _distance(first: Point, second: Point) -> float:
    return math.hypot(second[0] - first[0], second[1] - first[1])


def _format_number(value: float) -> str:
    if not math.isfinite(value):
        raise ValueError("generated trajectory values must be finite")
    if value == 0.0:
        return "0"
    return format(value, ".17g")


def _coerce_metadata_mode(value: MetadataMode | str) -> MetadataMode:
    try:
        return MetadataMode(value)
    except (TypeError, ValueError) as error:
        supported = ", ".join(mode.value for mode in MetadataMode)
        raise ValueError(
            f"metadata_mode must be one of: {supported}"
        ) from error


def _validate_options(
    options: NormalizeOptions, source_revision: int
) -> MetadataMode:
    if not isinstance(options.circular, bool):
        raise ValueError("circular must be a boolean")
    for name in (
        "remove_closure_duplicate",
        "remove_degenerate_points",
        "resample",
    ):
        if not isinstance(getattr(options, name), bool):
            raise ValueError(f"{name} must be a boolean")
    if (
        not isinstance(options.resolution_m, (int, float))
        or isinstance(options.resolution_m, bool)
        or not math.isfinite(float(options.resolution_m))
        or float(options.resolution_m) <= 0.0
    ):
        raise ValueError("resolution_m must be finite and positive")
    if (
        not isinstance(source_revision, int)
        or isinstance(source_revision, bool)
        or source_revision < 0
    ):
        raise ValueError("source_revision must be a non-negative integer")
    return _coerce_metadata_mode(options.metadata_mode)


def _require_mpc_data(data: TrajectoryData) -> None:
    if data.format_name != "mpc":
        raise ValueError(
            "Normalize Geometry supports MPC trajectories only; "
            "Pure Pursuit normalization is not implemented"
        )
    if len(data.fieldnames) != len(MPC_COLUMNS) or set(data.fieldnames) != set(
        MPC_COLUMNS
    ):
        raise ValueError(
            "MPC normalization requires the strict seven-column schema"
        )
    if len(data.rows) != len(data.points):
        raise ValueError(
            "trajectory rows and points must have the same length"
        )
    if len(data.points) < 2:
        raise ValueError("trajectory requires at least two points")
    for point_index, point in enumerate(data.points):
        if len(point) != 2 or not all(math.isfinite(value) for value in point):
            raise ValueError(
                f"point {point_index} must contain finite x/y values"
            )
    for row_index, row in enumerate(data.rows):
        for column in MPC_COLUMNS:
            if column not in row:
                raise ValueError(f"row {row_index} is missing {column}")
        for column in ("vx_mps", "ax_mps2"):
            try:
                value = float(row[column])
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f"row {row_index} {column} must be a finite number"
                ) from error
            if not math.isfinite(value):
                raise ValueError(
                    f"row {row_index} {column} must be a finite number"
                )

    # Reuse the CSV contract's decimal parser for all seven fields.  Geometry
    # and s errors are repairable here, but malformed/non-finite source values
    # must not silently become valid merely because Python's float() is lax.
    input_report = validate_trajectory(
        data.fieldnames,
        data.rows,
        data.format_name,
        circular=False,
    )
    numeric_issues = [
        issue
        for issue in input_report.issues
        if issue.code == "EXTRA_ROW_FIELDS"
        or (
            issue.code == "INVALID_NUMBER"
            and issue.column in {"x_m", "y_m", "vx_mps", "ax_mps2"}
        )
    ]
    if numeric_issues:
        issue = numeric_issues[0]
        location = (
            f"line {issue.line_number}, column {issue.column}"
            if issue.column is not None
            else f"line {issue.line_number}"
        )
        raise ValueError(
            f"MPC normalization requires canonical finite numbers: "
            f"{location}: {issue.message}"
        )


def _path_length(points: Sequence[Point], circular: bool) -> float:
    length = sum(
        _distance(points[index - 1], points[index])
        for index in range(1, len(points))
    )
    if circular and len(points) >= 2:
        length += _distance(points[-1], points[0])
    if not math.isfinite(length):
        raise ValueError("trajectory path length must be finite")
    return length


def _canonical_arc(
    points: Sequence[Point], circular: bool
) -> Tuple[list[float], float]:
    positions = [0.0]
    for index in range(1, len(points)):
        spacing = _distance(points[index - 1], points[index])
        if spacing <= MIN_SEGMENT_LENGTH_M:
            raise ValueError(
                f"segment {index - 1} length must be more than "
                f"{MIN_SEGMENT_LENGTH_M:g} m"
            )
        positions.append(positions[-1] + spacing)
    total = positions[-1]
    if circular:
        closing_spacing = _distance(points[-1], points[0])
        if closing_spacing <= MIN_SEGMENT_LENGTH_M:
            raise ValueError(
                "circular closing segment length must be more than "
                f"{MIN_SEGMENT_LENGTH_M:g} m"
            )
        total += closing_spacing
    if not math.isfinite(total) or total <= 0.0:
        raise ValueError("trajectory path length must be finite and positive")
    return positions, total


def _clean_points(
    points: Sequence[Point], options: NormalizeOptions
) -> Tuple[list[Point], list[int], Tuple[int, ...], Tuple[int, ...]]:
    working_points = list(points)
    working_indices = list(range(len(points)))
    closure_removed: list[int] = []

    if options.circular:
        while (
            len(working_points) > 1
            and _distance(working_points[-1], working_points[0])
            <= CLOSURE_TOLERANCE_M
        ):
            if not options.remove_closure_duplicate:
                raise ValueError(
                    "circular input has a legacy duplicate endpoint; enable "
                    "remove_closure_duplicate to produce a unique endpoint"
                )
            closure_removed.append(working_indices.pop())
            working_points.pop()

    retained_points: list[Point] = []
    retained_indices: list[int] = []
    degenerate_removed: list[int] = []
    for point, original_index in zip(working_points, working_indices):
        if (
            retained_points
            and _distance(retained_points[-1], point) <= MIN_SEGMENT_LENGTH_M
        ):
            if not options.remove_degenerate_points:
                raise ValueError(
                    f"segment ending at point {original_index} is degenerate; "
                    "enable remove_degenerate_points"
                )
            degenerate_removed.append(original_index)
            continue
        retained_points.append(point)
        retained_indices.append(original_index)

    # The legacy closure pass normally handles this.  Re-check after internal
    # cleanup so unusual multiple-degenerate inputs cannot leave a zero seam.
    while (
        options.circular
        and len(retained_points) > 1
        and _distance(retained_points[-1], retained_points[0])
        <= MIN_SEGMENT_LENGTH_M
    ):
        if not options.remove_degenerate_points:
            raise ValueError(
                "circular closing segment is degenerate; enable "
                "remove_degenerate_points"
            )
        degenerate_removed.append(retained_indices.pop())
        retained_points.pop()

    minimum = 3 if options.circular else 2
    if len(retained_points) < minimum:
        raise ValueError(
            f"normalization leaves {len(retained_points)} points; "
            f"at least {minimum} are required"
        )
    return (
        retained_points,
        retained_indices,
        tuple(sorted(closure_removed)),
        tuple(sorted(degenerate_removed)),
    )


def _sample_scalar(
    values: Sequence[float],
    starts: Sequence[float],
    total: float,
    query: float,
    circular: bool,
) -> float:
    if not circular and query >= total:
        return values[-1]
    segment = min(len(starts) - 1, max(0, bisect_right(starts, query) - 1))
    next_index = segment + 1
    segment_end = total if next_index == len(starts) else starts[next_index]
    if next_index == len(starts):
        next_index = 0 if circular else segment
    length = segment_end - starts[segment]
    if length <= 0.0:
        raise ValueError("cannot interpolate a zero-length segment")
    ratio = (query - starts[segment]) / length
    return values[segment] + ratio * (values[next_index] - values[segment])


def _sample_point(
    points: Sequence[Point],
    starts: Sequence[float],
    total: float,
    query: float,
    circular: bool,
) -> Point:
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (
        _sample_scalar(xs, starts, total, query, circular),
        _sample_scalar(ys, starts, total, query, circular),
    )


def _resample_queries(
    total: float, options: NormalizeOptions
) -> Tuple[list[float], float]:
    resolution = float(options.resolution_m)
    estimated_segments = total / resolution
    if not math.isfinite(estimated_segments):
        raise ValueError(
            "resolution_m is too small for the trajectory path length"
        )
    if options.circular:
        segment_count = max(3, int(math.ceil(estimated_segments)))
        output_count = segment_count
    else:
        segment_count = max(1, int(math.ceil(estimated_segments)))
        output_count = segment_count + 1
    if output_count > _MAX_OUTPUT_POINTS:
        raise ValueError(
            f"normalization would create {output_count} points; maximum is "
            f"{_MAX_OUTPUT_POINTS}"
        )
    spacing = total / segment_count
    if options.circular and spacing <= CLOSURE_TOLERANCE_M:
        raise ValueError(
            "circular output spacing would be interpreted as a duplicate "
            f"endpoint (must be more than {CLOSURE_TOLERANCE_M:g} m)"
        )
    return [spacing * index for index in range(output_count)], spacing


def _linear_combination(
    first: Point,
    first_weight: float,
    second: Point,
    second_weight: float,
    third: Point,
    third_weight: float,
) -> Point:
    return (
        first_weight * first[0]
        + second_weight * second[0]
        + third_weight * third[0],
        first_weight * first[1]
        + second_weight * second[1]
        + third_weight * third[1],
    )


def _interior_derivatives(
    previous: Point,
    current: Point,
    following: Point,
    h_previous: float,
    h_next: float,
) -> Tuple[Point, Point]:
    total = h_previous + h_next
    first = _linear_combination(
        previous,
        -h_next / (h_previous * total),
        current,
        (h_next - h_previous) / (h_previous * h_next),
        following,
        h_previous / (h_next * total),
    )
    second = _linear_combination(
        previous,
        2.0 / (h_previous * total),
        current,
        -2.0 / (h_previous * h_next),
        following,
        2.0 / (h_next * total),
    )
    return first, second


def _forward_derivatives(
    first_point: Point,
    second_point: Point,
    third_point: Point,
    h_first: float,
    h_second: float,
) -> Tuple[Point, Point]:
    total = h_first + h_second
    first = _linear_combination(
        first_point,
        -(2.0 * h_first + h_second) / (h_first * total),
        second_point,
        total / (h_first * h_second),
        third_point,
        -h_first / (h_second * total),
    )
    second = _linear_combination(
        first_point,
        2.0 / (h_first * total),
        second_point,
        -2.0 / (h_first * h_second),
        third_point,
        2.0 / (h_second * total),
    )
    return first, second


def _geometry_values(
    points: Sequence[Point], circular: bool
) -> Tuple[list[float], list[float], Tuple[ValidationIssue, ...]]:
    count = len(points)
    psi_values: list[float] = []
    kappa_values: list[float] = []
    issues: list[ValidationIssue] = []

    for index in range(count):
        if count == 2:
            spacing = _distance(points[0], points[1])
            derivative = (
                (points[1][0] - points[0][0]) / spacing,
                (points[1][1] - points[0][1]) / spacing,
            )
            second_derivative = (0.0, 0.0)
        elif circular or 0 < index < count - 1:
            previous_index = (index - 1) % count
            next_index = (index + 1) % count
            derivative, second_derivative = _interior_derivatives(
                points[previous_index],
                points[index],
                points[next_index],
                _distance(points[previous_index], points[index]),
                _distance(points[index], points[next_index]),
            )
        elif index == 0:
            derivative, second_derivative = _forward_derivatives(
                points[0],
                points[1],
                points[2],
                _distance(points[0], points[1]),
                _distance(points[1], points[2]),
            )
        else:
            reverse_derivative, second_derivative = _forward_derivatives(
                points[-1],
                points[-2],
                points[-3],
                _distance(points[-1], points[-2]),
                _distance(points[-2], points[-3]),
            )
            derivative = (-reverse_derivative[0], -reverse_derivative[1])

        speed_squared = (
            derivative[0] * derivative[0]
            + derivative[1] * derivative[1]
        )
        if (
            not math.isfinite(speed_squared)
            or speed_squared <= _DERIVATIVE_EPSILON_SQUARED
        ):
            issues.append(
                ValidationIssue(
                    code="DEGENERATE_GEOMETRY_DERIVATIVE",
                    severity=Severity.ERROR,
                    message=(
                        "geometry derivative is too small to compute "
                        "heading/curvature"
                    ),
                    line_number=index + 2,
                    point_index=index,
                    value=speed_squared,
                )
            )
            psi_values.append(0.0)
            kappa_values.append(0.0)
            continue
        psi = math.atan2(derivative[1], derivative[0])
        denominator = speed_squared ** 1.5
        numerator = (
            derivative[0] * second_derivative[1]
            - derivative[1] * second_derivative[0]
        )
        kappa = numerator / denominator
        if not math.isfinite(kappa):
            issues.append(
                ValidationIssue(
                    code="NONFINITE_GEOMETRY_DERIVATIVE",
                    severity=Severity.ERROR,
                    message="generated curvature must be finite",
                    line_number=index + 2,
                    point_index=index,
                    value=kappa,
                )
            )
            kappa = 0.0
        psi_values.append(psi)
        kappa_values.append(kappa)
    return psi_values, kappa_values, tuple(issues)


def _spacing_metrics(
    points: Sequence[Point], circular: bool
) -> Tuple[
    float,
    Optional[float],
    Optional[float],
    Optional[float],
    Optional[float],
]:
    spacings = [
        _distance(points[index - 1], points[index])
        for index in range(1, len(points))
    ]
    closing = None
    if circular:
        closing = _distance(points[-1], points[0])
        spacings.append(closing)
    total = sum(spacings)
    if not spacings:
        return total, None, None, None, closing
    return (
        total,
        min(spacings),
        max(spacings),
        total / len(spacings),
        closing,
    )


def normalize_geometry(
    data: TrajectoryData,
    options: NormalizeOptions,
    *,
    source_revision: int,
) -> CandidateResult:
    """Create a validated MPC candidate without mutating ``data``."""

    mode = _validate_options(options, source_revision)
    _require_mpc_data(data)
    original = copy.deepcopy(data)
    input_path_length = _path_length(original.points, options.circular)

    (
        cleaned_points,
        retained_indices,
        closure_removed,
        degenerate_removed,
    ) = _clean_points(original.points, options)
    starts, cleaned_length = _canonical_arc(cleaned_points, options.circular)
    cleaned_rows = [original.rows[index] for index in retained_indices]

    if options.resample:
        queries, requested_spacing = _resample_queries(cleaned_length, options)
        output_points = [
            _sample_point(
                cleaned_points,
                starts,
                cleaned_length,
                query,
                options.circular,
            )
            for query in queries
        ]
    else:
        queries = list(starts)
        requested_spacing = 0.0
        output_points = list(cleaned_points)

    if (
        options.circular
        and _distance(output_points[-1], output_points[0])
        <= CLOSURE_TOLERANCE_M
    ):
        raise ValueError(
            "generated circular closing chord would be interpreted as a "
            f"duplicate endpoint (must be more than {CLOSURE_TOLERANCE_M:g} m)"
        )

    topology_or_count_changed = (
        bool(closure_removed)
        or bool(degenerate_removed)
        or len(output_points) != len(original.points)
    )
    if mode is MetadataMode.PRESERVE and topology_or_count_changed:
        raise ValueError(
            "metadata_mode='preserve' requires unchanged topology and "
            "point count; "
            "select metadata_mode='interpolate' after cleanup or resampling"
        )

    if mode is MetadataMode.PRESERVE:
        velocities = [row["vx_mps"] for row in original.rows]
        accelerations = [row["ax_mps2"] for row in original.rows]
    else:
        source_velocities = [float(row["vx_mps"]) for row in cleaned_rows]
        source_accelerations = [float(row["ax_mps2"]) for row in cleaned_rows]
        velocities = [
            _format_number(
                _sample_scalar(
                    source_velocities,
                    starts,
                    cleaned_length,
                    query,
                    options.circular,
                )
            )
            for query in queries
        ]
        accelerations = [
            _format_number(
                _sample_scalar(
                    source_accelerations,
                    starts,
                    cleaned_length,
                    query,
                    options.circular,
                )
            )
            for query in queries
        ]

    output_starts, _unused_output_arc_length = _canonical_arc(
        output_points, options.circular
    )
    psi_values, kappa_values, geometry_issues = _geometry_values(
        output_points, options.circular
    )
    output_rows = [
        {
            "s_m": _format_number(output_starts[index]),
            "x_m": _format_number(point[0]),
            "y_m": _format_number(point[1]),
            "psi_rad": _format_number(psi_values[index]),
            "kappa_radpm": _format_number(kappa_values[index]),
            "vx_mps": velocities[index],
            "ax_mps2": accelerations[index],
        }
        for index, point in enumerate(output_points)
    ]
    candidate_data = TrajectoryData(
        path=original.path,
        fieldnames=list(MPC_COLUMNS),
        rows=output_rows,
        points=list(output_points),
        x_column="x_m",
        y_column="y_m",
        format_name="mpc",
    )
    validation = validate_trajectory(
        candidate_data.fieldnames,
        candidate_data.rows,
        candidate_data.format_name,
        options.circular,
        initial_issues=geometry_issues,
    )
    (
        output_length,
        output_min_spacing,
        output_max_spacing,
        output_mean_spacing,
        output_closing_spacing,
    ) = _spacing_metrics(output_points, options.circular)
    transformation = TransformationReport(
        input_point_count=len(original.points),
        cleaned_point_count=len(cleaned_points),
        output_point_count=len(output_points),
        retained_source_indices=tuple(retained_indices),
        removed_closure_indices=closure_removed,
        removed_degenerate_indices=degenerate_removed,
        circular=options.circular,
        resampled=options.resample,
        requested_resolution_m=(
            float(options.resolution_m) if options.resample else None
        ),
        metadata_mode=mode,
        input_path_length_m=input_path_length,
        cleaned_path_length_m=cleaned_length,
        output_path_length_m=output_length,
        output_min_spacing_m=output_min_spacing,
        output_max_spacing_m=output_max_spacing,
        output_mean_spacing_m=output_mean_spacing,
        output_closing_spacing_m=output_closing_spacing,
    )
    parameters: Mapping[str, object] = MappingProxyType(
        {
            "circular": options.circular,
            "metadata_mode": mode.value,
            "remove_closure_duplicate": options.remove_closure_duplicate,
            "remove_degenerate_points": options.remove_degenerate_points,
            "resample": options.resample,
            "resolution_m": float(options.resolution_m),
            "requested_spacing_m": (
                requested_spacing if options.resample else None
            ),
        }
    )
    return CandidateResult(
        source_revision=source_revision,
        operation="normalize_geometry",
        parameters=parameters,
        dataset=candidate_data,
        validation=validation,
        transformation=transformation,
    )
