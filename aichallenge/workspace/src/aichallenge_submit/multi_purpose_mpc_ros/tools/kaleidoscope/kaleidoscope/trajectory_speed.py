"""Pure, offline speed-profile generation for MPC trajectory CSV data.

The generator deliberately does not write files and never mutates its source
``TrajectoryData``.  A successful result contains a deep-copied candidate;
any invalid input, infeasible constraint, non-convergence, or post-validation
failure returns ``candidate=None``.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
import math
from numbers import Real
from typing import Callable, Mapping, Optional, Sequence

from .trajectory_contract import Severity
from .trajectory_contract import TrajectoryData
from .trajectory_contract import TrajectoryMetrics
from .trajectory_contract import ValidationIssue
from .trajectory_contract import ValidationLimits
from .trajectory_contract import ValidationReport
from .trajectory_contract import validate_trajectory


@dataclass(frozen=True)
class SpeedProfileParameters:
    """Finite safety limits used to generate an offline MPC speed profile."""

    v_max_mps: float
    a_max_mps2: float
    a_min_mps2: float
    ay_max_mps2: float
    minimum_speed_mps: float = 0.0
    epsilon: float = 1e-9
    tolerance: float = 1e-9
    max_iterations: int = 1000


@dataclass(frozen=True)
class SpeedProfileResult:
    """Revision-bound candidate and its independent validation report."""

    source_revision: int
    candidate: Optional[TrajectoryData]
    report: ValidationReport
    iterations: int


def _empty_metrics(point_count: int = 0) -> TrajectoryMetrics:
    return TrajectoryMetrics(
        point_count=point_count,
        normalized_point_count=point_count,
        duplicate_endpoint=False,
        closure_distance_m=None,
        closing_edge_spacing_m=None,
        total_distance_m=None,
        min_spacing_m=None,
        max_spacing_m=None,
        mean_spacing_m=None,
        max_abs_psi_difference_rad=None,
        max_abs_kappa_radpm=None,
        max_abs_kappa_difference_radpm=None,
        min_velocity_mps=None,
        max_velocity_mps=None,
        min_acceleration_mps2=None,
        max_acceleration_mps2=None,
        max_lateral_acceleration_mps2=None,
    )


def _issue(
    code: str,
    message: str,
    *,
    column: Optional[str] = None,
    value: object = None,
    point_index: Optional[int] = None,
    segment_index: Optional[int] = None,
    s_m: Optional[float] = None,
) -> ValidationIssue:
    return ValidationIssue(
        code=code,
        severity=Severity.ERROR,
        message=message,
        line_number=(point_index + 2 if point_index is not None else None),
        point_index=point_index,
        segment_index=segment_index,
        s_m=s_m,
        column=column,
        value=value,
    )


def _is_finite_real(value: object) -> bool:
    return (
        isinstance(value, Real)
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _parameter_issues(
    parameters: SpeedProfileParameters,
    circular: object,
    source_revision: object,
) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []

    def require_real(
        name: str,
        value: object,
        predicate: Callable[[float], bool],
        description: str,
    ) -> None:
        valid = _is_finite_real(value)
        if valid:
            valid = predicate(float(value))
        if not valid:
            issues.append(
                _issue(
                    "INVALID_SPEED_PARAMETER",
                    f"{name} must be finite and {description}",
                    column=name,
                    value=value,
                )
            )

    require_real("v_max_mps", parameters.v_max_mps, lambda value: value > 0.0, "positive")
    require_real(
        "a_max_mps2", parameters.a_max_mps2, lambda value: value > 0.0, "positive"
    )
    require_real(
        "a_min_mps2", parameters.a_min_mps2, lambda value: value < 0.0, "negative"
    )
    require_real(
        "ay_max_mps2", parameters.ay_max_mps2, lambda value: value > 0.0, "positive"
    )
    require_real(
        "minimum_speed_mps",
        parameters.minimum_speed_mps,
        lambda value: value >= 0.0,
        "non-negative",
    )
    require_real("epsilon", parameters.epsilon, lambda value: value > 0.0, "positive")
    require_real(
        "tolerance", parameters.tolerance, lambda value: value > 0.0, "positive"
    )

    if (
        not isinstance(parameters.max_iterations, int)
        or isinstance(parameters.max_iterations, bool)
        or parameters.max_iterations <= 0
    ):
        issues.append(
            _issue(
                "INVALID_SPEED_PARAMETER",
                "max_iterations must be a positive integer",
                column="max_iterations",
                value=parameters.max_iterations,
            )
        )
    if not isinstance(circular, bool):
        issues.append(
            _issue(
                "INVALID_SPEED_PARAMETER",
                "circular must be a bool",
                column="circular",
                value=circular,
            )
        )
    if (
        not isinstance(source_revision, int)
        or isinstance(source_revision, bool)
        or source_revision < 0
    ):
        issues.append(
            _issue(
                "INVALID_SPEED_PARAMETER",
                "source_revision must be a non-negative integer",
                column="source_revision",
                value=source_revision,
            )
        )
    return issues


def _validated_report(
    data: TrajectoryData,
    *,
    circular: bool,
    initial_issues: Sequence[ValidationIssue] = (),
    limits: Optional[ValidationLimits] = None,
) -> ValidationReport:
    if data.format_name in ("mpc", "pure_pursuit"):
        return validate_trajectory(
            data.fieldnames,
            data.rows,
            data.format_name,
            circular,
            limits,
            initial_issues=initial_issues,
        )
    return ValidationReport(
        format_name=data.format_name,
        circular=circular,
        issues=tuple(initial_issues),
        metrics=_empty_metrics(len(data.rows)),
    )


def _row_float(row: Mapping[object, object], column: str) -> float:
    """Read a validated column while tolerating normalized header whitespace."""

    if column in row:
        return float(row[column])
    for raw_name, raw_value in row.items():
        if raw_name is None:
            continue
        normalized = str(raw_name).strip(" \t\n\r\v\f")
        if normalized.startswith("\ufeff"):
            normalized = normalized[1:].strip(" \t\n\r\v\f")
        if normalized == column:
            return float(raw_value)
    raise KeyError(column)


def _set_row_value(row: dict[object, object], column: str, value: str) -> None:
    if column in row:
        row[column] = value
        return
    for raw_name in row:
        if raw_name is None:
            continue
        normalized = str(raw_name).strip(" \t\n\r\v\f")
        if normalized.startswith("\ufeff"):
            normalized = normalized[1:].strip(" \t\n\r\v\f")
        if normalized == column:
            row[raw_name] = value
            return
    raise KeyError(column)


def _curvature_cap(
    kappa_radpm: float,
    parameters: SpeedProfileParameters,
) -> float:
    denominator = max(abs(kappa_radpm), float(parameters.epsilon))
    ratio = float(parameters.ay_max_mps2) / denominator
    lateral_cap = math.inf if math.isinf(ratio) else math.sqrt(ratio)
    return min(float(parameters.v_max_mps), lateral_cap)


def _reachable_speed(start_speed: float, acceleration: float, distance: float) -> float:
    term = 2.0 * acceleration * distance
    if math.isinf(term):
        return math.inf
    return math.hypot(start_speed, math.sqrt(term))


def _edge_acceleration(start_speed: float, end_speed: float, distance: float) -> float:
    if start_speed == end_speed:
        return 0.0
    return ((end_speed - start_speed) * (end_speed + start_speed)) / (2.0 * distance)


def _post_issues(
    rows: Sequence[Mapping[object, object]],
    velocities: Sequence[float],
    caps: Sequence[float],
    edges: Sequence[tuple[int, int, float]],
    parameters: SpeedProfileParameters,
) -> list[ValidationIssue]:
    issues: list[ValidationIssue] = []
    tolerance = float(parameters.tolerance)
    minimum = float(parameters.minimum_speed_mps)
    for index, (velocity, cap) in enumerate(zip(velocities, caps)):
        s_m = _row_float(rows[index], "s_m")
        if velocity < minimum - tolerance:
            issues.append(
                _issue(
                    "MINIMUM_SPEED_VIOLATION",
                    "generated velocity is below minimum_speed_mps",
                    column="vx_mps",
                    value=velocity,
                    point_index=index,
                    s_m=s_m,
                )
            )
        if velocity > cap + tolerance:
            issues.append(
                _issue(
                    "SPEED_CAP_VIOLATION",
                    "generated velocity exceeds its curvature speed cap",
                    column="vx_mps",
                    value=velocity,
                    point_index=index,
                    s_m=s_m,
                )
            )

    for segment_index, (start, end, distance) in enumerate(edges):
        forward_limit = _reachable_speed(
            velocities[start], float(parameters.a_max_mps2), distance
        )
        if velocities[end] > forward_limit + tolerance:
            issues.append(
                _issue(
                    "FORWARD_SPEED_CONSTRAINT_VIOLATION",
                    "generated edge violates the forward acceleration constraint",
                    column="vx_mps",
                    value=velocities[end],
                    point_index=end,
                    segment_index=segment_index,
                    s_m=_row_float(rows[end], "s_m"),
                )
            )
        backward_limit = _reachable_speed(
            velocities[end], abs(float(parameters.a_min_mps2)), distance
        )
        if velocities[start] > backward_limit + tolerance:
            issues.append(
                _issue(
                    "BACKWARD_SPEED_CONSTRAINT_VIOLATION",
                    "generated edge violates the backward deceleration constraint",
                    column="vx_mps",
                    value=velocities[start],
                    point_index=start,
                    segment_index=segment_index,
                    s_m=_row_float(rows[start], "s_m"),
                )
            )
    return issues


def recompute_speed_profile(
    data: TrajectoryData,
    *,
    circular: bool,
    parameters: SpeedProfileParameters,
    source_revision: int,
) -> SpeedProfileResult:
    """Build a constraint-checked MPC speed candidate without mutating ``data``."""

    parameter_issues = _parameter_issues(parameters, circular, source_revision)
    topology = circular if isinstance(circular, bool) else False
    if data.format_name != "mpc":
        parameter_issues.append(
            _issue(
                "UNSUPPORTED_SPEED_PROFILE_FORMAT",
                "offline speed-profile generation supports MPC trajectories only",
                column="format_name",
                value=data.format_name,
            )
        )
    source_report = _validated_report(
        data,
        circular=topology,
        initial_issues=parameter_issues,
    )
    if not source_report.is_valid:
        return SpeedProfileResult(
            source_revision=source_revision,
            candidate=None,
            report=source_report,
            iterations=0,
        )

    normalized_count = source_report.metrics.normalized_point_count
    rows = data.rows
    points = [
        (_row_float(row, "x_m"), _row_float(row, "y_m"))
        for row in rows[:normalized_count]
    ]
    kappas = [
        _row_float(row, "kappa_radpm") for row in rows[:normalized_count]
    ]
    caps = [_curvature_cap(kappa, parameters) for kappa in kappas]
    minimum = float(parameters.minimum_speed_mps)
    tolerance = float(parameters.tolerance)
    infeasible = [
        _issue(
            "MINIMUM_SPEED_INFEASIBLE",
            "minimum_speed_mps exceeds the safe curvature speed cap",
            column="minimum_speed_mps",
            value=minimum,
            point_index=index,
            s_m=_row_float(rows[index], "s_m"),
        )
        for index, cap in enumerate(caps)
        if minimum > cap + tolerance
    ]
    if infeasible:
        report = _validated_report(
            data,
            circular=topology,
            initial_issues=infeasible,
        )
        return SpeedProfileResult(source_revision, None, report, 0)

    edges: list[tuple[int, int, float]] = []
    for start in range(max(0, normalized_count - 1)):
        end = start + 1
        distance = math.hypot(
            points[end][0] - points[start][0],
            points[end][1] - points[start][1],
        )
        edges.append((start, end, distance))
    if topology:
        distance = math.hypot(
            points[0][0] - points[-1][0],
            points[0][1] - points[-1][1],
        )
        edges.append((normalized_count - 1, 0, distance))

    velocities = list(caps)
    converged = False
    iterations = 0
    for iterations in range(1, parameters.max_iterations + 1):
        maximum_change = 0.0
        for start, end, distance in edges:
            limit = _reachable_speed(
                velocities[start], float(parameters.a_max_mps2), distance
            )
            if limit < velocities[end]:
                maximum_change = max(maximum_change, velocities[end] - limit)
                velocities[end] = limit
        for start, end, distance in reversed(edges):
            limit = _reachable_speed(
                velocities[end], abs(float(parameters.a_min_mps2)), distance
            )
            if limit < velocities[start]:
                maximum_change = max(maximum_change, velocities[start] - limit)
                velocities[start] = limit
        if maximum_change <= tolerance:
            converged = True
            break

    if not converged:
        report = _validated_report(
            data,
            circular=topology,
            initial_issues=(
                _issue(
                    "SPEED_PROFILE_NONCONVERGENCE",
                    "speed-profile relaxation did not converge within max_iterations",
                    column="max_iterations",
                    value=parameters.max_iterations,
                ),
            ),
        )
        return SpeedProfileResult(source_revision, None, report, iterations)

    candidate = copy.deepcopy(data)
    accelerations: list[float] = [0.0] * normalized_count
    for start, end, distance in edges:
        acceleration = _edge_acceleration(
            velocities[start], velocities[end], distance
        )
        if not math.isfinite(acceleration):
            report = _validated_report(
                data,
                circular=topology,
                initial_issues=(
                    _issue(
                        "NONFINITE_SPEED_PROFILE_DERIVATION",
                        "derived outgoing acceleration must be finite",
                        column="ax_mps2",
                        value=acceleration,
                        point_index=start,
                        segment_index=start,
                        s_m=_row_float(rows[start], "s_m"),
                    ),
                ),
            )
            return SpeedProfileResult(source_revision, None, report, iterations)
        accelerations[start] = acceleration
    if not topology:
        accelerations[-1] = 0.0

    for index in range(normalized_count):
        _set_row_value(candidate.rows[index], "vx_mps", repr(velocities[index]))
        acceleration_text = (
            "0.0" if not topology and index == normalized_count - 1
            else repr(accelerations[index])
        )
        _set_row_value(candidate.rows[index], "ax_mps2", acceleration_text)

    duplicate_endpoint = source_report.metrics.duplicate_endpoint
    if duplicate_endpoint:
        _set_row_value(candidate.rows[-1], "vx_mps", repr(velocities[0]))
        _set_row_value(candidate.rows[-1], "ax_mps2", repr(accelerations[0]))

    post_issues = _post_issues(
        candidate.rows[:normalized_count],
        velocities,
        caps,
        edges,
        parameters,
    )
    limits = ValidationLimits(
        v_max_mps=float(parameters.v_max_mps),
        a_max_mps2=float(parameters.a_max_mps2),
        a_min_mps2=float(parameters.a_min_mps2),
        ay_max_mps2=float(parameters.ay_max_mps2),
        tolerance=tolerance,
    )
    report = _validated_report(
        candidate,
        circular=topology,
        initial_issues=post_issues,
        limits=limits,
    )
    if not report.is_valid:
        return SpeedProfileResult(source_revision, None, report, iterations)
    return SpeedProfileResult(source_revision, candidate, report, iterations)
