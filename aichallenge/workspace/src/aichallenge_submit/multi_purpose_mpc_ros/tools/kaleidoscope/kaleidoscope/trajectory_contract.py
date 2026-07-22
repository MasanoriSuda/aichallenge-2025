"""Pure validation and safe CSV writing for the Kaleidoscope editor.

This module intentionally has no ROS 2 or Tkinter dependency so its safety
rules can be exercised in headless unit tests.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from decimal import Decimal
from decimal import InvalidOperation
from enum import Enum
import math
import os
from pathlib import Path
import re
import stat
import sys
import tempfile
from typing import Callable, Mapping, Optional, Sequence, Tuple


MPC_COLUMNS: Tuple[str, ...] = (
    "s_m",
    "x_m",
    "y_m",
    "psi_rad",
    "kappa_radpm",
    "vx_mps",
    "ax_mps2",
)
PURE_PURSUIT_COLUMNS: Tuple[str, ...] = (
    "x",
    "y",
    "z",
    "x_quat",
    "y_quat",
    "z_quat",
    "w_quat",
    "speed",
)
CLOSURE_TOLERANCE_M = 1e-3
MIN_SEGMENT_LENGTH_M = 1e-6
Point = Tuple[float, float]

_DECIMAL_NUMBER = re.compile(
    r"^[+-]?(?:(?:[0-9]+(?:\.[0-9]*)?)|(?:\.[0-9]+))(?:[eE][+-]?[0-9]+)?$"
)
_NONFINITE_NUMBER = re.compile(
    r"^[+-]?(?:inf(?:inity)?|nan(?:\([^)]*\))?)$", re.IGNORECASE
)
_ASCII_WHITESPACE = " \t\n\r\v\f"


class Severity(str, Enum):
    ERROR = "error"
    WARNING = "warning"
    INFO = "info"


@dataclass
class TrajectoryData:
    """Mutable compatibility dataset shared by pure processing and the Tk UI."""

    path: Path
    fieldnames: list[str]
    rows: list[dict[str, str]]
    points: list[Point]
    x_column: str
    y_column: str
    format_name: str


@dataclass(frozen=True)
class ValidationIssue:
    code: str
    severity: Severity
    message: str
    line_number: Optional[int] = None
    point_index: Optional[int] = None
    segment_index: Optional[int] = None
    s_m: Optional[float] = None
    column: Optional[str] = None
    value: object = None


@dataclass(frozen=True)
class TrajectoryMetrics:
    point_count: int
    normalized_point_count: int
    duplicate_endpoint: bool
    closure_distance_m: Optional[float]
    closing_edge_spacing_m: Optional[float]
    total_distance_m: Optional[float]
    min_spacing_m: Optional[float]
    max_spacing_m: Optional[float]
    mean_spacing_m: Optional[float]
    max_abs_psi_difference_rad: Optional[float]
    max_abs_kappa_radpm: Optional[float]
    max_abs_kappa_difference_radpm: Optional[float]
    min_velocity_mps: Optional[float]
    max_velocity_mps: Optional[float]
    min_acceleration_mps2: Optional[float]
    max_acceleration_mps2: Optional[float]
    max_lateral_acceleration_mps2: Optional[float]


@dataclass(frozen=True)
class ValidationLimits:
    v_max_mps: Optional[float] = None
    a_max_mps2: Optional[float] = None
    a_min_mps2: Optional[float] = None
    ay_max_mps2: Optional[float] = None
    tolerance: float = 1e-9


@dataclass(frozen=True)
class ValidationReport:
    format_name: str
    circular: bool
    issues: Tuple[ValidationIssue, ...]
    metrics: TrajectoryMetrics

    @property
    def error_count(self) -> int:
        return sum(issue.severity is Severity.ERROR for issue in self.issues)

    @property
    def warning_count(self) -> int:
        return sum(issue.severity is Severity.WARNING for issue in self.issues)

    @property
    def info_count(self) -> int:
        return sum(issue.severity is Severity.INFO for issue in self.issues)

    @property
    def is_valid(self) -> bool:
        return self.error_count == 0


def _normalize_header(value: object, first: bool = False) -> str:
    normalized = "" if value is None else str(value).strip(_ASCII_WHITESPACE)
    if first and normalized.startswith("\ufeff"):
        normalized = normalized[1:].strip(_ASCII_WHITESPACE)
    return normalized


def _normalized_fieldnames(fieldnames: Sequence[object]) -> Tuple[str, ...]:
    return tuple(
        _normalize_header(value, first=index == 0)
        for index, value in enumerate(fieldnames)
    )


def detect_trajectory_format(fieldnames: Sequence[object]) -> str:
    """Return ``mpc`` or ``pure_pursuit`` from a CSV header."""

    normalized = set(_normalized_fieldnames(fieldnames))
    if {"x_m", "y_m"}.issubset(normalized):
        return "mpc"
    if {"x", "y"}.issubset(normalized):
        return "pure_pursuit"
    raise ValueError("trajectory header must contain x_m/y_m or x/y columns")


def _expected_columns(format_name: str) -> Tuple[str, ...]:
    if format_name == "mpc":
        return MPC_COLUMNS
    if format_name == "pure_pursuit":
        return PURE_PURSUIT_COLUMNS
    raise ValueError(f"unsupported trajectory format: {format_name}")


def _parse_number(raw_value: object) -> Tuple[Optional[float], Optional[str]]:
    if raw_value is None:
        return None, "numeric value is missing"
    value = str(raw_value).strip(_ASCII_WHITESPACE)
    if not value:
        return None, "numeric value is empty"
    if _NONFINITE_NUMBER.fullmatch(value):
        return None, "numeric value must be finite"
    if not _DECIMAL_NUMBER.fullmatch(value):
        return None, "invalid or partially converted numeric value"
    try:
        parsed = float(value)
    except (OverflowError, ValueError):
        return None, "invalid numeric value"
    if not math.isfinite(parsed):
        return None, "numeric value must be finite"
    if parsed != 0.0 and abs(parsed) < sys.float_info.min:
        return None, "numeric value is outside the normal double range"
    if parsed == 0.0:
        try:
            if Decimal(value) != 0:
                return None, "numeric value is outside double range"
        except InvalidOperation:
            return None, "invalid numeric value"
    return parsed, None


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


def _issue_sort_key(issue: ValidationIssue) -> tuple:
    severity_order = {
        Severity.ERROR: 0,
        Severity.WARNING: 1,
        Severity.INFO: 2,
    }
    return (
        severity_order[issue.severity],
        issue.line_number if issue.line_number is not None else 1 << 60,
        issue.point_index if issue.point_index is not None else 1 << 60,
        issue.segment_index if issue.segment_index is not None else 1 << 60,
        issue.code,
        issue.column or "",
    )


def _safe_difference(
    lhs: float,
    rhs: float,
    *,
    code: str,
    label: str,
    line_number: Optional[int],
    point_index: Optional[int],
    issues: list[ValidationIssue],
) -> Optional[float]:
    difference = lhs - rhs
    if math.isfinite(difference):
        return difference
    issues.append(
        ValidationIssue(
            code=code,
            severity=Severity.ERROR,
            message=f"derived {label} must be finite",
            line_number=line_number,
            point_index=point_index,
            value=difference,
        )
    )
    return None


def _wrapped_angle_difference(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _validate_limits(limits: ValidationLimits) -> Tuple[ValidationIssue, ...]:
    issues: list[ValidationIssue] = []

    def require(
        name: str,
        value: Optional[float],
        predicate: Callable[[float], bool],
        description: str,
    ) -> None:
        if value is None:
            return
        if not math.isfinite(value) or not predicate(value):
            issues.append(
                ValidationIssue(
                    code="INVALID_LIMIT",
                    severity=Severity.ERROR,
                    message=f"{name} must be finite and {description}",
                    column=name,
                    value=value,
                )
            )

    require("v_max_mps", limits.v_max_mps, lambda value: value > 0.0, "positive")
    require("a_max_mps2", limits.a_max_mps2, lambda value: value > 0.0, "positive")
    require("a_min_mps2", limits.a_min_mps2, lambda value: value < 0.0, "negative")
    require("ay_max_mps2", limits.ay_max_mps2, lambda value: value > 0.0, "positive")
    require("tolerance", limits.tolerance, lambda value: value >= 0.0, "non-negative")
    return tuple(issues)


def validate_trajectory(
    fieldnames: Sequence[object],
    rows: Sequence[Mapping[object, object]],
    format_name: str,
    circular: bool,
    limits: Optional[ValidationLimits] = None,
    *,
    line_numbers: Optional[Sequence[int]] = None,
    initial_issues: Sequence[ValidationIssue] = (),
) -> ValidationReport:
    """Validate a trajectory table without mutating its inputs."""

    limits = limits or ValidationLimits()
    issues: list[ValidationIssue] = list(initial_issues)
    issues.extend(_validate_limits(limits))
    expected = _expected_columns(format_name)
    normalized_headers = _normalized_fieldnames(fieldnames)

    header_counts = {header: normalized_headers.count(header) for header in normalized_headers}
    for index, header in enumerate(normalized_headers):
        if not header:
            issues.append(
                ValidationIssue(
                    code="EMPTY_HEADER",
                    severity=Severity.ERROR,
                    message="CSV header name is empty",
                    line_number=1,
                    column=f"column-{index + 1}",
                    value=header,
                )
            )
        elif header_counts[header] > 1 and normalized_headers.index(header) == index:
            issues.append(
                ValidationIssue(
                    code="DUPLICATE_HEADER",
                    severity=Severity.ERROR,
                    message="CSV header name is duplicated",
                    line_number=1,
                    column=header,
                    value=header,
                )
            )

    actual_set = set(normalized_headers)
    expected_set = set(expected)
    for missing in sorted(expected_set - actual_set):
        issues.append(
            ValidationIssue(
                code="MISSING_HEADER",
                severity=Severity.ERROR,
                message="required CSV header is missing",
                line_number=1,
                column=missing,
                value="<missing>",
            )
        )
    for extra in sorted(actual_set - expected_set):
        if extra:
            issues.append(
                ValidationIssue(
                    code="EXTRA_HEADER",
                    severity=Severity.ERROR,
                    message="unexpected CSV header is not allowed",
                    line_number=1,
                    column=extra,
                    value=extra,
                )
            )

    if len(normalized_headers) != len(expected):
        issues.append(
            ValidationIssue(
                code="HEADER_COLUMN_COUNT",
                severity=Severity.ERROR,
                message=f"CSV must contain exactly {len(expected)} columns",
                line_number=1,
                value=len(normalized_headers),
            )
        )

    if not rows:
        issues.append(
            ValidationIssue(
                code="NO_DATA_ROWS",
                severity=Severity.ERROR,
                message="trajectory CSV has no data rows",
                line_number=2,
            )
        )

    if line_numbers is not None and len(line_numbers) != len(rows):
        raise ValueError("line_numbers must have one entry per data row")

    raw_to_normalized = {
        raw: normalized
        for raw, normalized in zip(fieldnames, normalized_headers)
        if normalized
    }
    typed_rows: list[dict[str, float]] = []
    typed_line_numbers: list[int] = []

    schema_valid = not any(
        issue.severity is Severity.ERROR and issue.line_number == 1
        for issue in issues
    )
    if schema_valid:
        for row_index, row in enumerate(rows):
            line_number = (
                line_numbers[row_index] if line_numbers is not None else row_index + 2
            )
            if None in row:
                issues.append(
                    ValidationIssue(
                        code="EXTRA_ROW_FIELDS",
                        severity=Severity.ERROR,
                        message="row contains more fields than the header",
                        line_number=line_number,
                        point_index=row_index,
                        value=row.get(None),
                    )
                )

            normalized_row: dict[str, object] = {}
            for raw_header, normalized_header in raw_to_normalized.items():
                normalized_row[normalized_header] = row.get(raw_header)

            typed: dict[str, float] = {}
            row_valid = None not in row
            for column in expected:
                raw_value = normalized_row.get(column)
                parsed, reason = _parse_number(raw_value)
                if reason is not None:
                    issues.append(
                        ValidationIssue(
                            code="INVALID_NUMBER",
                            severity=Severity.ERROR,
                            message=reason,
                            line_number=line_number,
                            point_index=row_index,
                            column=column,
                            value=raw_value,
                        )
                    )
                    row_valid = False
                else:
                    typed[column] = parsed  # type: ignore[assignment]
            if row_valid:
                typed_rows.append(typed)
                typed_line_numbers.append(line_number)

    if len(typed_rows) != len(rows):
        report_issues = tuple(sorted(issues, key=_issue_sort_key))
        return ValidationReport(
            format_name=format_name,
            circular=circular,
            issues=report_issues,
            metrics=_empty_metrics(len(rows)),
        )

    minimum_points = 3 if circular else 2
    if len(typed_rows) < minimum_points:
        issues.append(
            ValidationIssue(
                code="INSUFFICIENT_POINTS",
                severity=Severity.ERROR,
                message=f"trajectory requires at least {minimum_points} points",
                line_number=typed_line_numbers[-1] + 1 if typed_line_numbers else 2,
                value=len(typed_rows),
            )
        )

    if format_name == "mpc":
        for index in range(1, len(typed_rows)):
            previous = typed_rows[index - 1]["s_m"]
            current = typed_rows[index]["s_m"]
            if current <= previous:
                issues.append(
                    ValidationIssue(
                        code="NON_INCREASING_S",
                        severity=Severity.ERROR,
                        message="s_m must be strictly increasing",
                        line_number=typed_line_numbers[index],
                        point_index=index,
                        s_m=current,
                        column="s_m",
                        value=current,
                    )
                )

    x_column, y_column = ("x_m", "y_m") if format_name == "mpc" else ("x", "y")
    points = [(row[x_column], row[y_column]) for row in typed_rows]

    # Match the C++ strict loader before any legacy duplicate endpoint is
    # normalized away. In particular, the raw final adjacent segment remains
    # part of the file contract even when the endpoint is later removed.
    for end in range(1, len(points)):
        start = end - 1
        raw_spacing = math.hypot(
            points[end][0] - points[start][0],
            points[end][1] - points[start][1],
        )
        if not math.isfinite(raw_spacing):
            issues.append(
                ValidationIssue(
                    code="NONFINITE_SEGMENT_LENGTH",
                    severity=Severity.ERROR,
                    message="derived segment length must be finite",
                    line_number=typed_line_numbers[end],
                    point_index=end,
                    segment_index=start,
                    value=raw_spacing,
                )
            )
        elif raw_spacing <= MIN_SEGMENT_LENGTH_M:
            exact_duplicate = raw_spacing == 0.0
            issues.append(
                ValidationIssue(
                    code=("DUPLICATE_POINT" if exact_duplicate else "DEGENERATE_SEGMENT"),
                    severity=Severity.ERROR,
                    message=(
                        "consecutive trajectory points are identical"
                        if exact_duplicate
                        else f"segment length must be more than {MIN_SEGMENT_LENGTH_M:g} m"
                    ),
                    line_number=typed_line_numbers[end],
                    point_index=end,
                    segment_index=start,
                    s_m=(typed_rows[end].get("s_m") if format_name == "mpc" else None),
                    value=raw_spacing,
                )
            )

    closure_distance: Optional[float] = None
    duplicate_endpoint = False
    if len(points) >= 2:
        closure_distance = math.hypot(
            points[-1][0] - points[0][0], points[-1][1] - points[0][1]
        )
        if not math.isfinite(closure_distance):
            issues.append(
                ValidationIssue(
                    code="NONFINITE_CLOSURE_DISTANCE",
                    severity=Severity.ERROR,
                    message="derived closure distance must be finite",
                    line_number=typed_line_numbers[-1],
                    point_index=len(points) - 1,
                    value=closure_distance,
                )
            )
            closure_distance = None
        elif circular and len(points) > 2 and closure_distance <= CLOSURE_TOLERANCE_M:
            duplicate_endpoint = True
            issues.append(
                ValidationIssue(
                    code="DUPLICATE_ENDPOINT",
                    severity=Severity.WARNING,
                    message="legacy circular endpoint duplicates the first point",
                    line_number=typed_line_numbers[-1],
                    point_index=len(points) - 1,
                    s_m=(typed_rows[-1].get("s_m") if format_name == "mpc" else None),
                    value=closure_distance,
                )
            )

    normalized_count = len(points) - (1 if duplicate_endpoint else 0)
    if circular and normalized_count < 3:
        issues.append(
            ValidationIssue(
                code="INSUFFICIENT_UNIQUE_POINTS",
                severity=Severity.ERROR,
                message="circular trajectory requires at least 3 unique points",
                value=normalized_count,
            )
        )

    segment_pairs = [(index, index + 1) for index in range(max(0, normalized_count - 1))]
    if circular and normalized_count >= 2:
        segment_pairs.append((normalized_count - 1, 0))

    spacings: list[float] = []
    closing_edge_spacing: Optional[float] = None
    total_distance = 0.0
    for segment_index, (start, end) in enumerate(segment_pairs):
        dx = points[end][0] - points[start][0]
        dy = points[end][1] - points[start][1]
        spacing = math.hypot(dx, dy)
        if not math.isfinite(spacing):
            if circular and end == 0:
                issues.append(
                    ValidationIssue(
                        code="NONFINITE_CLOSING_SEGMENT_LENGTH",
                        severity=Severity.ERROR,
                        message="derived closing segment length must be finite",
                        line_number=typed_line_numbers[end],
                        point_index=end,
                        segment_index=segment_index,
                        value=spacing,
                    )
                )
            continue
        if circular and end == 0 and spacing <= MIN_SEGMENT_LENGTH_M:
            exact_duplicate = spacing == 0.0
            issues.append(
                ValidationIssue(
                    code=(
                        "DUPLICATE_CLOSING_POINT"
                        if exact_duplicate
                        else "DEGENERATE_CLOSING_SEGMENT"
                    ),
                    severity=Severity.ERROR,
                    message=(
                        "last unique point duplicates the first point"
                        if exact_duplicate
                        else f"closing segment length must be more than "
                        f"{MIN_SEGMENT_LENGTH_M:g} m"
                    ),
                    line_number=typed_line_numbers[end],
                    point_index=end,
                    segment_index=segment_index,
                    s_m=(typed_rows[end].get("s_m") if format_name == "mpc" else None),
                    value=spacing,
                )
            )
        spacings.append(spacing)
        if circular and end == 0:
            closing_edge_spacing = spacing
        total_distance += spacing
        if not math.isfinite(total_distance):
            issues.append(
                ValidationIssue(
                    code="NONFINITE_TOTAL_DISTANCE",
                    severity=Severity.ERROR,
                    message="derived total path length must be finite",
                    segment_index=segment_index,
                    value=total_distance,
                )
            )
            total_distance = math.nan
            break

    max_abs_psi_difference: Optional[float] = None
    max_abs_kappa_difference: Optional[float] = None
    max_abs_kappa: Optional[float] = None
    min_velocity: Optional[float] = None
    max_velocity: Optional[float] = None
    min_acceleration: Optional[float] = None
    max_acceleration: Optional[float] = None
    max_lateral_acceleration: Optional[float] = None

    if format_name == "mpc" and typed_rows:
        kappas = [row["kappa_radpm"] for row in typed_rows[:normalized_count]]
        velocities = [row["vx_mps"] for row in typed_rows[:normalized_count]]
        accelerations = [row["ax_mps2"] for row in typed_rows[:normalized_count]]
        max_abs_kappa = max((abs(value) for value in kappas), default=None)
        min_velocity = min(velocities, default=None)
        max_velocity = max(velocities, default=None)
        min_acceleration = min(accelerations, default=None)
        max_acceleration = max(accelerations, default=None)

        psi_differences: list[float] = []
        kappa_differences: list[float] = []
        for segment_index, (start, end) in enumerate(segment_pairs):
            raw_psi_difference = _safe_difference(
                typed_rows[end]["psi_rad"],
                typed_rows[start]["psi_rad"],
                code="NONFINITE_PSI_DIFFERENCE",
                label="psi difference",
                line_number=typed_line_numbers[end],
                point_index=end,
                issues=issues,
            )
            if raw_psi_difference is not None:
                psi_differences.append(abs(_wrapped_angle_difference(raw_psi_difference)))
            kappa_difference = _safe_difference(
                typed_rows[end]["kappa_radpm"],
                typed_rows[start]["kappa_radpm"],
                code="NONFINITE_KAPPA_DIFFERENCE",
                label="kappa difference",
                line_number=typed_line_numbers[end],
                point_index=end,
                issues=issues,
            )
            if kappa_difference is not None:
                kappa_differences.append(abs(kappa_difference))
        if not circular and normalized_count >= 2:
            last_index = normalized_count - 1
            _safe_difference(
                typed_rows[last_index]["psi_rad"],
                typed_rows[0]["psi_rad"],
                code="NONFINITE_CLOSURE_PSI_DIFFERENCE",
                label="closure psi difference",
                line_number=typed_line_numbers[last_index],
                point_index=last_index,
                issues=issues,
            )
            _safe_difference(
                typed_rows[last_index]["kappa_radpm"],
                typed_rows[0]["kappa_radpm"],
                code="NONFINITE_CLOSURE_KAPPA_DIFFERENCE",
                label="closure kappa difference",
                line_number=typed_line_numbers[last_index],
                point_index=last_index,
                issues=issues,
            )
        max_abs_psi_difference = max(psi_differences, default=None)
        max_abs_kappa_difference = max(kappa_differences, default=None)

        lateral_accelerations: list[float] = []
        for index, row in enumerate(typed_rows[:normalized_count]):
            if limits.v_max_mps is not None and row["vx_mps"] > (
                limits.v_max_mps + limits.tolerance
            ):
                issues.append(
                    ValidationIssue(
                        code="V_MAX_EXCEEDED",
                        severity=Severity.ERROR,
                        message="velocity exceeds configured v_max",
                        line_number=typed_line_numbers[index],
                        point_index=index,
                        s_m=row["s_m"],
                        column="vx_mps",
                        value=row["vx_mps"],
                    )
                )
            if limits.a_max_mps2 is not None and row["ax_mps2"] > (
                limits.a_max_mps2 + limits.tolerance
            ):
                issues.append(
                    ValidationIssue(
                        code="A_MAX_EXCEEDED",
                        severity=Severity.ERROR,
                        message="acceleration exceeds configured a_max",
                        line_number=typed_line_numbers[index],
                        point_index=index,
                        s_m=row["s_m"],
                        column="ax_mps2",
                        value=row["ax_mps2"],
                    )
                )
            if limits.a_min_mps2 is not None and row["ax_mps2"] < (
                limits.a_min_mps2 - limits.tolerance
            ):
                issues.append(
                    ValidationIssue(
                        code="A_MIN_EXCEEDED",
                        severity=Severity.ERROR,
                        message="deceleration exceeds configured a_min",
                        line_number=typed_line_numbers[index],
                        point_index=index,
                        s_m=row["s_m"],
                        column="ax_mps2",
                        value=row["ax_mps2"],
                    )
                )
            abs_velocity = abs(row["vx_mps"])
            abs_kappa = abs(row["kappa_radpm"])
            lateral = (
                0.0
                if abs_velocity == 0.0 or abs_kappa == 0.0
                else abs_velocity * (abs_velocity * abs_kappa)
            )
            if not math.isfinite(lateral):
                issues.append(
                    ValidationIssue(
                        code="NONFINITE_LATERAL_ACCELERATION",
                        severity=Severity.ERROR,
                        message="derived lateral acceleration must be finite",
                        line_number=typed_line_numbers[index],
                        point_index=index,
                        s_m=row["s_m"],
                        value=lateral,
                    )
                )
                continue
            lateral_accelerations.append(lateral)
            if limits.ay_max_mps2 is not None and lateral > (
                limits.ay_max_mps2 + limits.tolerance
            ):
                issues.append(
                    ValidationIssue(
                        code="AY_MAX_EXCEEDED",
                        severity=Severity.ERROR,
                        message="lateral acceleration exceeds configured ay_max",
                        line_number=typed_line_numbers[index],
                        point_index=index,
                        s_m=row["s_m"],
                        value=lateral,
                    )
                )
        max_lateral_acceleration = max(lateral_accelerations, default=None)
    elif format_name == "pure_pursuit" and typed_rows:
        velocities = [row["speed"] for row in typed_rows[:normalized_count]]
        min_velocity = min(velocities, default=None)
        max_velocity = max(velocities, default=None)

    finite_total_distance = total_distance if math.isfinite(total_distance) else None
    metrics = TrajectoryMetrics(
        point_count=len(typed_rows),
        normalized_point_count=normalized_count,
        duplicate_endpoint=duplicate_endpoint,
        closure_distance_m=closure_distance,
        closing_edge_spacing_m=closing_edge_spacing,
        total_distance_m=finite_total_distance,
        min_spacing_m=min(spacings, default=None),
        max_spacing_m=max(spacings, default=None),
        mean_spacing_m=(
            finite_total_distance / len(spacings)
            if finite_total_distance is not None and spacings
            else None
        ),
        max_abs_psi_difference_rad=max_abs_psi_difference,
        max_abs_kappa_radpm=max_abs_kappa,
        max_abs_kappa_difference_radpm=max_abs_kappa_difference,
        min_velocity_mps=min_velocity,
        max_velocity_mps=max_velocity,
        min_acceleration_mps2=min_acceleration,
        max_acceleration_mps2=max_acceleration,
        max_lateral_acceleration_mps2=max_lateral_acceleration,
    )
    return ValidationReport(
        format_name=format_name,
        circular=circular,
        issues=tuple(sorted(issues, key=_issue_sort_key)),
        metrics=metrics,
    )


def validate_csv_file(
    path: Path,
    *,
    circular: bool,
    format_name: Optional[str] = None,
    limits: Optional[ValidationLimits] = None,
) -> ValidationReport:
    """Validate a CSV while retaining physical blank and malformed rows."""

    try:
        text = path.read_bytes().decode("utf-8")
    except (OSError, UnicodeError) as error:
        issue = ValidationIssue(
            code="FILE_READ_ERROR",
            severity=Severity.ERROR,
            message=str(error),
            line_number=1,
            value=str(path),
        )
        return ValidationReport(
            format_name=format_name or "unknown",
            circular=circular,
            issues=(issue,),
            metrics=_empty_metrics(),
        )
    if not text:
        issue = ValidationIssue(
            code="EMPTY_FILE",
            severity=Severity.ERROR,
            message="trajectory CSV is empty",
            line_number=1,
        )
        return ValidationReport(
            format_name=format_name or "unknown",
            circular=circular,
            issues=(issue,),
            metrics=_empty_metrics(),
        )

    # C++ uses std::getline(..., '\n'). Do not let Python split additional
    # Unicode or CR-only line separators that the runtime loader keeps in-field.
    lines = text.split("\n")
    if lines[-1] == "":
        lines.pop()

    fieldnames: list[object] = lines[0].split(",")
    initial_issues: list[ValidationIssue] = []
    if '"' in lines[0]:
        initial_issues.append(
            ValidationIssue(
                code="QUOTED_CSV_UNSUPPORTED",
                severity=Severity.ERROR,
                message="quoted CSV headers are not supported by the runtime loader",
                line_number=1,
                value=lines[0],
            )
        )
    if format_name is None:
        try:
            format_name = detect_trajectory_format(fieldnames)
        except ValueError as error:
            issue = ValidationIssue(
                code="UNKNOWN_FORMAT",
                severity=Severity.ERROR,
                message=str(error),
                line_number=1,
                value=lines[0],
            )
            return ValidationReport(
                format_name="unknown",
                circular=circular,
                issues=(issue,),
                metrics=_empty_metrics(),
            )

    rows: list[Mapping[object, object]] = []
    line_numbers: list[int] = []
    for line_number, line in enumerate(lines[1:], start=2):
        if not line.strip():
            initial_issues.append(
                ValidationIssue(
                    code="BLANK_DATA_ROW",
                    severity=Severity.ERROR,
                    message="blank data rows are not allowed",
                    line_number=line_number,
                    value="<blank>",
                )
            )
            continue
        if '"' in line:
            initial_issues.append(
                ValidationIssue(
                    code="QUOTED_CSV_UNSUPPORTED",
                    severity=Severity.ERROR,
                    message="quoted CSV fields are not supported by the runtime loader",
                    line_number=line_number,
                    value=line,
                )
            )
        fields = line.split(",")
        row: dict[object, object] = {
            header: fields[index] if index < len(fields) else None
            for index, header in enumerate(fieldnames)
        }
        if len(fields) > len(fieldnames):
            row[None] = fields[len(fieldnames):]
        rows.append(row)
        line_numbers.append(line_number)

    return validate_trajectory(
        fieldnames,
        rows,
        format_name,
        circular,
        limits,
        line_numbers=line_numbers,
        initial_issues=initial_issues,
    )


def _new_file_mode() -> int:
    current_umask = os.umask(0)
    os.umask(current_umask)
    return 0o666 & ~current_umask


def atomic_write_csv(
    path: Path,
    fieldnames: Sequence[str],
    rows: Sequence[Mapping[object, object]],
    *,
    validate_path: Optional[Callable[[Path], None]] = None,
) -> None:
    """Atomically write a CSV without replacing a symlink or changing mode."""

    target = Path(path)
    if target.is_symlink():
        raise ValueError(
            f"refusing to overwrite symlink {target}; use Save As with a regular file"
        )
    parent = target.parent.resolve(strict=True)
    resolved_target = parent / target.name
    target_mode = (
        stat.S_IMODE(resolved_target.stat().st_mode)
        if resolved_target.exists()
        else _new_file_mode()
    )

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=parent, text=True
    )
    temporary = Path(temporary_name)
    descriptor_open = True
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            descriptor_open = False
            writer = csv.DictWriter(
                stream,
                fieldnames=list(fieldnames),
                extrasaction="raise",
                lineterminator="\n",
            )
            writer.writeheader()
            writer.writerows(rows)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, target_mode)
        if validate_path is not None:
            validate_path(temporary)
        os.replace(temporary, resolved_target)
    except Exception:
        if descriptor_open:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        raise
