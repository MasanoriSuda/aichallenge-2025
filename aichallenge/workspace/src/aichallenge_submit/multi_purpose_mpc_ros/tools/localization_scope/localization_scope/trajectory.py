"""Reference trajectory loading and pose projection."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Iterable

from .models import Projection
from .models import TrajectoryPoint


class TrajectoryError(ValueError):
    """Raised for invalid or unsupported trajectory CSV input."""


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def load_trajectory_csv(path: Path) -> list[TrajectoryPoint]:
    if not path.is_file():
        raise TrajectoryError(f"trajectory CSV does not exist: {path}")
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                raise TrajectoryError("trajectory CSV has no header")
            required = {"x_m", "y_m"}
            missing = required.difference(reader.fieldnames)
            if missing:
                raise TrajectoryError(
                    f"trajectory CSV is missing columns: {', '.join(sorted(missing))}"
                )
            raw_rows = list(reader)
    except OSError as error:
        raise TrajectoryError(f"failed to read trajectory CSV: {error}") from error

    if len(raw_rows) < 2:
        raise TrajectoryError("trajectory CSV requires at least two rows")

    xy: list[tuple[float, float]] = []
    for index, row in enumerate(raw_rows):
        try:
            x = float(row["x_m"])
            y = float(row["y_m"])
        except (TypeError, ValueError) as error:
            raise TrajectoryError(f"row {index + 2} has invalid x_m/y_m") from error
        if not math.isfinite(x) or not math.isfinite(y):
            raise TrajectoryError(f"row {index + 2} has non-finite x_m/y_m")
        xy.append((x, y))

    cumulative = [0.0]
    for left, right in zip(xy, xy[1:]):
        cumulative.append(cumulative[-1] + math.hypot(right[0] - left[0], right[1] - left[1]))

    points: list[TrajectoryPoint] = []
    for index, (row, (x, y)) in enumerate(zip(raw_rows, xy)):
        next_index = min(index + 1, len(xy) - 1)
        previous_index = max(index - 1, 0)
        geometry_yaw = math.atan2(
            xy[next_index][1] - xy[previous_index][1],
            xy[next_index][0] - xy[previous_index][0],
        )

        def optional_float(key: str) -> float | None:
            raw = row.get(key)
            if raw in (None, ""):
                return None
            try:
                value = float(raw)
            except ValueError as error:
                raise TrajectoryError(f"row {index + 2} has invalid {key}") from error
            return value if math.isfinite(value) else None

        supplied_s = optional_float("s_m")
        supplied_yaw = optional_float("psi_rad")
        points.append(
            TrajectoryPoint(
                s_m=supplied_s if supplied_s is not None else cumulative[index],
                x_m=x,
                y_m=y,
                yaw_rad=supplied_yaw if supplied_yaw is not None else geometry_yaw,
                curvature_radpm=optional_float("kappa_radpm"),
                velocity_mps=optional_float("vx_mps"),
            )
        )
    return points


def project_pose(
    trajectory: list[TrajectoryPoint],
    x_m: float,
    y_m: float,
    yaw_rad: float | None = None,
    *,
    circular: bool = False,
) -> Projection:
    if len(trajectory) < 2:
        raise TrajectoryError("projection requires at least two trajectory points")

    segments = list(zip(trajectory, trajectory[1:]))
    if circular:
        segments.append((trajectory[-1], trajectory[0]))
    best: tuple[float, float, float, float, float, int] | None = None
    for index, (left, right) in enumerate(segments):
        dx = right.x_m - left.x_m
        dy = right.y_m - left.y_m
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            continue
        fraction = max(
            0.0,
            min(1.0, ((x_m - left.x_m) * dx + (y_m - left.y_m) * dy) / length_squared),
        )
        px = left.x_m + fraction * dx
        py = left.y_m + fraction * dy
        error_x = x_m - px
        error_y = y_m - py
        distance_squared = error_x * error_x + error_y * error_y
        yaw_delta = wrap_angle(right.yaw_rad - left.yaw_rad)
        reference_yaw = wrap_angle(left.yaw_rad + fraction * yaw_delta)
        heading_penalty = 0.0
        if yaw_rad is not None:
            heading_delta = wrap_angle(yaw_rad - reference_yaw)
            heading_penalty = 0.04 * (1.0 - math.cos(heading_delta))
        score = distance_squared + heading_penalty
        if best is None or score < best[0]:
            cross_track = (dx * error_y - dy * error_x) / math.sqrt(length_squared)
            if circular and index == len(trajectory) - 1:
                s_m = left.s_m + fraction * math.sqrt(length_squared)
            else:
                s_m = left.s_m + fraction * (right.s_m - left.s_m)
            best = (
                score,
                distance_squared,
                cross_track,
                s_m,
                reference_yaw,
                index,
            )

    if best is None:
        raise TrajectoryError("trajectory has no non-degenerate segments")
    _, distance_squared, cross_track, s_m, reference_yaw, segment_index = best
    yaw_error = wrap_angle(yaw_rad - reference_yaw) if yaw_rad is not None else None
    return Projection(
        s_m=s_m,
        cross_track_m=cross_track,
        distance_m=math.sqrt(distance_squared),
        reference_yaw_rad=reference_yaw,
        yaw_error_rad=yaw_error,
        segment_index=segment_index,
    )


def trajectory_xy(
    points: Iterable[TrajectoryPoint], *, circular: bool = False
) -> list[list[float]]:
    values = [[point.x_m, point.y_m] for point in points]
    if circular and values and values[-1] != values[0]:
        values.append(values[0])
    return values
