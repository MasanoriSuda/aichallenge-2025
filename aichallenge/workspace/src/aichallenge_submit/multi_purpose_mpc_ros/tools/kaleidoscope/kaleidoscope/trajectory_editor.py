#!/usr/bin/env python3
"""Tkinter editor for trajectory CSV files over Lanelet2 OSM rails."""

from __future__ import annotations

import argparse
from concurrent.futures import CancelledError
import copy
import csv
from dataclasses import dataclass
from dataclasses import field
import math
from pathlib import Path
import threading
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
from tkinter import ttk
from typing import Callable, Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET

from .trajectory_clearance import AdjustmentParameters
from .trajectory_clearance import AdjustmentResult
from .trajectory_clearance import AdjustmentStatus
from .trajectory_clearance import ClearanceReport
from .trajectory_clearance import MapLoadOptions
from .trajectory_clearance import OccupancyGrid
from .trajectory_clearance import Pose2D
from .trajectory_clearance import ValidationOptions
from .trajectory_clearance import VehicleFootprintSpec
from .trajectory_clearance import adjust_clearance
from .trajectory_clearance import footprint_polygon
from .trajectory_clearance import load_occupancy_grid
from .trajectory_clearance import validate_clearance
from .trajectory_clearance_dialog import ClearanceDialogConfig
from .trajectory_clearance_dialog import ask_clearance_settings
from .trajectory_clearance_dialog import clearance_report_summary
from .trajectory_clearance_dialog import provisional_default_config
from .trajectory_clearance_dialog import show_clearance_report
from .trajectory_contract import CLOSURE_TOLERANCE_M
from .trajectory_contract import MPC_COLUMNS
from .trajectory_contract import PURE_PURSUIT_COLUMNS
from .trajectory_contract import Severity
from .trajectory_contract import TrajectoryData
from .trajectory_contract import ValidationIssue
from .trajectory_contract import ValidationReport
from .trajectory_contract import atomic_write_csv
from .trajectory_contract import validate_csv_file
from .trajectory_contract import validate_trajectory
from .trajectory_plot import build_comparison_plot
from .trajectory_preview import LOCAL_A_MAX_MPS2
from .trajectory_preview import ask_normalize_options
from .trajectory_preview import ask_speed_parameters
from .trajectory_preview import preview_candidate
from .trajectory_preview import run_candidate_task
from .trajectory_processing import MetadataMode
from .trajectory_processing import normalize_geometry
from .trajectory_speed import SpeedProfileParameters
from .trajectory_speed import recompute_speed_profile


Point = Tuple[float, float]
_ASCII_WHITESPACE = " \t\n\r\v\f"
_MAX_EDITOR_DRAW_POINTS = 6_000
_NORMALIZATION_REPAIRABLE_CODES = frozenset(
    {
        "NON_INCREASING_S",
        "DUPLICATE_POINT",
        "DEGENERATE_SEGMENT",
        "DUPLICATE_CLOSING_POINT",
        "DEGENERATE_CLOSING_SEGMENT",
    }
)


@dataclass
class UndoState:
    rows: List[Dict[str, str]]
    points: List[Point]
    selected_index: Optional[int]
    dirty: bool
    geometry_dirty: bool
    speed_dirty: bool
    last_operation: str


@dataclass(frozen=True)
class OriginalDifference:
    original_point_count: int
    working_point_count: int
    original_length_m: float
    working_length_m: float
    maximum_displacement_m: float
    mean_displacement_m: float
    changed_ranges_m: Tuple[Tuple[float, float], ...]
    changed_indices: Tuple[int, ...]


def _display_arc_lengths(points: Sequence[Point]) -> Tuple[float, ...]:
    if not points:
        return ()
    values = [0.0]
    for first, second in zip(points, points[1:]):
        values.append(values[-1] + math.hypot(second[0] - first[0], second[1] - first[1]))
    return tuple(values)


def _interpolate_point_at_s(
    points: Sequence[Point],
    arc_lengths: Sequence[float],
    s_m: float,
) -> Point:
    if not points:
        raise ValueError("cannot interpolate an empty trajectory")
    if len(points) == 1 or s_m <= arc_lengths[0]:
        return points[0]
    if s_m >= arc_lengths[-1]:
        return points[-1]
    low = 0
    high = len(arc_lengths) - 1
    while low + 1 < high:
        middle = (low + high) // 2
        if arc_lengths[middle] <= s_m:
            low = middle
        else:
            high = middle
    span = arc_lengths[high] - arc_lengths[low]
    if span <= 1e-12:
        return points[low]
    factor = (s_m - arc_lengths[low]) / span
    return (
        points[low][0] + factor * (points[high][0] - points[low][0]),
        points[low][1] + factor * (points[high][1] - points[low][1]),
    )


def build_original_difference(
    original_points: Sequence[Point],
    working_points: Sequence[Point],
    *,
    change_threshold_m: float = 0.01,
) -> OriginalDifference:
    if not math.isfinite(change_threshold_m) or change_threshold_m < 0.0:
        raise ValueError("change threshold must be finite and non-negative")
    original_arc = _display_arc_lengths(original_points)
    working_arc = _display_arc_lengths(working_points)
    displacements: list[float] = []
    changed_indices: list[int] = []
    if original_points:
        for index, (point, s_m) in enumerate(zip(working_points, working_arc)):
            reference = _interpolate_point_at_s(original_points, original_arc, s_m)
            displacement = math.hypot(point[0] - reference[0], point[1] - reference[1])
            displacements.append(displacement)
            if displacement > change_threshold_m:
                changed_indices.append(index)
    elif working_points:
        displacements = [math.inf] * len(working_points)
        changed_indices = list(range(len(working_points)))

    changed_ranges: list[Tuple[float, float]] = []
    if changed_indices:
        start = previous = changed_indices[0]
        for index in changed_indices[1:]:
            if index != previous + 1:
                changed_ranges.append((working_arc[start], working_arc[previous]))
                start = index
            previous = index
        changed_ranges.append((working_arc[start], working_arc[previous]))
    finite_displacements = [value for value in displacements if math.isfinite(value)]
    maximum = max(displacements, default=0.0)
    mean = (
        sum(finite_displacements) / len(finite_displacements)
        if finite_displacements and len(finite_displacements) == len(displacements)
        else maximum
    )
    return OriginalDifference(
        original_point_count=len(original_points),
        working_point_count=len(working_points),
        original_length_m=original_arc[-1] if original_arc else 0.0,
        working_length_m=working_arc[-1] if working_arc else 0.0,
        maximum_displacement_m=maximum,
        mean_displacement_m=mean,
        changed_ranges_m=tuple(changed_ranges),
        changed_indices=tuple(changed_indices),
    )


def _trajectory_content_signature(data: TrajectoryData) -> object:
    """Bind a validation result to the exact mutable candidate content."""

    return (
        tuple(data.fieldnames),
        data.x_column,
        data.y_column,
        data.format_name,
        tuple(data.points),
        tuple(
            tuple(
                sorted(
                    ((repr(key), repr(value)) for key, value in row.items()),
                    key=lambda item: item[0],
                )
            )
            for row in data.rows
        ),
    )


def _trajectory_geometry_signature(data: TrajectoryData) -> object:
    """Identify the geometry fields relevant to a clearance result."""

    geometry_columns = (
        ("s_m", "x_m", "y_m", "psi_rad", "kappa_radpm")
        if data.format_name == "mpc"
        else tuple(data.fieldnames)
    )
    return (
        data.format_name,
        tuple(data.points),
        tuple(
            tuple(row.get(column) for column in geometry_columns)
            for row in data.rows
        ),
    )


@dataclass
class EditorCandidate:
    source_revision: int
    operation: str
    trajectory: TrajectoryData
    validation: ValidationReport
    transformation: object
    parameters: Dict[str, object]
    suggested_suffix: str
    geometry_dirty: bool
    speed_dirty: bool
    plot_data: Optional[object] = None
    apply_guard: Optional[
        Callable[[TrajectoryData], Tuple[bool, str, Optional[object]]]
    ] = field(default=None, repr=False)
    safety_payload: Optional[object] = field(default=None, repr=False)
    content_signature: object = field(init=False, repr=False)

    def __post_init__(self) -> None:
        self.content_signature = _trajectory_content_signature(self.trajectory)


def _package_share(package_name: str) -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception:  # noqa: BLE001
        return None

    try:
        return Path(get_package_share_directory(package_name))
    except Exception:  # noqa: BLE001
        return None


def _first_existing(paths: Sequence[Path]) -> Optional[Path]:
    for path in paths:
        if path.exists():
            return path
    return None


def _default_paths(preset: str = "mpc") -> Tuple[Optional[Path], Optional[Path]]:
    mpc_candidates: List[Path] = []
    pure_pursuit_candidates: List[Path] = []
    osm_candidates: List[Path] = []

    mpc_share = _package_share("multi_purpose_mpc_ros")
    if mpc_share is not None:
        mpc_candidates.append(mpc_share / "env" / "final_ver3" / "traj_mincurv.csv")

    pure_pursuit_share = _package_share("simple_trajectory_generator")
    if pure_pursuit_share is not None:
        pure_pursuit_candidates.append(
            pure_pursuit_share / "data" / "raceline_awsim_30km_from_garage.csv"
        )

    launch_share = _package_share("aichallenge_submit_launch")
    if launch_share is not None:
        osm_candidates.append(launch_share / "map" / "lanelet2_map.osm")

    source = Path(__file__).resolve()
    for parent in source.parents:
        mpc_candidates.append(parent / "env" / "final_ver3" / "traj_mincurv.csv")
        pure_pursuit_candidates.append(
            parent.parent
            / "simple_trajectory_generator"
            / "data"
            / "raceline_awsim_30km_from_garage.csv"
        )
        osm_candidates.append(
            parent.parent / "aichallenge_submit_launch" / "map" / "lanelet2_map.osm"
        )

    trajectory_candidates = (
        pure_pursuit_candidates if preset == "pure_pursuit" else mpc_candidates
    )
    return _first_existing(trajectory_candidates), _first_existing(osm_candidates)


def _default_occupancy_grid_path(trajectory_path: Path) -> Optional[Path]:
    """Find the map paired with a trajectory without changing runtime config."""

    candidates = [Path(trajectory_path).parent / "occupancy_grid_map.yaml"]
    package_share = _package_share("multi_purpose_mpc_ros")
    if package_share is not None:
        candidates.append(
            package_share / "env" / "final_ver3" / "occupancy_grid_map.yaml"
        )
    source = Path(__file__).resolve()
    for parent in source.parents:
        candidates.append(
            parent / "env" / "final_ver3" / "occupancy_grid_map.yaml"
        )
    return _first_existing(candidates)


def _tags(element: ET.Element) -> Dict[str, str]:
    return {
        tag.attrib.get("k", ""): tag.attrib.get("v", "")
        for tag in element.findall("tag")
    }


def load_osm_rails(path: Path) -> List[List[Point]]:
    tree = ET.parse(path)
    root = tree.getroot()

    nodes: Dict[str, Point] = {}
    for node in root.findall("node"):
        tag_map = _tags(node)
        x = tag_map.get("local_x")
        y = tag_map.get("local_y")
        if x is None or y is None:
            x = node.attrib.get("lon")
            y = node.attrib.get("lat")
        if x is None or y is None:
            continue
        try:
            nodes[node.attrib["id"]] = (float(x), float(y))
        except (KeyError, ValueError):
            continue

    ways: Dict[str, List[Point]] = {}
    for way in root.findall("way"):
        coords: List[Point] = []
        for nd in way.findall("nd"):
            point = nodes.get(nd.attrib.get("ref", ""))
            if point is not None:
                coords.append(point)
        if len(coords) >= 2:
            ways[way.attrib.get("id", "")] = coords

    lanelet_way_ids: List[str] = []
    seen = set()
    for relation in root.findall("relation"):
        tag_map = _tags(relation)
        if tag_map.get("type") != "lanelet":
            continue
        for member in relation.findall("member"):
            role = member.attrib.get("role")
            ref = member.attrib.get("ref", "")
            if role in ("left", "right") and ref in ways and ref not in seen:
                lanelet_way_ids.append(ref)
                seen.add(ref)

    if lanelet_way_ids:
        return [ways[way_id] for way_id in lanelet_way_ids]
    return list(ways.values())


def _detect_trajectory_columns(fieldnames: Sequence[str], path: Path) -> Tuple[str, str, str]:
    field_set = set(fieldnames)
    if {"x_m", "y_m"}.issubset(field_set):
        return "x_m", "y_m", "mpc"
    if {"x", "y"}.issubset(field_set):
        return "x", "y", "pure_pursuit"
    raise ValueError(
        f"{path} must contain either x_m/y_m columns or x/y columns"
    )


def load_trajectory(path: Path) -> TrajectoryData:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        raw_fieldnames = list(reader.fieldnames)
        fieldnames = []
        for index, raw_header in enumerate(raw_fieldnames):
            header = (
                ""
                if raw_header is None
                else raw_header.strip(_ASCII_WHITESPACE)
            )
            if index == 0 and header.startswith("\ufeff"):
                header = header[1:].strip(_ASCII_WHITESPACE)
            fieldnames.append(header)
        x_column, y_column, format_name = _detect_trajectory_columns(fieldnames, path)

        rows: List[Dict[str, str]] = []
        points: List[Point] = []
        for raw_row in reader:
            row: Dict[object, object] = {}
            for raw_header, header in zip(raw_fieldnames, fieldnames):
                row[header] = raw_row.get(raw_header)
            if None in raw_row:
                row[None] = raw_row[None]
            try:
                x = float(row[x_column])
                y = float(row[y_column])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"Invalid {x_column}/{y_column} row in {path}") from exc
            if not math.isfinite(x) or not math.isfinite(y):
                raise ValueError(f"Non-finite {x_column}/{y_column} row in {path}")
            rows.append(dict(row))  # type: ignore[arg-type]
            points.append((x, y))

    if len(points) < 2:
        raise ValueError(f"{path} must contain at least two trajectory points")
    return TrajectoryData(
        path=path,
        fieldnames=fieldnames,
        rows=rows,
        points=points,
        x_column=x_column,
        y_column=y_column,
        format_name=format_name,
    )


def _distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _display_indices(count: int, limit: int = _MAX_EDITOR_DRAW_POINTS) -> List[int]:
    if count <= 0:
        return []
    step = max(1, int(math.ceil(count / limit)))
    indices = list(range(0, count, step))
    if indices[-1] != count - 1:
        indices.append(count - 1)
    return indices


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _closed_duplicate(points: Sequence[Point]) -> bool:
    return (
        len(points) > 2
        and _distance(points[0], points[-1]) <= CLOSURE_TOLERANCE_M
    )


def _load_editor_trajectory(
    path: Path,
    *,
    circular: Optional[bool],
) -> Tuple[TrajectoryData, bool, ValidationReport]:
    """Load the compatibility model and retain the raw runtime contract check."""

    data = load_trajectory(path)
    topology = _closed_duplicate(data.points) if circular is None else circular
    report = validate_csv_file(
        path,
        circular=bool(topology),
        format_name=data.format_name,
    )
    return data, bool(topology), report


def _validation_failure_message(path: Path, report: ValidationReport) -> str:
    errors = [issue for issue in report.issues if issue.severity is Severity.ERROR]
    details = "; ".join(
        f"{issue.code} (line {issue.line_number or '?' }): {issue.message}"
        for issue in errors[:5]
    )
    if len(errors) > 5:
        details += f"; ... {len(errors) - 5} more error(s)"
    return f"{path}: trajectory validation failed: {details}"


def _is_normalization_repairable(
    data: TrajectoryData,
    report: ValidationReport,
) -> bool:
    """Whether the strict MPC rows are safe to open only for normalization."""

    errors = [
        issue
        for issue in report.issues
        if issue.severity is Severity.ERROR
    ]
    repairable = all(
        issue.code in _NORMALIZATION_REPAIRABLE_CODES
        or (
            issue.code == "INVALID_NUMBER"
            and issue.column in {"s_m", "psi_rad", "kappa_radpm"}
        )
        for issue in errors
    )
    return (
        data.format_name == "mpc"
        and bool(errors)
        and repairable
    )


def recompute_geometry(
    data: TrajectoryData, *, circular: Optional[bool] = None
) -> None:
    points = data.points
    n_points = len(points)
    if n_points < 2:
        return

    s_values = [0.0] * n_points
    for i in range(1, n_points):
        s_values[i] = s_values[i - 1] + _distance(points[i - 1], points[i])

    duplicate_endpoint = _closed_duplicate(points)
    closed = duplicate_endpoint if circular is None else circular
    unique_count = n_points - (1 if closed and duplicate_endpoint else 0)
    psi_values = [0.0] * n_points
    kappa_values = [0.0] * n_points

    for i in range(unique_count):
        prev_i = (i - 1) % unique_count if closed else max(0, i - 1)
        next_i = (i + 1) % unique_count if closed else min(unique_count - 1, i + 1)
        px, py = points[prev_i]
        x, y = points[i]
        nx, ny = points[next_i]

        if prev_i == i:
            dx = nx - x
            dy = ny - y
        elif next_i == i:
            dx = x - px
            dy = y - py
        else:
            dx = nx - px
            dy = ny - py
        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
            psi_values[i] = math.atan2(dy, dx)

        d1 = math.hypot(x - px, y - py)
        d2 = math.hypot(nx - x, ny - y)
        if d1 > 1e-6 and d2 > 1e-6:
            h1 = math.atan2(y - py, x - px)
            h2 = math.atan2(ny - y, nx - x)
            kappa_values[i] = _wrap_angle(h2 - h1) / (0.5 * (d1 + d2))

    if closed and duplicate_endpoint:
        psi_values[-1] = psi_values[0]
        kappa_values[-1] = kappa_values[0]

    for row, point, s_value, psi, kappa in zip(
        data.rows, points, s_values, psi_values, kappa_values
    ):
        if "s_m" in row:
            row["s_m"] = f"{s_value:.7f}"
        row[data.x_column] = f"{point[0]:.7f}"
        row[data.y_column] = f"{point[1]:.7f}"
        if "psi_rad" in row:
            row["psi_rad"] = f"{psi:.7f}"
        if "kappa_radpm" in row:
            row["kappa_radpm"] = f"{kappa:.7f}"
        if {"x_quat", "y_quat", "z_quat", "w_quat"}.issubset(row):
            row["x_quat"] = "0.0"
            row["y_quat"] = "0.0"
            row["z_quat"] = f"{math.sin(0.5 * psi):.16g}"
            row["w_quat"] = f"{math.cos(0.5 * psi):.16g}"


def _copy_with_current_points(
    data: TrajectoryData,
    *,
    recompute: bool = False,
    circular: Optional[bool] = None,
) -> TrajectoryData:
    candidate = copy.deepcopy(data)
    if recompute:
        recompute_geometry(candidate, circular=circular)
    else:
        for row, point in zip(candidate.rows, candidate.points):
            row[candidate.x_column] = f"{point[0]:.7f}"
            row[candidate.y_column] = f"{point[1]:.7f}"
    return candidate


def _canonical_rows(data: TrajectoryData) -> Tuple[List[str], List[Dict[str, str]]]:
    output_columns = (
        MPC_COLUMNS if data.format_name == "mpc" else PURE_PURSUIT_COLUMNS
    )
    normalized_headers: Dict[str, str] = {}
    for index, raw_header in enumerate(data.fieldnames):
        normalized = str(raw_header).strip(_ASCII_WHITESPACE)
        if index == 0 and normalized.startswith("\ufeff"):
            normalized = normalized[1:].strip(_ASCII_WHITESPACE)
        normalized_headers[normalized] = raw_header

    output_rows: List[Dict[str, str]] = []
    for row in data.rows:
        output_rows.append(
            {
                column: row[normalized_headers[column]]
                for column in output_columns
            }
        )
    return list(output_columns), output_rows


def validate_trajectory_data(
    data: TrajectoryData, *, circular: bool
) -> ValidationReport:
    candidate = _copy_with_current_points(data, recompute=False)
    return validate_trajectory(
        candidate.fieldnames,
        candidate.rows,
        candidate.format_name,
        circular,
    )


def save_trajectory(
    data: TrajectoryData,
    path: Path,
    recompute: bool = False,
    *,
    circular: Optional[bool] = None,
) -> ValidationReport:
    """Validate and atomically save without mutating ``data`` on failure."""

    is_circular = _closed_duplicate(data.points) if circular is None else circular
    candidate = _copy_with_current_points(
        data, recompute=recompute, circular=is_circular
    )
    report = validate_trajectory_data(candidate, circular=is_circular)
    if not report.is_valid:
        summary = "; ".join(
            f"{issue.code}: {issue.message}"
            for issue in report.issues
            if issue.severity is Severity.ERROR
        )
        raise ValueError(f"trajectory validation failed: {summary}")

    output_columns, output_rows = _canonical_rows(candidate)

    def validate_temporary(temporary_path: Path) -> None:
        temporary_report = validate_csv_file(
            temporary_path,
            circular=is_circular,
            format_name=candidate.format_name,
        )
        if not temporary_report.is_valid:
            raise ValueError("serialized trajectory failed final validation")

    atomic_write_csv(
        Path(path),
        output_columns,
        output_rows,
        validate_path=validate_temporary,
    )

    data.path = Path(path)
    data.fieldnames = output_columns
    data.rows = output_rows
    data.points = list(candidate.points)
    return report


class TrajectoryEditor(tk.Tk):
    def __init__(
        self,
        trajectory_path: Path,
        osm_path: Path,
        *,
        circular: Optional[bool] = None,
        circular_explicit: bool = False,
    ) -> None:
        trajectory, initial_topology, initial_report = _load_editor_trajectory(
            trajectory_path,
            circular=circular,
        )
        initial_repairable = _is_normalization_repairable(
            trajectory,
            initial_report,
        )
        if not initial_report.is_valid and not initial_repairable:
            raise ValueError(_validation_failure_message(trajectory_path, initial_report))
        rails = load_osm_rails(osm_path)

        super().__init__()
        self.title("Trajectory Editor")
        self.geometry("1200x800")

        self.trajectory_path = trajectory_path
        self.osm_path = osm_path
        self.trajectory = trajectory
        # The loaded source is a session-level reference, not a save baseline.
        # Keep ``original_trajectory`` as a compatibility alias for downstream
        # imports/tests, but never replace either snapshot on Save / Save As.
        self.loaded_original = copy.deepcopy(trajectory)
        self.original_trajectory = self.loaded_original
        self.loaded_original_circular = bool(initial_topology)
        self.original_difference = build_original_difference(
            self.loaded_original.points,
            self.trajectory.points,
        )
        self.rails = rails

        self.center_x = 0.0
        self.center_y = 0.0
        self.scale = 5.0
        self.selected_index: Optional[int] = None
        self.dragging_point = False
        self.drag_undo_saved = False
        self.panning = False
        self.pan_anchor = (0, 0)
        self.undo_stack: List[UndoState] = []
        self.circular_override = circular if circular_explicit else None
        self.circular = tk.BooleanVar(value=initial_topology)
        self.show_original = tk.BooleanVar(value=True)
        self.show_working = tk.BooleanVar(value=True)
        self.show_candidate = tk.BooleanVar(value=True)
        self.dirty = False
        self.geometry_dirty = initial_repairable
        self.speed_dirty = initial_repairable
        self.last_operation = "edited"
        self.candidate: Optional[EditorCandidate] = None
        self.revision = 0
        self.validation_report: Optional[ValidationReport] = None
        self.validation_revision: Optional[int] = None
        self.validation_external = False
        self.validation_issues_by_iid: Dict[str, ValidationIssue] = {}
        self.clearance_config: Optional[ClearanceDialogConfig] = None
        self.clearance_grid: Optional[OccupancyGrid] = None
        self.clearance_report: Optional[ClearanceReport] = None
        self.clearance_revision: Optional[int] = None
        self.clearance_state = "not_run"
        self.clearance_selected_issue: Optional[object] = None
        self.clearance_report_window: Optional[tk.Toplevel] = None
        self.influence_radius_points = tk.IntVar(value=4)
        self.smooth_alpha = tk.DoubleVar(value=0.15)
        self.smooth_passes = tk.IntVar(value=1)
        self.drag_origin_points: Optional[List[Point]] = None

        self._build_ui()
        self._bind_events()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.fit_view()
        self.after_idle(self.fit_view)
        self.validate_current()

    def _build_ui(self) -> None:
        toolbar = tk.Frame(self)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        tk.Button(toolbar, text="Open Traj", command=self.open_trajectory).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Open OSM", command=self.open_osm).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Validate", command=self.validate_current).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(
            toolbar,
            text="Normalize Geometry",
            command=self.normalize_geometry_candidate,
        ).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Button(
            toolbar,
            text="Recompute Speed",
            command=self.recompute_speed_candidate,
        ).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Button(toolbar, text="Save As", command=self.save_as).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Overwrite", command=self.save).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Undo", command=self.undo).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Fit", command=self.fit_view).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(
            toolbar,
            text="Center Selection",
            command=self.center_selection,
        ).pack(side=tk.LEFT, padx=2, pady=2)

        tuning_toolbar = tk.Frame(self)
        tuning_toolbar.pack(side=tk.TOP, fill=tk.X)
        tk.Button(
            tuning_toolbar,
            text="Smooth All",
            command=self.smooth_all_points,
        ).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(
            tuning_toolbar,
            text="Recompute Geometry",
            command=self.recompute_derived_geometry,
        ).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Label(tuning_toolbar, text="Smooth").pack(side=tk.LEFT, padx=(8, 2))
        tk.Spinbox(
            tuning_toolbar,
            from_=0.01,
            to=0.50,
            increment=0.01,
            width=5,
            format="%.2f",
            textvariable=self.smooth_alpha,
        ).pack(side=tk.LEFT, padx=2)
        tk.Label(tuning_toolbar, text="Passes").pack(side=tk.LEFT, padx=(8, 2))
        tk.Spinbox(
            tuning_toolbar,
            from_=1,
            to=10,
            width=3,
            textvariable=self.smooth_passes,
        ).pack(side=tk.LEFT, padx=2)
        tk.Label(tuning_toolbar, text="Influence pts").pack(
            side=tk.LEFT, padx=(8, 2)
        )
        tk.Spinbox(
            tuning_toolbar,
            from_=0,
            to=30,
            width=4,
            textvariable=self.influence_radius_points,
            command=self.redraw,
        ).pack(side=tk.LEFT, padx=2)
        tk.Checkbutton(
            tuning_toolbar,
            text="Circular",
            variable=self.circular,
            command=self._on_circular_changed,
        ).pack(side=tk.LEFT, padx=8)

        clearance_toolbar = tk.Frame(self)
        clearance_toolbar.pack(side=tk.TOP, fill=tk.X)
        tk.Label(clearance_toolbar, text="Wall clearance:").pack(
            side=tk.LEFT, padx=(4, 2), pady=2
        )
        tk.Button(
            clearance_toolbar,
            text="Vehicle / Margin Settings",
            command=self.configure_clearance,
        ).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Button(
            clearance_toolbar,
            text="Validate Clearance",
            command=self.validate_clearance_action,
        ).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Button(
            clearance_toolbar,
            text="Adjust Clearance",
            command=self.adjust_clearance_action,
        ).pack(side=tk.LEFT, padx=2, pady=2)
        self.clearance_summary = tk.StringVar(value="Clearance: not run")
        tk.Label(
            clearance_toolbar,
            textvariable=self.clearance_summary,
            anchor="w",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 4), pady=2)

        view_toolbar = tk.Frame(self)
        view_toolbar.pack(side=tk.TOP, fill=tk.X)
        tk.Label(view_toolbar, text="Layers:").pack(
            side=tk.LEFT, padx=(4, 2), pady=2
        )
        tk.Checkbutton(
            view_toolbar,
            text="Original (gray dashed)",
            variable=self.show_original,
            command=self._on_layer_visibility_changed,
            foreground="#6f7782",
        ).pack(side=tk.LEFT, padx=4, pady=2)
        tk.Checkbutton(
            view_toolbar,
            text="Working (blue solid)",
            variable=self.show_working,
            command=self._on_layer_visibility_changed,
            foreground="#1976c5",
        ).pack(side=tk.LEFT, padx=4, pady=2)
        tk.Checkbutton(
            view_toolbar,
            text="Candidate (orange dashed, when available)",
            variable=self.show_candidate,
            command=self._on_layer_visibility_changed,
            foreground="#c56800",
        ).pack(side=tk.LEFT, padx=4, pady=2)
        self.original_difference_summary = tk.StringVar(
            value="Original diff: unchanged"
        )
        tk.Label(
            view_toolbar,
            textvariable=self.original_difference_summary,
            anchor="w",
        ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 4), pady=2)

        self.status = tk.StringVar()
        tk.Label(self, textvariable=self.status, anchor="w").pack(
            side=tk.BOTTOM, fill=tk.X
        )

        validation_frame = tk.Frame(self)
        validation_frame.pack(side=tk.BOTTOM, fill=tk.X)
        self.validation_summary = tk.StringVar(value="Not validated")
        tk.Label(
            validation_frame,
            textvariable=self.validation_summary,
            anchor="w",
        ).pack(side=tk.TOP, fill=tk.X)

        issue_columns = ("severity", "code", "line", "s_m", "value", "message")
        self.issue_tree = ttk.Treeview(
            validation_frame,
            columns=issue_columns,
            show="headings",
            height=6,
            selectmode="browse",
        )
        column_widths = {
            "severity": 70,
            "code": 190,
            "line": 55,
            "s_m": 90,
            "value": 110,
            "message": 520,
        }
        for column in issue_columns:
            self.issue_tree.heading(column, text=column)
            self.issue_tree.column(
                column,
                width=column_widths[column],
                stretch=column == "message",
            )
        issue_scrollbar = ttk.Scrollbar(
            validation_frame,
            orient=tk.VERTICAL,
            command=self.issue_tree.yview,
        )
        self.issue_tree.configure(yscrollcommand=issue_scrollbar.set)
        issue_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.issue_tree.pack(side=tk.LEFT, fill=tk.X, expand=True)

        canvas_frame = ttk.Frame(self)
        canvas_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        canvas_frame.rowconfigure(0, weight=1)
        canvas_frame.columnconfigure(0, weight=1)
        self.canvas = tk.Canvas(canvas_frame, background="#101318")
        self.horizontal_scrollbar = ttk.Scrollbar(
            canvas_frame,
            orient=tk.HORIZONTAL,
            command=self._on_horizontal_scroll,
        )
        self.vertical_scrollbar = ttk.Scrollbar(
            canvas_frame,
            orient=tk.VERTICAL,
            command=self._on_vertical_scroll,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.vertical_scrollbar.grid(row=0, column=1, sticky="ns")
        self.horizontal_scrollbar.grid(row=1, column=0, sticky="ew")
        self._set_status()

    def _bind_events(self) -> None:
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self._on_left_down)
        self.canvas.bind("<B1-Motion>", self._on_left_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_left_up)
        self.canvas.bind("<ButtonPress-2>", self._on_pan_down)
        self.canvas.bind("<B2-Motion>", self._on_pan_drag)
        self.canvas.bind("<ButtonRelease-2>", self._on_pan_up)
        self.canvas.bind("<ButtonPress-3>", self._on_pan_down)
        self.canvas.bind("<B3-Motion>", self._on_pan_drag)
        self.canvas.bind("<ButtonRelease-3>", self._on_pan_up)
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Button-4>", lambda event: self._zoom_at(event.x, event.y, 1.15))
        self.canvas.bind("<Button-5>", lambda event: self._zoom_at(event.x, event.y, 1.0 / 1.15))
        self.bind("<Delete>", lambda _event: self.delete_selected())
        self.bind("<BackSpace>", lambda _event: self.delete_selected())
        self.bind("<Control-s>", lambda _event: self.save())
        self.bind("<Control-z>", lambda _event: self.undo())
        self.bind("<Control-m>", lambda _event: self.smooth_all_points())
        self.bind("<f>", lambda _event: self.fit_view())
        self.bind("<Left>", lambda event: self.nudge_selected(-1.0, 0.0, event))
        self.bind("<Right>", lambda event: self.nudge_selected(1.0, 0.0, event))
        self.bind("<Up>", lambda event: self.nudge_selected(0.0, 1.0, event))
        self.bind("<Down>", lambda event: self.nudge_selected(0.0, -1.0, event))
        self.issue_tree.bind("<<TreeviewSelect>>", self._on_issue_selected)

    def _clear_validation(self, reason: str = "Not validated") -> None:
        self.validation_report = None
        self.validation_revision = None
        self.validation_external = False
        self.validation_issues_by_iid.clear()
        for item in self.issue_tree.get_children():
            self.issue_tree.delete(item)
        self.validation_summary.set(reason)

    def _mark_modified(
        self, *, geometry_dirty: bool, speed_dirty: bool = False
    ) -> None:
        if geometry_dirty and self.trajectory.format_name == "mpc":
            speed_dirty = True
        self.candidate = None
        self.dirty = True
        self.last_operation = "edited"
        self.geometry_dirty = self.geometry_dirty or geometry_dirty
        self.speed_dirty = self.speed_dirty or speed_dirty
        self.revision += 1
        self._clear_validation("Validation stale after edit")
        self._clear_clearance("Clearance: stale after edit")

    def _on_circular_changed(self) -> None:
        self._mark_modified(geometry_dirty=True, speed_dirty=True)
        self.validation_summary.set("Validation stale after topology change")
        self.redraw()
        self._set_status("circular topology changed")

    @staticmethod
    def _format_issue_value(value: object) -> str:
        if value is None:
            return ""
        text = str(value)
        return text if len(text) <= 80 else text[:77] + "..."

    def _show_validation_report(
        self,
        report: ValidationReport,
        *,
        current_document: bool = True,
        source_label: Optional[str] = None,
    ) -> None:
        self.validation_report = report
        self.validation_revision = self.revision if current_document else None
        self.validation_external = not current_document
        self.validation_issues_by_iid.clear()
        for item in self.issue_tree.get_children():
            self.issue_tree.delete(item)

        for issue in report.issues:
            s_value = "" if issue.s_m is None else f"{issue.s_m:.6g}"
            item = self.issue_tree.insert(
                "",
                tk.END,
                values=(
                    issue.severity.value,
                    issue.code,
                    "" if issue.line_number is None else issue.line_number,
                    s_value,
                    self._format_issue_value(issue.value),
                    issue.message,
                ),
            )
            self.validation_issues_by_iid[item] = issue

        metrics = report.metrics
        metrics_text = (
            f"points={metrics.point_count}"
            f"/{metrics.normalized_point_count}"
        )
        if metrics.total_distance_m is not None:
            metrics_text += f", length={metrics.total_distance_m:.3f} m"
        if metrics.min_spacing_m is not None and metrics.max_spacing_m is not None:
            metrics_text += (
                f", spacing={metrics.min_spacing_m:.4f}.."
                f"{metrics.max_spacing_m:.4f} m"
            )
        if metrics.closing_edge_spacing_m is not None:
            metrics_text += f", seam={metrics.closing_edge_spacing_m:.4f} m"
        report_source = source_label or (
            "current" if current_document else "selected file"
        )
        self.validation_summary.set(
            f"Validation ({report_source}): errors={report.error_count}, "
            f"warnings={report.warning_count}, info={report.info_count} | "
            f"{metrics_text}"
        )

    def validate_current(self) -> ValidationReport:
        report = validate_trajectory_data(
            self.trajectory,
            circular=bool(self.circular.get()),
        )
        self._show_validation_report(report)
        self._set_status("validation complete")
        return report

    def _clear_clearance(
        self,
        reason: str = "Clearance: not run",
        *,
        state: Optional[str] = None,
    ) -> None:
        self.clearance_grid = None
        self.clearance_report = None
        self.clearance_revision = None
        self.clearance_selected_issue = None
        window = self.__dict__.get("clearance_report_window")
        try:
            if window is not None and window.winfo_exists():
                window.destroy()
        except tk.TclError:
            pass
        self.clearance_report_window = None
        if state is None:
            state = "stale" if self.__dict__.get("clearance_config") else "not_run"
        self.clearance_state = state
        summary = self.__dict__.get("clearance_summary")
        if summary is not None:
            summary.set(reason)

    def configure_clearance(self) -> bool:
        """Edit offline map/vehicle settings without mutating the document."""

        default_map = _default_occupancy_grid_path(self.trajectory.path)
        initial = self.clearance_config
        if initial is None and default_map is not None:
            try:
                initial = provisional_default_config(default_map)
            except ValueError:
                # The dialog can still be used with explicit manual values.
                initial = None
        result = ask_clearance_settings(
            self,
            initial=initial,
            map_yaml_path=default_map,
        )
        if result is None:
            self._set_status("clearance settings cancelled")
            return False
        self.clearance_config = result
        self._clear_clearance(
            "Clearance: settings changed; validation required",
            state="stale",
        )
        self.redraw()
        self._set_status("clearance settings applied")
        return True

    def _ensure_clearance_config(self) -> Optional[ClearanceDialogConfig]:
        if self.clearance_config is None and not self.configure_clearance():
            return None
        return self.clearance_config

    @staticmethod
    def _trajectory_clearance_poses(data: TrajectoryData) -> Tuple[Pose2D, ...]:
        if data.format_name != "mpc":
            raise ValueError(
                "Wall-clearance validation is available for strict seven-column "
                "MPC trajectories only."
            )
        if len(data.rows) != len(data.points):
            raise ValueError("trajectory rows and points have different lengths")
        poses: List[Pose2D] = []
        for index, (row, point) in enumerate(zip(data.rows, data.points)):
            try:
                yaw = float(row["psi_rad"])
                s_m = float(row["s_m"])
                curvature = float(row["kappa_radpm"])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(
                    f"trajectory point {index} has invalid geometry metadata"
                ) from error
            values = (point[0], point[1], yaw, s_m, curvature)
            if not all(math.isfinite(value) for value in values):
                raise ValueError(
                    f"trajectory point {index} has non-finite geometry metadata"
                )
            poses.append(
                Pose2D(
                    x_m=point[0],
                    y_m=point[1],
                    yaw_rad=yaw,
                    s_m=s_m,
                    curvature_radpm=curvature,
                )
            )
        return tuple(poses)

    @staticmethod
    def _map_load_options(config: ClearanceDialogConfig) -> MapLoadOptions:
        # Match the current C++ runtime preprocessing: enclosed occupied
        # components with fewer than five cells are removed after thresholding.
        return MapLoadOptions(
            unknown_is_occupied=config.unknown_is_occupied,
            fill_free_holes_below_cells=5,
            runtime_binary_parity=True,
        )

    @staticmethod
    def _vehicle_footprint(
        config: ClearanceDialogConfig,
    ) -> VehicleFootprintSpec:
        return VehicleFootprintSpec(**config.vehicle_footprint_kwargs())

    def _load_clearance_context(
        self,
        config: ClearanceDialogConfig,
        data: TrajectoryData,
    ) -> Tuple[OccupancyGrid, VehicleFootprintSpec, Tuple[Pose2D, ...]]:
        grid = load_occupancy_grid(
            config.map_yaml_path,
            options=self._map_load_options(config),
        )
        vehicle = self._vehicle_footprint(config)
        poses = self._trajectory_clearance_poses(data)
        return grid, vehicle, poses

    def _validation_options(
        self, config: ClearanceDialogConfig
    ) -> ValidationOptions:
        return ValidationOptions(
            circular=bool(self.circular.get()),
            sweep_step_m=config.sweep_step_m,
            include_sweep=True,
        )

    def _show_clearance_report(
        self,
        report: ClearanceReport,
        *,
        current_document: bool,
    ) -> None:
        if current_document:
            self.clearance_report = report
            self.clearance_revision = self.revision
            self.clearance_state = "safe" if report.is_safe else "unsafe"
            self.clearance_summary.set(
                "Clearance: " + clearance_report_summary(report)
            )
        existing = self.clearance_report_window
        try:
            if existing is not None and existing.winfo_exists():
                existing.destroy()
        except tk.TclError:
            pass
        self.clearance_report_window = show_clearance_report(
            self,
            report=report,
            on_center_issue=self._center_clearance_issue,
        )
        self.redraw()

    def _center_clearance_issue(self, issue: object) -> None:
        index = getattr(issue, "point_index", None)
        if index is None:
            index = getattr(issue, "segment_index", None)
        if index is None or not self.trajectory.points:
            self._set_status("clearance issue has no trajectory location")
            return
        self.selected_index = min(max(int(index), 0), len(self.trajectory.points) - 1)
        self.clearance_selected_issue = issue
        self.center_selection()
        self._set_status(f"clearance issue={getattr(issue, 'code', 'unknown')}")

    def _clearance_precheck(self) -> bool:
        if self.trajectory.format_name != "mpc":
            messagebox.showinfo(
                "Wall Clearance",
                "Wall-clearance validation and adjustment are currently limited "
                "to strict seven-column MPC trajectories. Original/Working "
                "layers and scrollbars remain available for Pure Pursuit.",
            )
            return False
        if self.geometry_dirty:
            messagebox.showerror(
                "Wall Clearance blocked",
                "Heading/curvature metadata is stale. Run Recompute Geometry or "
                "Normalize Geometry before checking the oriented vehicle footprint.",
            )
            self._set_status("clearance blocked: geometry stale")
            return False
        report = self.validate_current()
        if not report.is_valid:
            messagebox.showerror(
                "Wall Clearance blocked",
                f"Trajectory validation found {report.error_count} error(s).",
            )
            self._set_status("clearance blocked: trajectory validation errors")
            return False
        return True

    def validate_clearance_action(self) -> Optional[ClearanceReport]:
        """Run a read-only footprint and swept-footprint wall check."""

        if not self._clearance_precheck():
            return None
        config = self._ensure_clearance_config()
        if config is None:
            return None
        self._clear_clearance("Clearance: validation running", state="running")
        source_revision = self.revision
        snapshot = copy.deepcopy(self.trajectory)
        validation_options = self._validation_options(config)
        try:
            result = self._run_pure_candidate_task(
                "Validate Wall Clearance",
                lambda: self._validate_clearance_snapshot(
                    config,
                    snapshot,
                    validation_options,
                    source_revision,
                ),
            )
            if (
                not isinstance(result, tuple)
                or len(result) != 2
                or not isinstance(result[0], OccupancyGrid)
                or not isinstance(result[1], ClearanceReport)
            ):
                raise TypeError("clearance worker returned an unexpected result")
            grid, report = result
            if source_revision != self.revision:
                raise ValueError(
                    "working trajectory changed while clearance was being checked"
                )
        except Exception as exc:  # noqa: BLE001
            self._clear_clearance(
                "Clearance: validation failed; previous result invalidated",
                state="failed",
            )
            messagebox.showerror("Validate Clearance failed", str(exc))
            self._set_status("clearance validation failed")
            return None

        self.clearance_grid = grid
        self._show_clearance_report(report, current_document=True)
        self._set_status(
            "clearance safe" if report.is_safe else "clearance violations found"
        )
        return report

    def _validate_clearance_snapshot(
        self,
        config: ClearanceDialogConfig,
        snapshot: TrajectoryData,
        options: ValidationOptions,
        source_revision: int,
    ) -> Tuple[OccupancyGrid, ClearanceReport]:
        """Load and validate detached data without touching Tk state."""

        grid, vehicle, poses = self._load_clearance_context(config, snapshot)
        return grid, validate_clearance(
            grid,
            poses,
            vehicle,
            options=options,
            source_revision=source_revision,
        )

    def _clearance_apply_guard(
        self,
        *,
        config: ClearanceDialogConfig,
        expected_map_signature: str,
        vehicle: VehicleFootprintSpec,
        options: ValidationOptions,
    ) -> Callable[[TrajectoryData], Tuple[bool, str, Optional[object]]]:
        def guard(data: TrajectoryData) -> Tuple[bool, str, Optional[object]]:
            if self.clearance_config != config:
                return False, "clearance settings changed after candidate creation", None
            try:
                fresh_grid = load_occupancy_grid(
                    config.map_yaml_path,
                    options=self._map_load_options(config),
                )
                if fresh_grid.spec.signature != expected_map_signature:
                    return False, "occupancy-grid content changed after preview", None
                fresh_report = validate_clearance(
                    fresh_grid,
                    self._trajectory_clearance_poses(data),
                    vehicle,
                    options=options,
                    source_revision=self.revision,
                )
            except Exception as exc:  # noqa: BLE001
                return False, f"clearance recheck failed: {exc}", None
            if not fresh_report.is_safe:
                return False, "candidate is no longer wall-clearance safe", fresh_report
            return True, "", fresh_report

        return guard

    def adjust_clearance_action(self) -> bool:
        """Create a detached, constrained lateral-adjustment candidate."""

        if not self._clearance_precheck():
            return False
        config = self._ensure_clearance_config()
        if config is None:
            return False
        self._clear_clearance("Clearance: adjustment running", state="running")
        source_revision = self.revision
        snapshot = copy.deepcopy(self.trajectory)
        circular = bool(self.circular.get())
        final_options = self._validation_options(config)
        try:
            parameters = AdjustmentParameters(
                **config.adjustment_parameter_kwargs(
                    circular=circular
                )
            )
            task_result = self._run_pure_candidate_task(
                "Adjust Wall Clearance",
                lambda cancel_event: self._adjust_clearance_snapshot(
                    config,
                    snapshot,
                    parameters,
                    source_revision,
                    cancel_requested=cancel_event.is_set,
                ),
                cancellable=True,
            )
            if not isinstance(task_result, tuple) or len(task_result) != 3:
                raise TypeError("clearance worker returned an unexpected result")
            grid, vehicle, result = task_result
            if not isinstance(grid, OccupancyGrid):
                raise TypeError("clearance worker returned an invalid map")
            if not isinstance(vehicle, VehicleFootprintSpec):
                raise TypeError("clearance worker returned an invalid vehicle")
            if not isinstance(result, AdjustmentResult):
                raise TypeError("clearance worker returned an invalid adjustment")
            if source_revision != self.revision:
                raise ValueError(
                    "working trajectory changed while a clearance candidate was generated"
                )
        except CancelledError:
            self._clear_clearance(
                "Clearance: adjustment cancelled; validation required",
                state="stale",
            )
            self._set_status("clearance adjustment cancelled")
            return False
        except Exception as exc:  # noqa: BLE001
            self._clear_clearance(
                "Clearance: adjustment failed; previous result invalidated",
                state="failed",
            )
            messagebox.showerror("Adjust Clearance failed", str(exc))
            self._set_status("clearance adjustment failed")
            return False

        if result.status is AdjustmentStatus.NOT_NEEDED:
            self.clearance_grid = grid
            self._show_clearance_report(result.before_report, current_document=True)
            messagebox.showinfo(
                "Adjust Clearance",
                "The current trajectory already satisfies the selected clearance "
                "conditions; no candidate was created.",
            )
            self._set_status("clearance adjustment not needed")
            return False
        if result.status is not AdjustmentStatus.FEASIBLE or result.candidate is None:
            self.clearance_grid = grid
            self._show_clearance_report(result.before_report, current_document=True)
            messagebox.showerror(
                "Adjust Clearance infeasible",
                "No safe path was found within the selected maximum lateral shift. "
                "Working data was not changed.",
            )
            self._set_status("clearance adjustment infeasible")
            return False

        clearance_candidate = result.candidate
        if len(clearance_candidate.poses) != len(snapshot.points):
            messagebox.showerror(
                "Adjust Clearance failed",
                "The clearance candidate changed the point count unexpectedly.",
            )
            return False
        adjusted = copy.deepcopy(snapshot)
        adjusted.points = [
            (pose.x_m, pose.y_m) for pose in clearance_candidate.poses
        ]
        try:
            candidate_result = self._run_pure_candidate_task(
                "Validate Adjusted Clearance",
                lambda: self._validate_adjusted_clearance_candidate(
                    adjusted,
                    grid,
                    vehicle,
                    parameters,
                    final_options,
                    circular,
                    source_revision,
                ),
            )
            if not isinstance(candidate_result, tuple) or len(candidate_result) != 3:
                raise TypeError("candidate worker returned an unexpected result")
            adjusted, validation, final_report = candidate_result
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Adjust Clearance failed", str(exc))
            self._set_status("clearance candidate validation failed")
            return False
        if not validation.is_valid or not final_report.is_safe:
            self._show_clearance_report(final_report, current_document=False)
            messagebox.showerror(
                "Adjust Clearance rejected",
                "The regenerated candidate failed trajectory or clearance validation. "
                "Working data was not changed.",
            )
            self._set_status("clearance candidate rejected")
            return False

        candidate = EditorCandidate(
            source_revision=source_revision,
            operation="adjust_clearance",
            trajectory=adjusted,
            validation=validation,
            transformation=clearance_candidate,
            parameters={
                "map": str(config.map_yaml_path),
                "map_signature": grid.spec.signature,
                "vehicle_reference": config.reference_point,
                "body_length_m": config.body_length_m,
                "body_width_m": config.body_width_m,
                "envelope_length_m": config.envelope_length_m,
                "envelope_width_m": config.envelope_width_m,
                "minimum_clearance_m": final_report.minimum_clearance_m,
                "max_lateral_shift_m": clearance_candidate.max_shift_m,
                "max_abs_curvature_radpm": parameters.max_abs_curvature_radpm,
            },
            suggested_suffix="_clearance_adjusted",
            geometry_dirty=False,
            speed_dirty=True,
            apply_guard=self._clearance_apply_guard(
                config=config,
                expected_map_signature=grid.spec.signature,
                vehicle=vehicle,
                options=final_options,
            ),
            safety_payload=final_report,
        )
        self.clearance_grid = grid
        self.clearance_report = result.before_report
        self.clearance_revision = self.revision
        self.clearance_state = "unsafe"
        self.clearance_summary.set(
            "Clearance: " + clearance_report_summary(result.before_report)
        )
        applied = self._present_candidate(candidate)
        if applied:
            self._set_status(
                "clearance candidate applied; recompute speed before saving"
            )
        return applied

    def _adjust_clearance_snapshot(
        self,
        config: ClearanceDialogConfig,
        snapshot: TrajectoryData,
        parameters: AdjustmentParameters,
        source_revision: int,
        *,
        cancel_requested: Optional[Callable[[], bool]] = None,
    ) -> Tuple[OccupancyGrid, VehicleFootprintSpec, AdjustmentResult]:
        """Load inputs and create a detached adjustment result off the Tk thread."""

        grid, vehicle, poses = self._load_clearance_context(config, snapshot)
        result = adjust_clearance(
            grid,
            poses,
            vehicle,
            parameters=parameters,
            source_revision=source_revision,
            cancel_requested=cancel_requested,
        )
        return grid, vehicle, result

    def _validate_adjusted_clearance_candidate(
        self,
        adjusted: TrajectoryData,
        grid: OccupancyGrid,
        vehicle: VehicleFootprintSpec,
        parameters: AdjustmentParameters,
        options: ValidationOptions,
        circular: bool,
        source_revision: int,
    ) -> Tuple[TrajectoryData, ValidationReport, ClearanceReport]:
        """Regenerate and fully validate a detached candidate off the Tk thread."""

        recompute_geometry(adjusted, circular=circular)
        validation = validate_trajectory_data(adjusted, circular=circular)
        final_poses = self._trajectory_clearance_poses(adjusted)
        maximum_curvature = max(
            abs(pose.curvature_radpm or 0.0) for pose in final_poses
        )
        if (
            parameters.max_abs_curvature_radpm is not None
            and maximum_curvature
            > parameters.max_abs_curvature_radpm + 1e-9
        ):
            raise ValueError(
                "regenerated candidate exceeds maximum absolute curvature: "
                f"{maximum_curvature:.6g} > "
                f"{parameters.max_abs_curvature_radpm:.6g} rad/m"
            )
        final_report = validate_clearance(
            grid,
            final_poses,
            vehicle,
            options=options,
            source_revision=source_revision,
        )
        return adjusted, validation, final_report

    def _run_pure_candidate_task(
        self,
        title: str,
        task: Callable[..., object],
        *,
        cancellable: bool = False,
    ) -> object:
        """Use a responsive modal worker in Tk and a direct path in headless tests."""

        if self.__dict__.get("tk") is None:
            return task(threading.Event()) if cancellable else task()
        return run_candidate_task(
            self,
            title=title,
            task=task,
            cancellable=cancellable,
        )

    def _preview_baseline(self) -> Tuple[TrajectoryData, ValidationReport]:
        """Return a detached, renderable snapshot of the current revision."""

        baseline = copy.deepcopy(self.trajectory)
        report = validate_trajectory_data(
            baseline,
            circular=bool(self.circular.get()),
        )
        if report.is_valid:
            return baseline, report

        # A repairable baseline remains truthful: zero-length spacing and stale
        # derived columns are shown as-is, while the preview labels geometry
        # metadata stale.  Schema/non-finite failures remain unrenderable.
        if not _is_normalization_repairable(
            baseline,
            report,
        ):
            raise ValueError(
                "The current revision cannot be plotted safely. Run Validate "
                "and resolve its errors before creating a preview."
            )
        if any(
            issue.code == "INVALID_NUMBER"
            for issue in report.issues
            if issue.severity is Severity.ERROR
        ):
            recompute_geometry(baseline, circular=bool(self.circular.get()))
            report = validate_trajectory_data(
                baseline,
                circular=bool(self.circular.get()),
            )
            if not report.is_valid and not _is_normalization_repairable(
                baseline,
                report,
            ):
                raise ValueError(
                    "The repairable source could not be converted into a safe "
                    "Before plot snapshot."
                )
        return baseline, report

    def _present_candidate(self, candidate: EditorCandidate) -> bool:
        """Preview one detached candidate and apply it only after confirmation."""

        if candidate.source_revision != self.revision:
            messagebox.showerror(
                "Candidate is stale",
                "The working trajectory changed after this candidate was created. "
                "Create a new candidate from the current revision.",
            )
            self._set_status("stale candidate rejected")
            return False

        try:
            baseline, baseline_report = self._preview_baseline()
            candidate.plot_data = self._run_pure_candidate_task(
                "Build Candidate Preview",
                lambda: build_comparison_plot(
                    baseline,
                    baseline_report,
                    candidate.trajectory,
                    candidate.validation,
                    allow_repairable_before=not baseline_report.is_valid,
                    allow_candidate_errors=not candidate.validation.is_valid,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Candidate preview failed", str(exc))
            self._set_status("candidate preview failed")
            return False

        self.candidate = candidate
        self.redraw()
        self._set_status(f"previewing candidate={candidate.operation}")
        preview_parameters = dict(candidate.parameters)
        if self.geometry_dirty:
            preview_parameters["before_geometry_metadata"] = "stale"
        try:
            apply_requested = preview_candidate(
                self,
                comparison=candidate.plot_data,
                validation=candidate.validation,
                transformation=candidate.transformation,
                operation=candidate.operation,
                parameters=preview_parameters,
            )
        except Exception as exc:  # noqa: BLE001
            if self.candidate is candidate:
                self.candidate = None
            self.redraw()
            messagebox.showerror("Candidate preview failed", str(exc))
            self._set_status("candidate preview failed; working data unchanged")
            return False
        if apply_requested:
            return self._apply_candidate(candidate)

        if self.candidate is candidate:
            self.candidate = None
        if not candidate.validation.is_valid:
            self._show_validation_report(
                candidate.validation,
                current_document=False,
                source_label=f"{candidate.operation} candidate",
            )
        self.redraw()
        self._set_status("candidate discarded; working data unchanged")
        return False

    def _apply_candidate(self, candidate: EditorCandidate) -> bool:
        """Commit a validated candidate when its source revision is current."""

        if self.candidate is not candidate or candidate.source_revision != self.revision:
            if self.candidate is candidate:
                self.candidate = None
            messagebox.showerror(
                "Candidate is stale",
                "The candidate no longer matches the working trajectory revision.",
            )
            self.redraw()
            self._set_status("stale candidate rejected")
            return False
        if _trajectory_content_signature(candidate.trajectory) != candidate.content_signature:
            self.candidate = None
            messagebox.showerror(
                "Candidate changed",
                "The candidate content changed after validation. Create a new "
                "candidate before applying it.",
            )
            self.redraw()
            self._set_status("mutated candidate rejected")
            return False
        if not candidate.validation.is_valid:
            messagebox.showerror(
                "Candidate rejected",
                "A candidate with validation errors cannot be applied.",
            )
            self._set_status("invalid candidate rejected")
            return False

        fresh_report = validate_trajectory_data(
            candidate.trajectory,
            circular=bool(self.circular.get()),
        )
        if not fresh_report.is_valid:
            self.candidate = None
            self._show_validation_report(
                fresh_report,
                current_document=False,
                source_label=f"{candidate.operation} candidate recheck",
            )
            messagebox.showerror(
                "Candidate rejected",
                "The candidate failed validation immediately before Apply.",
            )
            self.redraw()
            self._set_status("candidate revalidation failed")
            return False

        safety_payload = candidate.safety_payload
        current_clearance = self.__dict__.get("clearance_report")
        if (
            safety_payload is None
            and candidate.operation == "recompute_speed_profile"
            and isinstance(current_clearance, ClearanceReport)
            and current_clearance.is_safe
            and self.__dict__.get("clearance_state") == "safe"
            and self.__dict__.get("clearance_revision") == self.revision
            and _trajectory_geometry_signature(candidate.trajectory)
            == _trajectory_geometry_signature(self.trajectory)
        ):
            # A speed-profile candidate changes only vx/ax metadata.  Keep the
            # current geometry-bound SAFE result and advance its revision below.
            safety_payload = current_clearance
        if candidate.apply_guard is not None:
            try:
                guard_result = self._run_pure_candidate_task(
                    "Recheck Candidate Safety",
                    lambda: candidate.apply_guard(candidate.trajectory),
                )
                safe_to_apply, rejection_reason, safety_payload = guard_result
            except Exception as exc:  # noqa: BLE001
                safe_to_apply = False
                rejection_reason = f"candidate safety recheck failed: {exc}"
                safety_payload = None
            if not safe_to_apply:
                self.candidate = None
                if isinstance(safety_payload, ClearanceReport):
                    self._show_clearance_report(
                        safety_payload,
                        current_document=False,
                    )
                messagebox.showerror(
                    "Candidate safety check failed",
                    rejection_reason or "The candidate is no longer safe to apply.",
                )
                self.redraw()
                self._set_status("candidate safety recheck failed")
                return False

        self._push_undo()
        selected_index = self.selected_index
        self.trajectory = copy.deepcopy(candidate.trajectory)
        if selected_index is None:
            self.selected_index = None
        else:
            self.selected_index = min(
                max(selected_index, 0), len(self.trajectory.points) - 1
            )
        self.dirty = True
        self.geometry_dirty = candidate.geometry_dirty
        self.speed_dirty = candidate.speed_dirty
        self.last_operation = candidate.operation
        self.candidate = None
        self.revision += 1
        self._clear_validation("Validation stale after candidate apply")
        self._clear_clearance("Clearance: stale after candidate apply")
        if isinstance(safety_payload, ClearanceReport):
            self.clearance_report = safety_payload
            self.clearance_revision = self.revision
            self.clearance_state = "safe"
            summary = self.__dict__.get("clearance_summary")
            if summary is not None:
                summary.set("Clearance: " + clearance_report_summary(safety_payload))
        self._show_validation_report(candidate.validation)
        self.redraw()
        self._set_status(f"candidate applied={candidate.operation}")
        return True

    def normalize_geometry_candidate(self) -> None:
        """Generate an explicit MPC normalization candidate for preview."""

        if self.trajectory.format_name != "mpc":
            messagebox.showinfo(
                "Normalize Geometry",
                "Normalize Geometry is available for strict seven-column MPC "
                "trajectories only. Pure Pursuit editing remains unchanged.",
            )
            return
        options = ask_normalize_options(
            self,
            circular=bool(self.circular.get()),
            speed_dirty=self.speed_dirty,
        )
        if options is None:
            self._set_status("normalization cancelled")
            return
        source_revision = self.revision
        source_snapshot = copy.deepcopy(self.trajectory)
        try:
            result = self._run_pure_candidate_task(
                "Normalize Geometry",
                lambda: normalize_geometry(
                    source_snapshot,
                    options,
                    source_revision=source_revision,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Normalize Geometry failed", str(exc))
            self._set_status("normalization failed; working data unchanged")
            return

        candidate = EditorCandidate(
            source_revision=result.source_revision,
            operation=result.operation,
            trajectory=result.dataset,
            validation=result.validation,
            transformation=result.transformation,
            parameters=dict(result.parameters),
            suggested_suffix="_normalized",
            geometry_dirty=False,
            speed_dirty=(
                result.transformation.metadata_mode is MetadataMode.RECOMPUTE
            ),
        )
        self._present_candidate(candidate)

    def recompute_speed_candidate(self) -> None:
        """Generate an offline MPC speed-profile candidate for preview."""

        if self.trajectory.format_name != "mpc":
            messagebox.showinfo(
                "Recompute Speed Profile",
                "Speed-profile generation is available for strict seven-column "
                "MPC trajectories only.",
            )
            return
        if self.geometry_dirty:
            messagebox.showerror(
                "Recompute Speed Profile blocked",
                "Geometry-derived fields are stale. Apply Normalize Geometry or "
                "run Recompute Geometry before generating a speed profile.",
            )
            self._set_status("speed candidate blocked: geometry stale")
            return

        report = validate_trajectory_data(
            self.trajectory,
            circular=bool(self.circular.get()),
        )
        if not report.is_valid:
            self._show_validation_report(report)
            messagebox.showerror(
                "Recompute Speed Profile blocked",
                f"The current trajectory has {report.error_count} validation "
                "error(s).",
            )
            self._set_status("speed candidate blocked: validation errors")
            return

        metrics = report.metrics
        defaults = {
            "v_max_mps": max(metrics.max_velocity_mps or 0.0, 0.1),
            "a_max_mps2": LOCAL_A_MAX_MPS2,
            "a_min_mps2": min(metrics.min_acceleration_mps2 or 0.0, -1.0),
            "ay_max_mps2": max(
                metrics.max_lateral_acceleration_mps2 or 0.0,
                1.0,
            ),
            "minimum_speed_mps": 0.0,
        }
        parameters = ask_speed_parameters(
            self,
            circular=bool(self.circular.get()),
            defaults=defaults,
        )
        if parameters is None:
            self._set_status("speed-profile generation cancelled")
            return
        if not isinstance(parameters, SpeedProfileParameters):
            messagebox.showerror(
                "Recompute Speed Profile failed",
                "The speed dialog returned an invalid parameter object.",
            )
            return

        source_revision = self.revision
        source_snapshot = copy.deepcopy(self.trajectory)
        circular = bool(self.circular.get())
        try:
            result = self._run_pure_candidate_task(
                "Recompute Speed Profile",
                lambda: recompute_speed_profile(
                    source_snapshot,
                    circular=circular,
                    parameters=parameters,
                    source_revision=source_revision,
                ),
            )
        except Exception as exc:  # noqa: BLE001
            self.candidate = None
            messagebox.showerror("Recompute Speed Profile failed", str(exc))
            self._set_status("speed-profile failure; working data unchanged")
            return
        if result.candidate is None:
            self._show_validation_report(
                result.report,
                current_document=False,
                source_label="speed-profile candidate",
            )
            messagebox.showerror(
                "Recompute Speed Profile failed",
                f"No safe candidate was produced ({result.report.error_count} "
                "validation error(s)).",
            )
            self._set_status("speed-profile candidate rejected")
            return

        parameter_values: Dict[str, object] = {
            "v_max_mps": parameters.v_max_mps,
            "a_max_mps2": parameters.a_max_mps2,
            "a_min_mps2": parameters.a_min_mps2,
            "ay_max_mps2": parameters.ay_max_mps2,
            "minimum_speed_mps": parameters.minimum_speed_mps,
            "epsilon": parameters.epsilon,
            "tolerance": parameters.tolerance,
            "max_iterations": parameters.max_iterations,
            "iterations": result.iterations,
            "runtime_consumption": "offline metadata only; pending",
        }
        candidate = EditorCandidate(
            source_revision=result.source_revision,
            operation="recompute_speed_profile",
            trajectory=result.candidate,
            validation=result.report,
            transformation={"iterations": result.iterations},
            parameters=parameter_values,
            suggested_suffix="_speed_profiled",
            geometry_dirty=False,
            speed_dirty=False,
        )
        self._present_candidate(candidate)

    def _on_issue_selected(self, _event: tk.Event) -> None:
        selection = self.issue_tree.selection()
        if not selection:
            return
        issue = self.validation_issues_by_iid.get(selection[0])
        if issue is None:
            return
        if self.validation_external:
            self._set_status(f"external issue={issue.code}")
            return
        index = issue.point_index
        if index is None and issue.segment_index is not None:
            index = issue.segment_index
        if index is not None and self.trajectory.points:
            self.selected_index = min(max(index, 0), len(self.trajectory.points) - 1)
            self.center_selection()
        self._set_status(f"issue={issue.code}")

    def _set_status(self, extra: str = "") -> None:
        selected = "--" if self.selected_index is None else str(self.selected_index)
        dirty_state = "modified" if self.dirty else "saved"
        derived_state = []
        if self.geometry_dirty:
            derived_state.append("geometry-stale")
        if self.speed_dirty:
            derived_state.append("speed-stale")
        if not derived_state:
            derived_state.append("derived-ok")
        if self.validation_revision == self.revision and self.validation_report is not None:
            validation_state = (
                "validated-ok"
                if self.validation_report.is_valid
                else f"validated-errors={self.validation_report.error_count}"
            )
        else:
            validation_state = "not-validated"
        candidate_state = (
            "candidate=none"
            if self.candidate is None
            else f"candidate={self.candidate.operation}@{self.candidate.source_revision}"
        )
        runtime_speed_state = (
            "offline-speed-metadata(runtime-consumption-pending)"
            if self.last_operation == "recompute_speed_profile"
            else "runtime-speed-unchanged"
        )
        base = (
            f"traj={self.trajectory.path} | format={self.trajectory.format_name} | "
            f"circular={int(bool(self.circular.get()))} | revision={self.revision} | "
            f"{dirty_state} | {','.join(derived_state)} | {validation_state} | "
            f"last={self.last_operation} | {candidate_state} | "
            f"{runtime_speed_state} | "
            f"osm={self.osm_path} | "
            f"points={len(self.trajectory.points)} | selected={selected} | "
            f"influence=+/-{self._influence_radius()}"
        )
        if extra:
            base = f"{base} | {extra}"
        self.status.set(base)

    def _smooth_alpha(self) -> float:
        try:
            return max(0.0, min(0.5, float(self.smooth_alpha.get())))
        except (tk.TclError, ValueError):
            return 0.15

    def _smooth_passes(self) -> int:
        try:
            return max(1, min(10, int(self.smooth_passes.get())))
        except (tk.TclError, ValueError):
            return 1

    def _layer_visible(self, attribute: str, *, default: bool = True) -> bool:
        variable = getattr(self, attribute, None)
        if variable is None:
            return default
        try:
            return bool(variable.get())
        except (tk.TclError, AttributeError):
            return default

    @staticmethod
    def _clearance_cell_polygon(grid: OccupancyGrid, cell: Tuple[int, int]) -> Tuple[Point, ...]:
        row, column = cell
        if grid.state(row, column).value == "outside":
            return ()
        resolution = grid.spec.resolution_m
        map_y = grid.height - 1 - row
        center_x = column * resolution
        center_y = map_y * resolution
        half = 0.5 * resolution
        return tuple(
            grid.local_to_world(center_x + dx, center_y + dy)
            for dx, dy in ((-half, -half), (half, -half), (half, half), (-half, half))
        )

    def _visible_content_bounds(self) -> Optional[Tuple[float, float, float, float]]:
        points: List[Point] = []
        for rail in getattr(self, "rails", []):
            points.extend(rail)
        if self._layer_visible("show_original"):
            original = getattr(
                self,
                "loaded_original",
                getattr(self, "original_trajectory", None),
            )
            if original is not None:
                points.extend(original.points)
        if self._layer_visible("show_working"):
            points.extend(self.trajectory.points)
        candidate = getattr(self, "candidate", None)
        if candidate is not None and self._layer_visible("show_candidate"):
            points.extend(candidate.trajectory.points)
        clearance_report = self.__dict__.get("clearance_report")
        clearance_grid = self.__dict__.get("clearance_grid")
        if (
            clearance_report is not None
            and clearance_grid is not None
            and self.__dict__.get("clearance_revision") == self.revision
        ):
            for issue in tuple(clearance_report.issues)[:500]:
                if issue.grid_cell is not None:
                    points.extend(
                        self._clearance_cell_polygon(
                            clearance_grid,
                            issue.grid_cell,
                        )
                    )
            selected_issue = self.__dict__.get("clearance_selected_issue")
            config = self.__dict__.get("clearance_config")
            if selected_issue is not None and config is not None:
                index = getattr(selected_issue, "point_index", None)
                if index is None:
                    index = getattr(selected_issue, "segment_index", None)
                if index is not None and self.trajectory.points:
                    index = min(max(int(index), 0), len(self.trajectory.points) - 1)
                    try:
                        pose = self._trajectory_clearance_poses(self.trajectory)[index]
                        vehicle = self._vehicle_footprint(config)
                        points.extend(
                            footprint_polygon(pose, vehicle, include_margin=True)
                        )
                    except Exception:  # noqa: BLE001
                        pass

        finite_points = [
            point
            for point in points
            if math.isfinite(point[0]) and math.isfinite(point[1])
        ]
        if not finite_points:
            return None
        return (
            min(point[0] for point in finite_points),
            max(point[0] for point in finite_points),
            min(point[1] for point in finite_points),
            max(point[1] for point in finite_points),
        )

    def _scroll_domain(self) -> Tuple[float, float, float, float]:
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        scale = max(float(self.scale), 1e-9)
        bounds = self._visible_content_bounds()
        if bounds is None:
            half_width = width / (2.0 * scale)
            half_height = height / (2.0 * scale)
            return (
                self.center_x - half_width,
                self.center_x + half_width,
                self.center_y - half_height,
                self.center_y + half_height,
            )

        min_x, max_x, min_y, max_y = bounds
        padding_world = 40.0 / scale
        min_x -= padding_world
        max_x += padding_world
        min_y -= padding_world
        max_y += padding_world

        viewport_width = width / scale
        viewport_height = height / scale
        if max_x - min_x < viewport_width:
            midpoint = 0.5 * (min_x + max_x)
            min_x = midpoint - 0.5 * viewport_width
            max_x = midpoint + 0.5 * viewport_width
        if max_y - min_y < viewport_height:
            midpoint = 0.5 * (min_y + max_y)
            min_y = midpoint - 0.5 * viewport_height
            max_y = midpoint + 0.5 * viewport_height
        return min_x, max_x, min_y, max_y

    def _scroll_fractions(self, axis: str) -> Tuple[float, float]:
        min_x, max_x, min_y, max_y = self._scroll_domain()
        scale = max(float(self.scale), 1e-9)
        if axis == "x":
            domain_min, domain_max = min_x, max_x
            viewport_span = max(self.canvas.winfo_width(), 1) / scale
            visible_start = self.center_x - 0.5 * viewport_span
        else:
            domain_min, domain_max = min_y, max_y
            viewport_span = max(self.canvas.winfo_height(), 1) / scale
            # Scrollbar coordinates increase down while world Y increases up.
            visible_start = domain_max - (self.center_y + 0.5 * viewport_span)

        domain_span = max(domain_max - domain_min, viewport_span, 1e-12)
        if axis == "x":
            first = (visible_start - domain_min) / domain_span
        else:
            first = visible_start / domain_span
        visible_fraction = min(1.0, viewport_span / domain_span)
        first = max(0.0, min(first, 1.0 - visible_fraction))
        return first, min(1.0, first + visible_fraction)

    def _set_scroll_fraction(self, axis: str, first: float) -> None:
        min_x, max_x, min_y, max_y = self._scroll_domain()
        scale = max(float(self.scale), 1e-9)
        if axis == "x":
            domain_min, domain_max = min_x, max_x
            viewport_span = max(self.canvas.winfo_width(), 1) / scale
        else:
            domain_min, domain_max = min_y, max_y
            viewport_span = max(self.canvas.winfo_height(), 1) / scale
        domain_span = max(domain_max - domain_min, viewport_span, 1e-12)
        visible_fraction = min(1.0, viewport_span / domain_span)
        first = max(0.0, min(float(first), 1.0 - visible_fraction))
        if axis == "x":
            self.center_x = domain_min + first * domain_span + 0.5 * viewport_span
        else:
            visible_top = domain_max - first * domain_span
            self.center_y = visible_top - 0.5 * viewport_span

    def _scroll_axis(self, axis: str, *args: str) -> None:
        if not args:
            return
        first, last = self._scroll_fractions(axis)
        if args[0] == "moveto" and len(args) >= 2:
            target = float(args[1])
        elif args[0] == "scroll" and len(args) >= 3:
            amount = int(args[1])
            step = (last - first) * (0.9 if args[2] == "pages" else 0.1)
            target = first + amount * step
        else:
            return
        self._set_scroll_fraction(axis, target)
        self.redraw()

    def _on_horizontal_scroll(self, *args: str) -> None:
        self._scroll_axis("x", *args)

    def _on_vertical_scroll(self, *args: str) -> None:
        self._scroll_axis("y", *args)

    def _clamp_view_center(self) -> None:
        for axis in ("x", "y"):
            first, _last = self._scroll_fractions(axis)
            self._set_scroll_fraction(axis, first)

    def _update_scrollbars(self) -> None:
        x_first, x_last = self._scroll_fractions("x")
        y_first, y_last = self._scroll_fractions("y")
        self.horizontal_scrollbar.set(x_first, x_last)
        self.vertical_scrollbar.set(y_first, y_last)
        min_x, max_x, min_y, max_y = self._scroll_domain()
        virtual_width = max(
            max(self.canvas.winfo_width(), 1),
            min((max_x - min_x) * self.scale, 1_000_000_000.0),
        )
        virtual_height = max(
            max(self.canvas.winfo_height(), 1),
            min((max_y - min_y) * self.scale, 1_000_000_000.0),
        )
        self.canvas.configure(scrollregion=(0.0, 0.0, virtual_width, virtual_height))

    def _on_layer_visibility_changed(self) -> None:
        self.redraw()
        self._set_status("layer visibility changed")

    def world_to_screen(self, point: Point) -> Point:
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        x = (point[0] - self.center_x) * self.scale + width * 0.5
        y = height * 0.5 - (point[1] - self.center_y) * self.scale
        return x, y

    def screen_to_world(self, x: float, y: float) -> Point:
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        wx = self.center_x + (x - width * 0.5) / self.scale
        wy = self.center_y + (height * 0.5 - y) / self.scale
        return wx, wy

    def fit_view(self) -> None:
        bounds = self._visible_content_bounds()
        if bounds is None:
            return
        min_x, max_x, min_y, max_y = bounds
        self.center_x = (min_x + max_x) * 0.5
        self.center_y = (min_y + max_y) * 0.5

        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        self.scale = max(
            0.01,
            min(0.9 * min(width / span_x, height / span_y), 5000.0),
        )
        self.redraw()

    def center_selection(self) -> None:
        if self.selected_index is None or not self.trajectory.points:
            self._set_status("center selection: no point selected")
            return
        index = min(max(self.selected_index, 0), len(self.trajectory.points) - 1)
        self.center_x, self.center_y = self.trajectory.points[index]
        self.redraw()
        self._set_status(f"centered selection={index}")

    def _draw_clearance_overlay(self) -> None:
        report = getattr(self, "clearance_report", None)
        if report is None or self.clearance_revision != self.revision:
            return

        grid = self.__dict__.get("clearance_grid")
        for issue in tuple(report.issues)[:500]:
            if grid is not None and issue.grid_cell is not None:
                cell_polygon = self._clearance_cell_polygon(grid, issue.grid_cell)
                if cell_polygon:
                    cell_coords: List[float] = []
                    for corner in (*cell_polygon, cell_polygon[0]):
                        cell_coords.extend(self.world_to_screen(corner))
                    self.canvas.create_line(
                        *cell_coords,
                        fill="#ff3b4f",
                        width=2,
                    )
            index = issue.point_index
            if index is None:
                index = issue.segment_index
            if index is None or not self.trajectory.points:
                continue
            point = self.trajectory.points[
                min(max(int(index), 0), len(self.trajectory.points) - 1)
            ]
            sx, sy = self.world_to_screen(point)
            radius = 5.0
            self.canvas.create_line(
                sx - radius,
                sy - radius,
                sx + radius,
                sy + radius,
                fill="#ff3b4f",
                width=2,
            )
            self.canvas.create_line(
                sx - radius,
                sy + radius,
                sx + radius,
                sy - radius,
                fill="#ff3b4f",
                width=2,
            )

        selected_issue = getattr(self, "clearance_selected_issue", None)
        config = getattr(self, "clearance_config", None)
        if selected_issue is None or config is None:
            return
        index = getattr(selected_issue, "point_index", None)
        if index is None:
            index = getattr(selected_issue, "segment_index", None)
        if index is None or not self.trajectory.points:
            return
        index = min(max(int(index), 0), len(self.trajectory.points) - 1)
        try:
            pose = self._trajectory_clearance_poses(self.trajectory)[index]
            vehicle = self._vehicle_footprint(config)
            polygons = (
                (footprint_polygon(pose, vehicle, include_margin=True), "#ff4fc3", (5, 3)),
                (footprint_polygon(pose, vehicle, include_margin=False), "#34d5eb", None),
            )
        except Exception:  # noqa: BLE001 - overlay must never break editing
            return
        for polygon, color, dash in polygons:
            coords: List[float] = []
            for point in (*polygon, polygon[0]):
                sx, sy = self.world_to_screen(point)
                coords.extend((sx, sy))
            self.canvas.create_line(
                *coords,
                fill=color,
                width=2,
                dash=dash,
            )

    def _refresh_original_difference(self) -> OriginalDifference:
        original = self.__dict__.get(
            "loaded_original",
            self.__dict__.get("original_trajectory"),
        )
        original_points = () if original is None else original.points
        difference = build_original_difference(
            original_points,
            self.trajectory.points,
        )
        self.original_difference = difference
        summary = self.__dict__.get("original_difference_summary")
        if summary is not None:
            ranges = ", ".join(
                f"{start:.2f}-{end:.2f}m"
                for start, end in difference.changed_ranges_m[:3]
            )
            if len(difference.changed_ranges_m) > 3:
                ranges += f", +{len(difference.changed_ranges_m) - 3} more"
            if not ranges:
                ranges = "none"
            summary.set(
                "Original diff: "
                f"points={difference.working_point_count - difference.original_point_count:+d}, "
                f"length={difference.working_length_m - difference.original_length_m:+.3f}m, "
                f"max={difference.maximum_displacement_m:.3f}m, "
                f"mean={difference.mean_displacement_m:.3f}m, changed={ranges}"
            )
        return difference

    def redraw(self) -> None:
        self._clamp_view_center()
        self.canvas.delete("all")
        difference = self._refresh_original_difference()

        for rail in self.rails:
            coords: List[float] = []
            for point in rail:
                sx, sy = self.world_to_screen(point)
                coords.extend([sx, sy])
            if len(coords) >= 4:
                self.canvas.create_line(
                    *coords,
                    fill="#6f7782",
                    width=1,
                    smooth=False,
                )

        if self._layer_visible("show_original"):
            original = getattr(
                self,
                "loaded_original",
                getattr(self, "original_trajectory", None),
            )
            original_coords: List[float] = []
            if original is not None:
                for index in _display_indices(len(original.points)):
                    sx, sy = self.world_to_screen(original.points[index])
                    original_coords.extend([sx, sy])
                if (
                    getattr(self, "loaded_original_circular", bool(self.circular.get()))
                    and original.points
                    and not _closed_duplicate(original.points)
                ):
                    sx, sy = self.world_to_screen(original.points[0])
                    original_coords.extend([sx, sy])
            if len(original_coords) >= 4:
                self.canvas.create_line(
                    *original_coords,
                    fill="#8b929c",
                    width=2,
                    dash=(3, 4),
                    smooth=False,
                )

        if self._layer_visible("show_working"):
            traj_coords: List[float] = []
            for index in _display_indices(len(self.trajectory.points)):
                point = self.trajectory.points[index]
                sx, sy = self.world_to_screen(point)
                traj_coords.extend([sx, sy])
            if (
                self.circular.get()
                and self.trajectory.points
                and not _closed_duplicate(self.trajectory.points)
            ):
                sx, sy = self.world_to_screen(self.trajectory.points[0])
                traj_coords.extend([sx, sy])
            if len(traj_coords) >= 4:
                self.canvas.create_line(
                    *traj_coords,
                    fill="#3aa0ff",
                    width=2,
                    smooth=False,
                )
            changed = set(difference.changed_indices)
            for index in range(len(self.trajectory.points) - 1):
                if index not in changed and index + 1 not in changed:
                    continue
                first = self.world_to_screen(self.trajectory.points[index])
                second = self.world_to_screen(self.trajectory.points[index + 1])
                self.canvas.create_line(
                    *first,
                    *second,
                    fill="#d65cff",
                    width=3,
                )

        if self.candidate is not None and self._layer_visible("show_candidate"):
            candidate_coords: List[float] = []
            for index in _display_indices(len(self.candidate.trajectory.points)):
                point = self.candidate.trajectory.points[index]
                sx, sy = self.world_to_screen(point)
                candidate_coords.extend([sx, sy])
            if (
                self.circular.get()
                and self.candidate.trajectory.points
                and not _closed_duplicate(self.candidate.trajectory.points)
            ):
                sx, sy = self.world_to_screen(self.candidate.trajectory.points[0])
                candidate_coords.extend([sx, sy])
            if len(candidate_coords) >= 4:
                self.canvas.create_line(
                    *candidate_coords,
                    fill="#ff9f43",
                    width=2,
                    dash=(7, 4),
                    smooth=False,
                )

        if self._layer_visible("show_working"):
            radius = 3.0
            influenced = self._influenced_indices(self.selected_index)
            visible_indices = set(_display_indices(len(self.trajectory.points)))
            visible_indices.update(influenced)
            if self.selected_index is not None:
                visible_indices.add(self.selected_index)
            for idx in sorted(visible_indices):
                point = self.trajectory.points[idx]
                sx, sy = self.world_to_screen(point)
                canonical_idx = self._canonical_index(idx, self.trajectory.points)
                if idx == self.selected_index:
                    fill = "#ffb02e"
                elif canonical_idx in influenced:
                    fill = "#8bd46e"
                else:
                    fill = "#e8eef7"
                outline = "#ffffff" if idx == self.selected_index else "#213040"
                self.canvas.create_oval(
                    sx - radius,
                    sy - radius,
                    sx + radius,
                    sy + radius,
                    fill=fill,
                    outline=outline,
                    width=1,
                )
        self._draw_clearance_overlay()
        self._update_scrollbars()

    def _push_undo(self) -> None:
        working_visibility = self.__dict__.get("show_working")
        if working_visibility is not None:
            try:
                if not bool(working_visibility.get()):
                    working_visibility.set(True)
            except (tk.TclError, AttributeError):
                pass
        self.undo_stack.append(
            UndoState(
                rows=copy.deepcopy(self.trajectory.rows),
                points=list(self.trajectory.points),
                selected_index=self.selected_index,
                dirty=self.dirty,
                geometry_dirty=self.geometry_dirty,
                speed_dirty=self.speed_dirty,
                last_operation=self.last_operation,
            )
        )
        if len(self.undo_stack) > 30:
            self.undo_stack.pop(0)

    def undo(self) -> None:
        if not self.undo_stack:
            self._set_status("nothing to undo")
            return
        state = self.undo_stack.pop()
        self.trajectory.rows = state.rows
        self.trajectory.points = state.points
        self.selected_index = state.selected_index
        self.dirty = state.dirty
        self.geometry_dirty = state.geometry_dirty
        self.speed_dirty = state.speed_dirty
        self.last_operation = state.last_operation
        self.candidate = None
        self.revision += 1
        self._clear_validation("Validation stale after undo")
        self._clear_clearance("Clearance: stale after undo")
        self.redraw()
        self._set_status("undo")

    def _nearest_point(self, x: float, y: float, max_px: float = 12.0) -> Optional[int]:
        best_index: Optional[int] = None
        best_dist_sq = max_px * max_px
        for idx, point in enumerate(self.trajectory.points):
            sx, sy = self.world_to_screen(point)
            dist_sq = (sx - x) ** 2 + (sy - y) ** 2
            if dist_sq <= best_dist_sq:
                best_dist_sq = dist_sq
                best_index = idx
        return best_index

    def _nearest_segment(self, x: float, y: float) -> Optional[int]:
        points = self.trajectory.points
        if len(points) < 2:
            return None
        best_index: Optional[int] = None
        best_dist_sq = float("inf")
        segment_count = len(points) - 1
        if self.circular.get() and not _closed_duplicate(points):
            segment_count += 1
        for i in range(segment_count):
            next_index = (i + 1) % len(points)
            ax, ay = self.world_to_screen(points[i])
            bx, by = self.world_to_screen(points[next_index])
            dx = bx - ax
            dy = by - ay
            denom = dx * dx + dy * dy
            if denom <= 1e-9:
                continue
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / denom))
            px = ax + t * dx
            py = ay + t * dy
            dist_sq = (px - x) ** 2 + (py - y) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_index = i
        return best_index

    def _on_left_down(self, event: tk.Event) -> None:
        self.focus_set()
        if not self._layer_visible("show_working"):
            self.show_working.set(True)
            self.redraw()
            self._set_status("Working layer restored before editing")
            return
        if event.state & 0x0001:
            self.insert_point(event.x, event.y)
            return

        index = self._nearest_point(event.x, event.y)
        self.selected_index = index
        if index is not None:
            self.drag_origin_points = list(self.trajectory.points)
            self.dragging_point = True
            self.drag_undo_saved = False
        self.redraw()
        self._set_status()

    def _on_left_drag(self, event: tk.Event) -> None:
        if not self.dragging_point or self.selected_index is None:
            return
        if self.drag_origin_points is None:
            self.drag_origin_points = list(self.trajectory.points)
        wx, wy = self.screen_to_world(event.x, event.y)
        origin_point = self.drag_origin_points[self.selected_index]
        dx = wx - origin_point[0]
        dy = wy - origin_point[1]
        if math.hypot(dx, dy) <= 1e-12:
            return
        if not self.drag_undo_saved:
            self._push_undo()
            self.drag_undo_saved = True
        self._apply_influenced_delta(
            self.drag_origin_points,
            self.selected_index,
            dx,
            dy,
        )
        self.redraw()
        self._set_status(f"x={wx:.3f}, y={wy:.3f}")

    def _on_left_up(self, _event: tk.Event) -> None:
        moved = self.drag_undo_saved
        if moved:
            self._mark_modified(geometry_dirty=True)
        self.dragging_point = False
        self.drag_undo_saved = False
        self.drag_origin_points = None
        self._set_status("point moved" if moved else "selection unchanged")

    def _on_pan_down(self, event: tk.Event) -> None:
        self.panning = True
        self.pan_anchor = (event.x, event.y)

    def _on_pan_drag(self, event: tk.Event) -> None:
        if not self.panning:
            return
        dx = event.x - self.pan_anchor[0]
        dy = event.y - self.pan_anchor[1]
        self.center_x -= dx / self.scale
        self.center_y += dy / self.scale
        self.pan_anchor = (event.x, event.y)
        self.redraw()

    def _on_pan_up(self, _event: tk.Event) -> None:
        self.panning = False

    def _on_mousewheel(self, event: tk.Event) -> None:
        factor = 1.15 if event.delta > 0 else 1.0 / 1.15
        self._zoom_at(event.x, event.y, factor)

    def _zoom_at(self, x: float, y: float, factor: float) -> None:
        before = self.screen_to_world(x, y)
        self.scale = max(0.01, min(self.scale * factor, 5000.0))
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        self.center_x = before[0] - (x - width * 0.5) / self.scale
        self.center_y = before[1] - (height * 0.5 - y) / self.scale
        self.redraw()

    def _influence_radius(self) -> int:
        try:
            return max(0, int(self.influence_radius_points.get()))
        except (tk.TclError, ValueError):
            return 0

    def _canonical_index(self, index: int, points: Sequence[Point]) -> int:
        if (
            self.circular.get()
            and _closed_duplicate(points)
            and index == len(points) - 1
        ):
            return 0
        return index

    def _influenced_indices(self, selected_index: Optional[int]) -> set:
        if selected_index is None:
            return set()
        points = self.trajectory.points
        closed = bool(self.circular.get())
        duplicate_endpoint = closed and _closed_duplicate(points)
        count = len(points) - 1 if duplicate_endpoint else len(points)
        if count <= 0:
            return set()

        selected = self._canonical_index(selected_index, points)
        radius = self._influence_radius()
        indices = set()
        for idx in range(count):
            distance = abs(idx - selected)
            if closed:
                distance = min(distance, count - distance)
            if 0 < distance <= radius:
                indices.add(idx)
        return indices

    def _influence_weight(self, distance: int, radius: int) -> float:
        if distance <= 0:
            return 1.0
        if radius <= 0 or distance > radius:
            return 0.0
        return 0.5 * (1.0 + math.cos(math.pi * distance / (radius + 1)))

    def _apply_influenced_delta(
        self,
        origin_points: Sequence[Point],
        selected_index: int,
        dx: float,
        dy: float,
    ) -> None:
        closed = bool(self.circular.get())
        duplicate_endpoint = closed and _closed_duplicate(origin_points)
        count = len(origin_points) - 1 if duplicate_endpoint else len(origin_points)
        if count <= 0:
            return

        selected = self._canonical_index(selected_index, origin_points)
        radius = self._influence_radius()
        updated = list(origin_points)

        for idx in range(count):
            distance = abs(idx - selected)
            if closed:
                distance = min(distance, count - distance)
            weight = self._influence_weight(distance, radius)
            if weight <= 0.0:
                continue
            x, y = origin_points[idx]
            updated[idx] = (x + dx * weight, y + dy * weight)

        if duplicate_endpoint:
            updated[-1] = updated[0]
        self.trajectory.points = updated

    def _set_point(self, index: int, point: Point) -> None:
        points = self.trajectory.points
        closed = bool(self.circular.get()) and _closed_duplicate(points)
        points[index] = point
        if closed:
            if index == 0:
                points[-1] = point
            elif index == len(points) - 1:
                points[0] = point

    def smooth_all_points(self) -> None:
        points = self.trajectory.points
        closed = bool(self.circular.get())
        duplicate_endpoint = closed and _closed_duplicate(points)
        count = len(points) - 1 if duplicate_endpoint else len(points)
        if count < 3:
            self._set_status("cannot smooth: too few points")
            return

        alpha = self._smooth_alpha()
        passes = self._smooth_passes()
        self._push_undo()

        smoothed = list(points)
        for _ in range(passes):
            source = list(smoothed)
            updated = list(source)
            for idx in range(count):
                if not closed and (idx == 0 or idx == count - 1):
                    continue
                prev_idx = (idx - 1) % count if closed else idx - 1
                next_idx = (idx + 1) % count if closed else idx + 1
                px, py = source[prev_idx]
                x, y = source[idx]
                nx, ny = source[next_idx]
                avg_x = 0.5 * (px + nx)
                avg_y = 0.5 * (py + ny)
                updated[idx] = (
                    (1.0 - alpha) * x + alpha * avg_x,
                    (1.0 - alpha) * y + alpha * avg_y,
                )
            if duplicate_endpoint:
                updated[-1] = updated[0]
            smoothed = updated

        self.trajectory.points = smoothed
        self._mark_modified(geometry_dirty=True)
        self.redraw()
        self._set_status(f"smoothed all points alpha={alpha:.2f}, passes={passes}")

    def insert_point(self, x: float, y: float) -> None:
        segment_index = self._nearest_segment(x, y)
        if segment_index is None:
            return
        wx, wy = self.screen_to_world(x, y)
        self._push_undo()
        insert_index = segment_index + 1
        source_index = min(segment_index, len(self.trajectory.rows) - 1)
        self.trajectory.rows.insert(insert_index, copy.deepcopy(self.trajectory.rows[source_index]))
        self.trajectory.points.insert(insert_index, (wx, wy))
        self.selected_index = insert_index
        self._mark_modified(geometry_dirty=True, speed_dirty=True)
        self.redraw()
        self._set_status(f"inserted point {insert_index}")

    def delete_selected(self) -> None:
        if self.selected_index is None:
            return
        circular = bool(self.circular.get())
        duplicate_endpoint = circular and _closed_duplicate(self.trajectory.points)
        unique_count = len(self.trajectory.points) - (1 if duplicate_endpoint else 0)
        minimum_count = 3 if circular else 2
        if unique_count <= minimum_count:
            self._set_status("cannot delete: too few unique points")
            return
        if duplicate_endpoint and self.selected_index in (
            0,
            len(self.trajectory.points) - 1,
        ):
            self._set_status("cannot delete duplicated closure point")
            return
        self._push_undo()
        index = self.selected_index
        self.trajectory.rows.pop(index)
        self.trajectory.points.pop(index)
        self.selected_index = min(index, len(self.trajectory.points) - 1)
        self._mark_modified(geometry_dirty=True, speed_dirty=True)
        self.redraw()
        self._set_status(f"deleted point {index}")

    def nudge_selected(self, dx: float, dy: float, event: tk.Event) -> None:
        if self.selected_index is None:
            return
        step = 1.0 if event.state & 0x0001 else 0.1
        self._push_undo()
        self._apply_influenced_delta(
            list(self.trajectory.points),
            self.selected_index,
            dx * step,
            dy * step,
        )
        self._mark_modified(geometry_dirty=True)
        self.redraw()
        self._set_status(f"nudged {self.selected_index}")

    def recompute_derived_geometry(self) -> None:
        """Explicitly refresh geometry-derived CSV fields on a candidate copy."""

        self._push_undo()
        candidate = copy.deepcopy(self.trajectory)
        try:
            recompute_geometry(candidate, circular=bool(self.circular.get()))
            report = validate_trajectory_data(
                candidate,
                circular=bool(self.circular.get()),
            )
        except Exception as exc:  # noqa: BLE001
            self.undo_stack.pop()
            messagebox.showerror("Recompute Geometry failed", str(exc))
            return

        if not report.is_valid:
            self.undo_stack.pop()
            self._show_validation_report(
                report,
                current_document=False,
                source_label="recompute candidate",
            )
            messagebox.showerror(
                "Recompute Geometry rejected",
                f"The candidate has {report.error_count} validation error(s). "
                "Working data and Undo history were not changed.",
            )
            self._set_status("geometry candidate rejected")
            return

        self.trajectory.rows = candidate.rows
        self.trajectory.points = candidate.points
        self.dirty = True
        self.geometry_dirty = False
        if self.trajectory.format_name == "mpc":
            self.speed_dirty = True
        self.last_operation = "edited"
        self.candidate = None
        self.revision += 1
        self._clear_validation("Validation stale after geometry recompute")
        self._clear_clearance("Clearance: stale after geometry recompute")
        self._show_validation_report(report)
        self.redraw()
        self._set_status("derived geometry recomputed")

    def _confirm_unsaved_changes(self, action: str) -> bool:
        if not self.dirty:
            return True
        answer = messagebox.askyesnocancel(
            "Unsaved trajectory changes",
            f"The current trajectory has unsaved changes.\n\n"
            f"Save a copy before {action}?\n\n"
            "Yes: Save As\nNo: discard changes\nCancel: keep editing",
        )
        if answer is None:
            return False
        if answer:
            return self.save_as()
        return True

    def close(self) -> None:
        if self._confirm_unsaved_changes("closing the editor"):
            self.destroy()

    def open_trajectory(self) -> None:
        path = filedialog.askopenfilename(
            title="Open trajectory CSV",
            initialdir=str(self.trajectory.path.parent),
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        selected_path = Path(path)
        try:
            candidate, candidate_topology, source_report = _load_editor_trajectory(
                selected_path,
                circular=self.circular_override,
            )
        except Exception as exc:  # noqa: BLE001
            report = validate_csv_file(
                selected_path,
                circular=bool(self.circular.get()),
            )
            self._show_validation_report(report, current_document=False)
            self._set_status("selected trajectory rejected; current document unchanged")
            messagebox.showerror("Open trajectory failed", str(exc))
            return

        source_repairable = _is_normalization_repairable(
            candidate,
            source_report,
        )
        if not source_report.is_valid and not source_repairable:
            self._show_validation_report(source_report, current_document=False)
            self._set_status("selected trajectory rejected; current document unchanged")
            messagebox.showerror(
                "Open trajectory failed",
                f"Validation found {source_report.error_count} error(s). "
                "The current document was not replaced.",
            )
            return

        if not self._confirm_unsaved_changes("opening another trajectory"):
            return

        self.trajectory = candidate
        self.loaded_original = copy.deepcopy(candidate)
        self.original_trajectory = self.loaded_original
        self.loaded_original_circular = bool(candidate_topology)
        self.trajectory_path = selected_path
        self.circular.set(bool(candidate_topology))
        self.selected_index = None
        self.undo_stack.clear()
        self.dirty = False
        self.geometry_dirty = source_repairable
        self.speed_dirty = source_repairable
        self.last_operation = "edited"
        self.candidate = None
        self.revision += 1
        self._clear_validation("Not validated after open")
        self.clearance_config = None
        self._clear_clearance("Clearance: not run for newly opened trajectory")
        self.fit_view()
        self.validate_current()
        self._set_status("trajectory loaded")

    def open_osm(self) -> None:
        path = filedialog.askopenfilename(
            title="Open Lanelet2 OSM",
            initialdir=str(self.osm_path.parent),
            filetypes=[("OSM", "*.osm"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            self.osm_path = Path(path)
            self.rails = load_osm_rails(self.osm_path)
            self.fit_view()
            self._set_status("OSM loaded")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Open OSM failed", str(exc))

    @staticmethod
    def _suggested_filename(path: Path, operation: str) -> str:
        suffix = path.suffix or ".csv"
        stem = path.stem
        operation_suffix = {
            "normalize_geometry": "_normalized",
            "recompute_speed_profile": "_speed_profiled",
            "adjust_clearance": "_clearance_adjusted",
        }.get(operation, "_edited")
        for known_suffix in (
            "_clearance_adjusted",
            "_normalized",
            "_speed_profiled",
            "_edited",
        ):
            if stem.endswith(known_suffix):
                stem = stem[: -len(known_suffix)]
                break
        return stem + operation_suffix + suffix

    @staticmethod
    def _edited_filename(path: Path) -> str:
        """Compatibility helper retained for downstream imports/tests."""

        return TrajectoryEditor._suggested_filename(path, "edited")

    def _save_to_path(self, path: Path) -> bool:
        target = Path(path)
        stale_fields = []
        if self.geometry_dirty:
            stale_fields.append("geometry (s/heading/curvature or quaternion)")
        if self.speed_dirty:
            stale_fields.append("speed/acceleration metadata")
        if stale_fields:
            messagebox.showerror(
                "Save blocked",
                "Derived fields are stale:\n- "
                + "\n- ".join(stale_fields)
                + "\n\nRun Normalize Geometry or Recompute Geometry, then "
                "Recompute Speed Profile when speed metadata is stale.",
            )
            self._set_status("save blocked: stale derived fields")
            return False

        current_clearance = self.__dict__.get("clearance_report")
        current_clearance_revision = self.__dict__.get("clearance_revision")
        clearance_state = self.__dict__.get("clearance_state", "not_run")
        clearance_config = self.__dict__.get("clearance_config")
        if clearance_config is not None and clearance_state in {
            "running",
            "failed",
            "stale",
        }:
            messagebox.showerror(
                "Save blocked",
                "Wall-clearance state is "
                f"{clearance_state}. Run Validate Clearance successfully before saving.",
            )
            self._set_status(f"save blocked: clearance {clearance_state}")
            return False
        if (
            current_clearance is not None
            and current_clearance_revision == self.revision
            and not current_clearance.is_safe
        ):
            messagebox.showerror(
                "Save blocked",
                "The current wall-clearance report contains violations. Resolve "
                "them or change the explicit map/vehicle settings and validate again.",
            )
            self._set_status("save blocked: wall-clearance violations")
            return False
        if clearance_state == "safe":
            if (
                current_clearance is None
                or current_clearance_revision != self.revision
                or clearance_config is None
            ):
                messagebox.showerror(
                    "Save blocked",
                    "Wall-clearance SAFE state is stale or incomplete. Validate again.",
                )
                self._set_status("save blocked: inconsistent clearance state")
                return False
            try:
                fresh_grid = load_occupancy_grid(
                    clearance_config.map_yaml_path,
                    options=self._map_load_options(clearance_config),
                )
            except Exception as exc:  # noqa: BLE001
                self._clear_clearance(
                    "Clearance: map recheck failed before save",
                    state="failed",
                )
                messagebox.showerror(
                    "Save blocked",
                    f"Could not recheck the occupancy grid: {exc}",
                )
                self._set_status("save blocked: clearance map recheck failed")
                return False
            if fresh_grid.spec.signature != current_clearance.map_signature:
                self._clear_clearance(
                    "Clearance: map changed; validation required",
                    state="stale",
                )
                messagebox.showerror(
                    "Save blocked",
                    "The occupancy-grid content changed after the SAFE report. "
                    "Run Validate Clearance again.",
                )
                self._set_status("save blocked: clearance map changed")
                return False

        report = self.validate_current()
        if not report.is_valid:
            messagebox.showerror(
                "Save blocked",
                f"Validation found {report.error_count} error(s). "
                "Select an issue in the list for details.",
            )
            self._set_status("save blocked: validation errors")
            return False

        if target.is_symlink():
            messagebox.showerror(
                "Save blocked",
                f"Refusing to replace a symbolic link:\n{target}\n\n"
                "Use Save As and select a regular file in the source workspace.",
            )
            self._set_status("save blocked: symbolic-link target")
            return False

        target_exists = target.exists()
        warning_text = (
            f"\nValidation warnings: {report.warning_count}"
            if report.warning_count
            else ""
        )
        if target_exists or report.warning_count:
            resolved = target.resolve(strict=False)
            action = "Overwrite this file?" if target_exists else "Save this new file?"
            confirmed = messagebox.askyesno(
                "Confirm trajectory save",
                f"{action}\n\n"
                f"Target: {target}\n"
                f"Resolved: {resolved}\n"
                f"Format: {self.trajectory.format_name}\n"
                f"Points: {len(self.trajectory.points)}\n"
                f"Document revision: {self.revision}"
                "\nRuntime note: vx_mps/ax_mps2 are offline metadata; "
                "current C++ MPC runtime speed consumption is pending."
                f"{warning_text}",
            )
            if not confirmed:
                self._set_status("save cancelled")
                return False

        candidate = copy.deepcopy(self.trajectory)
        try:
            save_trajectory(
                candidate,
                target,
                recompute=False,
                circular=bool(self.circular.get()),
            )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Save failed", str(exc))
            self._set_status("save failed; existing file left unchanged")
            return False

        clearance_was_current = (
            current_clearance is not None
            and current_clearance_revision == self.revision
        )
        self.trajectory = candidate
        self.trajectory_path = target
        self.dirty = False
        self.geometry_dirty = False
        self.speed_dirty = False
        self.candidate = None
        self.undo_stack.clear()
        self.revision += 1
        if clearance_was_current:
            self.clearance_revision = self.revision
        self._clear_validation("Not validated after save")
        self.validate_current()
        self.redraw()
        self._set_status("saved atomically")
        return True

    def save(self) -> bool:
        """Explicit overwrite path; always confirms when the target exists."""

        return self._save_to_path(self.trajectory.path)

    def save_as(self) -> bool:
        """Default save path: create an operation-specific validated copy."""

        initialfile = self._suggested_filename(
            self.trajectory.path,
            self.last_operation,
        )
        path = filedialog.asksaveasfilename(
            title="Save trajectory CSV as a validated copy",
            initialdir=str(self.trajectory.path.parent),
            initialfile=initialfile,
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            self._set_status("save as cancelled")
            return False
        return self._save_to_path(Path(path))


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--preset",
        choices=("mpc", "pure_pursuit"),
        default="mpc",
        help="Default trajectory preset to open when --trajectory is omitted.",
    )
    parser.add_argument(
        "--trajectory",
        type=Path,
        default=None,
        help="Trajectory CSV path.",
    )
    parser.add_argument(
        "--osm",
        type=Path,
        default=None,
        help="Lanelet2 OSM path.",
    )
    topology = parser.add_mutually_exclusive_group()
    topology.add_argument(
        "--circular",
        dest="circular",
        action="store_true",
        help="Treat the trajectory as a circular path, independent of endpoint duplication.",
    )
    topology.add_argument(
        "--open",
        dest="circular",
        action="store_false",
        help="Treat the trajectory as an open path.",
    )
    parser.set_defaults(circular=None)
    args = parser.parse_args(argv)
    trajectory_was_omitted = args.trajectory is None
    args.circular_explicit = args.circular is not None
    default_traj, default_osm = _default_paths(args.preset)
    if args.trajectory is None:
        args.trajectory = default_traj
    if args.osm is None:
        args.osm = default_osm
    if args.circular is None and trajectory_was_omitted and args.preset == "mpc":
        # This package's built-in MPC raceline is a loop. This is a local preset,
        # not an Automotive AI Challenge 2026 interface requirement.
        args.circular = True
    return args


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    if args.trajectory is None:
        raise SystemExit("trajectory path is required")
    if args.osm is None:
        raise SystemExit("OSM path is required")
    try:
        app = TrajectoryEditor(
            args.trajectory,
            args.osm,
            circular=args.circular,
            circular_explicit=args.circular_explicit,
        )
    except Exception as error:  # noqa: BLE001
        raise SystemExit(f"trajectory editor startup failed: {error}") from error
    app.mainloop()


if __name__ == "__main__":
    main()
