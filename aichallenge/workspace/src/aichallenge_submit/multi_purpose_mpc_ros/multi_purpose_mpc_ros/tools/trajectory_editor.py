#!/usr/bin/env python3
"""Tkinter editor for trajectory CSV files over Lanelet2 OSM rails."""

from __future__ import annotations

import argparse
import copy
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
from typing import Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET


Point = Tuple[float, float]


@dataclass
class TrajectoryData:
    path: Path
    fieldnames: List[str]
    rows: List[Dict[str, str]]
    points: List[Point]
    x_column: str
    y_column: str
    format_name: str


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
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        fieldnames = list(reader.fieldnames)
        x_column, y_column, format_name = _detect_trajectory_columns(fieldnames, path)

        rows: List[Dict[str, str]] = []
        points: List[Point] = []
        for row in reader:
            try:
                x = float(row[x_column])
                y = float(row[y_column])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"Invalid {x_column}/{y_column} row in {path}") from exc
            rows.append(dict(row))
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


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _closed_duplicate(points: Sequence[Point]) -> bool:
    return len(points) > 2 and _distance(points[0], points[-1]) < 1e-5


def recompute_geometry(data: TrajectoryData) -> None:
    points = data.points
    n_points = len(points)
    if n_points < 2:
        return

    s_values = [0.0] * n_points
    for i in range(1, n_points):
        s_values[i] = s_values[i - 1] + _distance(points[i - 1], points[i])

    closed = _closed_duplicate(points)
    psi_values = [0.0] * n_points
    kappa_values = [0.0] * n_points

    for i in range(n_points):
        if closed and i == n_points - 1:
            continue

        prev_i = i - 1 if i > 0 else (n_points - 2 if closed else i)
        next_i = i + 1 if i < n_points - 1 else (1 if closed else i)
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

    if closed:
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


def save_trajectory(data: TrajectoryData, path: Path, recompute: bool = True) -> None:
    if recompute:
        recompute_geometry(data)
    else:
        for row, point in zip(data.rows, data.points):
            row[data.x_column] = f"{point[0]:.7f}"
            row[data.y_column] = f"{point[1]:.7f}"
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=data.fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(data.rows)
    data.path = path


class TrajectoryEditor(tk.Tk):
    def __init__(self, trajectory_path: Path, osm_path: Path) -> None:
        super().__init__()
        self.title("Trajectory Editor")
        self.geometry("1200x800")

        self.trajectory_path = trajectory_path
        self.osm_path = osm_path
        self.trajectory = load_trajectory(trajectory_path)
        self.rails = load_osm_rails(osm_path)

        self.center_x = 0.0
        self.center_y = 0.0
        self.scale = 5.0
        self.selected_index: Optional[int] = None
        self.dragging_point = False
        self.panning = False
        self.pan_anchor = (0, 0)
        self.undo_stack: List[Tuple[List[Dict[str, str]], List[Point], Optional[int]]] = []
        self.recompute_on_save = tk.BooleanVar(value=True)
        self.influence_radius_points = tk.IntVar(value=4)
        self.drag_origin_points: Optional[List[Point]] = None

        self._build_ui()
        self._bind_events()
        self.fit_view()

    def _build_ui(self) -> None:
        toolbar = tk.Frame(self)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        tk.Button(toolbar, text="Open Traj", command=self.open_trajectory).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Open OSM", command=self.open_osm).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Save", command=self.save).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Save As", command=self.save_as).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Undo", command=self.undo).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Fit", command=self.fit_view).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Label(toolbar, text="Influence pts").pack(side=tk.LEFT, padx=(8, 2))
        tk.Spinbox(
            toolbar,
            from_=0,
            to=30,
            width=4,
            textvariable=self.influence_radius_points,
            command=self.redraw,
        ).pack(side=tk.LEFT, padx=2)
        tk.Checkbutton(
            toolbar,
            text="Recompute geometry on save",
            variable=self.recompute_on_save,
        ).pack(side=tk.LEFT, padx=8)

        self.canvas = tk.Canvas(self, background="#101318")
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        self.status = tk.StringVar()
        tk.Label(self, textvariable=self.status, anchor="w").pack(side=tk.BOTTOM, fill=tk.X)
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
        self.bind("<f>", lambda _event: self.fit_view())
        self.bind("<Left>", lambda event: self.nudge_selected(-1.0, 0.0, event))
        self.bind("<Right>", lambda event: self.nudge_selected(1.0, 0.0, event))
        self.bind("<Up>", lambda event: self.nudge_selected(0.0, 1.0, event))
        self.bind("<Down>", lambda event: self.nudge_selected(0.0, -1.0, event))

    def _set_status(self, extra: str = "") -> None:
        selected = "--" if self.selected_index is None else str(self.selected_index)
        base = (
            f"traj={self.trajectory.path} | format={self.trajectory.format_name} | "
            f"osm={self.osm_path} | "
            f"points={len(self.trajectory.points)} | selected={selected} | "
            f"influence=+/-{self._influence_radius()}"
        )
        if extra:
            base = f"{base} | {extra}"
        self.status.set(base)

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
        points: List[Point] = list(self.trajectory.points)
        for rail in self.rails:
            points.extend(rail)
        if not points:
            return

        min_x = min(p[0] for p in points)
        max_x = max(p[0] for p in points)
        min_y = min(p[1] for p in points)
        max_y = max(p[1] for p in points)
        self.center_x = (min_x + max_x) * 0.5
        self.center_y = (min_y + max_y) * 0.5

        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        self.scale = 0.9 * min(width / span_x, height / span_y)
        self.redraw()

    def redraw(self) -> None:
        self.canvas.delete("all")

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

        traj_coords: List[float] = []
        for point in self.trajectory.points:
            sx, sy = self.world_to_screen(point)
            traj_coords.extend([sx, sy])
        if len(traj_coords) >= 4:
            self.canvas.create_line(
                *traj_coords,
                fill="#3aa0ff",
                width=2,
                smooth=False,
            )

        radius = 3.0
        influenced = self._influenced_indices(self.selected_index)
        for idx, point in enumerate(self.trajectory.points):
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

    def _push_undo(self) -> None:
        self.undo_stack.append(
            (
                copy.deepcopy(self.trajectory.rows),
                list(self.trajectory.points),
                self.selected_index,
            )
        )
        if len(self.undo_stack) > 30:
            self.undo_stack.pop(0)

    def undo(self) -> None:
        if not self.undo_stack:
            self._set_status("nothing to undo")
            return
        rows, points, selected = self.undo_stack.pop()
        self.trajectory.rows = rows
        self.trajectory.points = points
        self.selected_index = selected
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
        for i in range(len(points) - 1):
            ax, ay = self.world_to_screen(points[i])
            bx, by = self.world_to_screen(points[i + 1])
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
        if event.state & 0x0001:
            self.insert_point(event.x, event.y)
            return

        index = self._nearest_point(event.x, event.y)
        self.selected_index = index
        if index is not None:
            self._push_undo()
            self.drag_origin_points = list(self.trajectory.points)
            self.dragging_point = True
        self.redraw()
        self._set_status()

    def _on_left_drag(self, event: tk.Event) -> None:
        if not self.dragging_point or self.selected_index is None:
            return
        if self.drag_origin_points is None:
            self.drag_origin_points = list(self.trajectory.points)
        wx, wy = self.screen_to_world(event.x, event.y)
        origin_point = self.drag_origin_points[self.selected_index]
        self._apply_influenced_delta(
            self.drag_origin_points,
            self.selected_index,
            wx - origin_point[0],
            wy - origin_point[1],
        )
        self.redraw()
        self._set_status(f"x={wx:.3f}, y={wy:.3f}")

    def _on_left_up(self, _event: tk.Event) -> None:
        self.dragging_point = False
        self.drag_origin_points = None

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
        if _closed_duplicate(points) and index == len(points) - 1:
            return 0
        return index

    def _influenced_indices(self, selected_index: Optional[int]) -> set:
        if selected_index is None:
            return set()
        points = self.trajectory.points
        closed = _closed_duplicate(points)
        count = len(points) - 1 if closed else len(points)
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
        closed = _closed_duplicate(origin_points)
        count = len(origin_points) - 1 if closed else len(origin_points)
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

        if closed:
            updated[-1] = updated[0]
        self.trajectory.points = updated

    def _set_point(self, index: int, point: Point) -> None:
        points = self.trajectory.points
        closed = _closed_duplicate(points)
        points[index] = point
        if closed:
            if index == 0:
                points[-1] = point
            elif index == len(points) - 1:
                points[0] = point

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
        recompute_geometry(self.trajectory)
        self.redraw()
        self._set_status(f"inserted point {insert_index}")

    def delete_selected(self) -> None:
        if self.selected_index is None:
            return
        if len(self.trajectory.points) <= 3:
            self._set_status("cannot delete: too few points")
            return
        if _closed_duplicate(self.trajectory.points) and self.selected_index in (
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
        recompute_geometry(self.trajectory)
        self.redraw()
        self._set_status(f"deleted point {index}")

    def nudge_selected(self, dx: float, dy: float, event: tk.Event) -> None:
        if self.selected_index is None:
            return
        step = 1.0 if event.state & 0x0001 else 0.1
        self._push_undo()
        self._apply_influenced_delta(
            self.trajectory.points,
            self.selected_index,
            dx * step,
            dy * step,
        )
        self.redraw()
        self._set_status(f"nudged {self.selected_index}")

    def open_trajectory(self) -> None:
        path = filedialog.askopenfilename(
            title="Open trajectory CSV",
            initialdir=str(self.trajectory.path.parent),
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            self.trajectory = load_trajectory(Path(path))
            self.selected_index = None
            self.undo_stack.clear()
            self.fit_view()
            self._set_status("trajectory loaded")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Open trajectory failed", str(exc))

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

    def save(self) -> None:
        try:
            save_trajectory(
                self.trajectory,
                self.trajectory.path,
                recompute=self.recompute_on_save.get(),
            )
            self.redraw()
            self._set_status("saved")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Save failed", str(exc))

    def save_as(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Save trajectory CSV",
            initialdir=str(self.trajectory.path.parent),
            initialfile=self.trajectory.path.name,
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            save_trajectory(
                self.trajectory,
                Path(path),
                recompute=self.recompute_on_save.get(),
            )
            self.redraw()
            self._set_status("saved as")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Save As failed", str(exc))


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
    args = parser.parse_args(argv)
    default_traj, default_osm = _default_paths(args.preset)
    if args.trajectory is None:
        args.trajectory = default_traj
    if args.osm is None:
        args.osm = default_osm
    return args


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    if args.trajectory is None:
        raise SystemExit("trajectory path is required")
    if args.osm is None:
        raise SystemExit("OSM path is required")
    app = TrajectoryEditor(args.trajectory, args.osm)
    app.mainloop()


if __name__ == "__main__":
    main()
