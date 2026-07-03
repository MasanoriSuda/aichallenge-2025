#!/usr/bin/env python3
"""Tkinter GUI for placing virtual V2X obstacles on the course map."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from v2x_msgs.msg import V2XVehiclePosition, V2XVehiclePositionArray

from multi_purpose_mpc_ros.tools.trajectory_editor import (
    Point,
    _default_paths,
    load_osm_rails,
    load_trajectory,
)


@dataclass
class VirtualObstacle:
    vehicle_id: str
    x: float
    y: float
    z: float = 0.0


class VirtualV2XObstacleGui(tk.Tk):
    def __init__(
        self,
        node: Node,
        trajectory_path: Optional[Path],
        osm_path: Optional[Path],
        frame_id: str,
        vehicle_id: str,
        rate_hz: float,
    ) -> None:
        super().__init__()
        self.title("Virtual V2X Obstacle GUI")
        self.geometry("1200x800")

        self.node = node
        self.frame_id = frame_id
        self.rails: List[List[Point]] = []
        self.trajectory: List[Point] = []
        self.obstacles: List[VirtualObstacle] = []
        self.selected_index: Optional[int] = None
        self.dragging_obstacle = False
        self.panning = False
        self.last_mouse = (0, 0)

        self.scale = 1.0
        self.center_x = 0.0
        self.center_y = 0.0

        self.vehicle_id = tk.StringVar(value=vehicle_id)
        self.z_value = tk.DoubleVar(value=0.0)
        self.publish_enabled = tk.BooleanVar(value=True)
        self.rate_hz = tk.DoubleVar(value=max(rate_hz, 0.1))
        self.status_text = tk.StringVar(value="ready")
        self.coord_text = tk.StringVar(value="")
        self._clock_wait_logged = False

        self.pub = self.node.create_publisher(
            V2XVehiclePositionArray,
            "/v2x/vehicle_positions",
            1,
        )

        self._load_initial_data(trajectory_path, osm_path)
        self._build_ui()
        self.fit_view()
        self._schedule_ros_spin()
        self._schedule_publish()

    def _build_ui(self) -> None:
        toolbar = tk.Frame(self)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        tk.Label(toolbar, text="ID").pack(side=tk.LEFT, padx=(8, 2))
        tk.Entry(toolbar, textvariable=self.vehicle_id, width=18).pack(
            side=tk.LEFT, padx=(0, 8))

        tk.Label(toolbar, text="Z").pack(side=tk.LEFT, padx=(0, 2))
        tk.Spinbox(
            toolbar,
            from_=-20.0,
            to=20.0,
            increment=0.1,
            textvariable=self.z_value,
            width=6,
        ).pack(side=tk.LEFT, padx=(0, 8))

        tk.Checkbutton(
            toolbar,
            text="Publish",
            variable=self.publish_enabled,
        ).pack(side=tk.LEFT, padx=(0, 8))

        tk.Label(toolbar, text="Hz").pack(side=tk.LEFT, padx=(0, 2))
        tk.Spinbox(
            toolbar,
            from_=0.5,
            to=50.0,
            increment=0.5,
            textvariable=self.rate_hz,
            width=6,
        ).pack(side=tk.LEFT, padx=(0, 8))

        tk.Button(toolbar, text="Delete", command=self.delete_selected).pack(
            side=tk.LEFT, padx=(0, 4))
        tk.Button(toolbar, text="Clear", command=self.clear_obstacles).pack(
            side=tk.LEFT, padx=(0, 4))
        tk.Button(toolbar, text="Fit", command=self.fit_view).pack(
            side=tk.LEFT, padx=(0, 4))
        tk.Button(toolbar, text="Open CSV", command=self.open_trajectory).pack(
            side=tk.LEFT, padx=(0, 4))
        tk.Button(toolbar, text="Open OSM", command=self.open_osm).pack(
            side=tk.LEFT, padx=(0, 8))

        tk.Label(toolbar, textvariable=self.coord_text).pack(
            side=tk.RIGHT, padx=(8, 8))

        self.canvas = tk.Canvas(self, background="#111111", highlightthickness=0)
        self.canvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.redraw())
        self.canvas.bind("<ButtonPress-1>", self._on_left_press)
        self.canvas.bind("<B1-Motion>", self._on_left_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_left_release)
        self.canvas.bind("<ButtonPress-3>", self._on_right_press)
        self.canvas.bind("<B3-Motion>", self._on_right_drag)
        self.canvas.bind("<ButtonRelease-3>", self._on_right_release)
        self.canvas.bind("<Motion>", self._on_motion)
        self.canvas.bind("<MouseWheel>", self._on_mousewheel)
        self.canvas.bind("<Button-4>", self._on_mousewheel)
        self.canvas.bind("<Button-5>", self._on_mousewheel)

        status = tk.Label(self, textvariable=self.status_text, anchor="w")
        status.pack(side=tk.BOTTOM, fill=tk.X)

    def _load_initial_data(
        self,
        trajectory_path: Optional[Path],
        osm_path: Optional[Path],
    ) -> None:
        if trajectory_path is not None:
            try:
                self.trajectory = load_trajectory(trajectory_path).points
            except Exception as exc:  # noqa: BLE001
                self.status_text.set(f"trajectory load failed: {exc}")
        if osm_path is not None:
            try:
                self.rails = load_osm_rails(osm_path)
            except Exception as exc:  # noqa: BLE001
                self.status_text.set(f"OSM load failed: {exc}")

    def _all_points(self) -> List[Point]:
        points: List[Point] = []
        points.extend(self.trajectory)
        for rail in self.rails:
            points.extend(rail)
        points.extend((ob.x, ob.y) for ob in self.obstacles)
        return points

    def fit_view(self) -> None:
        points = self._all_points()
        if not points:
            return
        min_x = min(p[0] for p in points)
        max_x = max(p[0] for p in points)
        min_y = min(p[1] for p in points)
        max_y = max(p[1] for p in points)
        self.center_x = (min_x + max_x) * 0.5
        self.center_y = (min_y + max_y) * 0.5
        width = max(self.canvas.winfo_width(), 100)
        height = max(self.canvas.winfo_height(), 100)
        span_x = max(max_x - min_x, 1.0)
        span_y = max(max_y - min_y, 1.0)
        self.scale = 0.88 * min(width / span_x, height / span_y)
        self.redraw()

    def world_to_screen(self, point: Point) -> Tuple[float, float]:
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

    def redraw(self) -> None:
        if not hasattr(self, "canvas"):
            return
        self.canvas.delete("all")
        self._draw_polylines(self.rails, fill="#5f666d", width=1)
        self._draw_polylines([self.trajectory], fill="#2f8bff", width=2)
        for index, obstacle in enumerate(self.obstacles):
            self._draw_obstacle(index, obstacle)

    def _draw_polylines(
        self,
        polylines: Sequence[Sequence[Point]],
        fill: str,
        width: int,
    ) -> None:
        for points in polylines:
            if len(points) < 2:
                continue
            coords: List[float] = []
            for point in points:
                sx, sy = self.world_to_screen(point)
                coords.extend([sx, sy])
            self.canvas.create_line(*coords, fill=fill, width=width)

    def _draw_obstacle(self, index: int, obstacle: VirtualObstacle) -> None:
        sx, sy = self.world_to_screen((obstacle.x, obstacle.y))
        radius_px = max(7.0, 0.75 * self.scale)
        fill = "#ff4040" if index == self.selected_index else "#ff9f1c"
        outline = "#ffffff" if index == self.selected_index else "#111111"
        self.canvas.create_oval(
            sx - radius_px,
            sy - radius_px,
            sx + radius_px,
            sy + radius_px,
            fill=fill,
            outline=outline,
            width=2,
        )
        self.canvas.create_text(
            sx + radius_px + 4,
            sy,
            text=obstacle.vehicle_id,
            fill="#ffffff",
            anchor="w",
        )

    def _nearest_obstacle(self, x: float, y: float) -> Optional[int]:
        best_index = None
        best_dist = 18.0
        for index, obstacle in enumerate(self.obstacles):
            sx, sy = self.world_to_screen((obstacle.x, obstacle.y))
            dist = math.hypot(sx - x, sy - y)
            if dist < best_dist:
                best_index = index
                best_dist = dist
        return best_index

    def _next_vehicle_id(self) -> str:
        raw = self.vehicle_id.get().strip() or "virtual_obstacle"
        existing = {ob.vehicle_id for ob in self.obstacles}
        if raw not in existing:
            return raw
        suffix = 2
        while f"{raw}_{suffix}" in existing:
            suffix += 1
        return f"{raw}_{suffix}"

    def _place_obstacle(self, x: float, y: float) -> None:
        wx, wy = self.screen_to_world(x, y)
        if self.selected_index is None:
            self.obstacles.append(
                VirtualObstacle(
                    vehicle_id=self._next_vehicle_id(),
                    x=wx,
                    y=wy,
                    z=float(self.z_value.get()),
                )
            )
            self.selected_index = len(self.obstacles) - 1
        else:
            obstacle = self.obstacles[self.selected_index]
            obstacle.x = wx
            obstacle.y = wy
            obstacle.z = float(self.z_value.get())
        self.status_text.set(
            f"selected {self.obstacles[self.selected_index].vehicle_id} "
            f"at x={wx:.2f}, y={wy:.2f}")
        self.redraw()

    def _on_left_press(self, event: tk.Event) -> None:
        self.last_mouse = (event.x, event.y)
        nearest = self._nearest_obstacle(event.x, event.y)
        if nearest is not None:
            self.selected_index = nearest
            self.dragging_obstacle = True
            self.redraw()
            return
        self.selected_index = None
        self._place_obstacle(event.x, event.y)
        self.dragging_obstacle = True

    def _on_left_drag(self, event: tk.Event) -> None:
        if self.dragging_obstacle and self.selected_index is not None:
            self._place_obstacle(event.x, event.y)

    def _on_left_release(self, _event: tk.Event) -> None:
        self.dragging_obstacle = False

    def _on_right_press(self, event: tk.Event) -> None:
        self.panning = True
        self.last_mouse = (event.x, event.y)

    def _on_right_drag(self, event: tk.Event) -> None:
        if not self.panning:
            return
        dx = event.x - self.last_mouse[0]
        dy = event.y - self.last_mouse[1]
        self.center_x -= dx / self.scale
        self.center_y += dy / self.scale
        self.last_mouse = (event.x, event.y)
        self.redraw()

    def _on_right_release(self, _event: tk.Event) -> None:
        self.panning = False

    def _on_motion(self, event: tk.Event) -> None:
        wx, wy = self.screen_to_world(event.x, event.y)
        self.coord_text.set(f"x={wx:.2f} y={wy:.2f}")

    def _on_mousewheel(self, event: tk.Event) -> None:
        factor = 1.12
        if getattr(event, "num", None) == 5 or getattr(event, "delta", 0) < 0:
            factor = 1.0 / factor
        before = self.screen_to_world(event.x, event.y)
        self.scale = max(0.02, min(self.scale * factor, 200.0))
        width = max(self.canvas.winfo_width(), 1)
        height = max(self.canvas.winfo_height(), 1)
        self.center_x = before[0] - (event.x - width * 0.5) / self.scale
        self.center_y = before[1] - (height * 0.5 - event.y) / self.scale
        self.redraw()

    def delete_selected(self) -> None:
        if self.selected_index is None:
            return
        removed = self.obstacles.pop(self.selected_index)
        self.selected_index = None
        self.status_text.set(f"deleted {removed.vehicle_id}")
        self.redraw()

    def clear_obstacles(self) -> None:
        self.obstacles.clear()
        self.selected_index = None
        self.status_text.set("cleared")
        self.redraw()

    def open_trajectory(self) -> None:
        path = filedialog.askopenfilename(
            title="Open trajectory CSV",
            filetypes=[("CSV", "*.csv"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            self.trajectory = load_trajectory(Path(path)).points
            self.fit_view()
            self.status_text.set("trajectory loaded")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Open trajectory failed", str(exc))

    def open_osm(self) -> None:
        path = filedialog.askopenfilename(
            title="Open Lanelet2 OSM",
            filetypes=[("OSM", "*.osm"), ("All files", "*.*")],
        )
        if not path:
            return
        try:
            self.rails = load_osm_rails(Path(path))
            self.fit_view()
            self.status_text.set("OSM loaded")
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Open OSM failed", str(exc))

    def _schedule_ros_spin(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.after(20, self._schedule_ros_spin)

    def _schedule_publish(self) -> None:
        self._publish()
        try:
            rate = max(0.1, float(self.rate_hz.get()))
        except (tk.TclError, ValueError):
            rate = 10.0
        self.after(max(20, int(1000.0 / rate)), self._schedule_publish)

    def _publish(self) -> None:
        if not self.publish_enabled.get() or not self.obstacles:
            return
        stamp = self.node.get_clock().now().to_msg()
        if stamp.sec == 0 and stamp.nanosec == 0:
            if not self._clock_wait_logged:
                self._clock_wait_logged = True
                self.status_text.set("waiting for /clock")
            return

        msg = V2XVehiclePositionArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        for obstacle in self.obstacles:
            vehicle = V2XVehiclePosition()
            vehicle.header.stamp = stamp
            vehicle.header.frame_id = self.frame_id
            vehicle.vehicle_id = obstacle.vehicle_id
            vehicle.position.x = obstacle.x
            vehicle.position.y = obstacle.y
            vehicle.position.z = obstacle.z
            vehicle.covariance.x = 0.2
            vehicle.covariance.y = 0.2
            vehicle.covariance.z = 0.2
            msg.vehicles.append(vehicle)
        self.pub.publish(msg)


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    default_traj, default_osm = _default_paths()
    parser = argparse.ArgumentParser(
        description="Place virtual V2X obstacles with a GUI.")
    parser.add_argument("--trajectory", type=Path, default=default_traj)
    parser.add_argument("--osm", type=Path, default=default_osm)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--vehicle-id", default="force_obstacle")
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--use-wall-time", action="store_true")
    return parser.parse_args(remove_ros_args(args=list(argv))[1:])


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(list(argv) if argv is not None else [])
    rclpy.init(args=list(argv) if argv is not None else None)
    parameter_overrides = []
    if not args.use_wall_time:
        parameter_overrides.append(
            Parameter("use_sim_time", Parameter.Type.BOOL, True))
    node = Node(
        "virtual_v2x_obstacle_gui",
        parameter_overrides=parameter_overrides,
    )
    app = VirtualV2XObstacleGui(
        node=node,
        trajectory_path=args.trajectory,
        osm_path=args.osm,
        frame_id=args.frame_id,
        vehicle_id=args.vehicle_id,
        rate_hz=args.rate,
    )
    try:
        app.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
