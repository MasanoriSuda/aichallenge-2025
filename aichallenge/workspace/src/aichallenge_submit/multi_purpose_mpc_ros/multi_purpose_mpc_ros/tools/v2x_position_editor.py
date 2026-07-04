#!/usr/bin/env python3
"""Tkinter V2X position publisher over the Lanelet2 map."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
from typing import List
from typing import Optional
from typing import Sequence
from typing import Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Vector3
from v2x_msgs.msg import V2XVehiclePosition
from v2x_msgs.msg import V2XVehiclePositionArray

from multi_purpose_mpc_ros.tools.trajectory_editor import Point
from multi_purpose_mpc_ros.tools.trajectory_editor import _default_paths
from multi_purpose_mpc_ros.tools.trajectory_editor import load_osm_rails
from multi_purpose_mpc_ros.tools.trajectory_editor import load_trajectory


@dataclass
class VirtualVehicle:
    vehicle_id: str
    x: float
    y: float
    z: float = 0.0
    covariance_x: float = 0.0
    covariance_y: float = 0.0
    covariance_z: float = 0.0


class V2XPositionEditor(tk.Tk):
    def __init__(
        self,
        node: Node,
        trajectory_path: Optional[Path],
        osm_path: Optional[Path],
        publish_rate_hz: float,
        default_covariance: float,
        display_radius: float,
    ) -> None:
        super().__init__()
        self.title("V2X Position Editor")
        self.geometry("1200x800")

        self.node = node
        self.publisher = node.create_publisher(V2XVehiclePositionArray, "/v2x/vehicle_positions", 1)
        self.trajectory_path = trajectory_path
        self.osm_path = osm_path
        self.trajectory_points: List[Point] = []
        self.rails: List[List[Point]] = []
        self.vehicles: List[VirtualVehicle] = []

        self.center_x = 0.0
        self.center_y = 0.0
        self.scale = 5.0
        self.selected_index: Optional[int] = None
        self.dragging_vehicle = False
        self.panning = False
        self.pan_anchor = (0, 0)
        self.publish_enabled = tk.BooleanVar(value=True)
        self.publish_rate_hz = tk.DoubleVar(value=max(0.1, publish_rate_hz))
        self.default_covariance = tk.DoubleVar(value=max(0.0, default_covariance))
        self.display_radius = tk.DoubleVar(value=max(0.1, display_radius))
        self.selected_id = tk.StringVar(value="")
        self.last_publish_time_sec = 0.0

        self._load_backgrounds()
        self._build_ui()
        self._bind_events()
        self.fit_view()
        self.after(20, self._spin_ros)
        self.after(50, self._publish_tick)

    def _load_backgrounds(self) -> None:
        if self.trajectory_path is not None and self.trajectory_path.exists():
            try:
                self.trajectory_points = list(load_trajectory(self.trajectory_path).points)
            except Exception as exc:  # noqa: BLE001
                messagebox.showwarning("Trajectory", f"Failed to load trajectory:\n{exc}")
                self.trajectory_points = []

        if self.osm_path is not None and self.osm_path.exists():
            try:
                self.rails = load_osm_rails(self.osm_path)
            except Exception as exc:  # noqa: BLE001
                messagebox.showwarning("OSM", f"Failed to load OSM:\n{exc}")
                self.rails = []

    def _build_ui(self) -> None:
        toolbar = tk.Frame(self)
        toolbar.pack(side=tk.TOP, fill=tk.X)

        tk.Button(toolbar, text="Open Traj", command=self.open_trajectory).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Open OSM", command=self.open_osm).pack(
            side=tk.LEFT, padx=2, pady=2
        )
        tk.Button(toolbar, text="Load", command=self.load_scene).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Button(toolbar, text="Save", command=self.save_scene).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Button(toolbar, text="Fit", command=self.fit_view).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Button(toolbar, text="Clear", command=self.clear_vehicles).pack(side=tk.LEFT, padx=2, pady=2)
        tk.Button(toolbar, text="Delete", command=self.delete_selected).pack(side=tk.LEFT, padx=2, pady=2)

        tk.Checkbutton(toolbar, text="Publish", variable=self.publish_enabled).pack(
            side=tk.LEFT, padx=(12, 2)
        )
        tk.Label(toolbar, text="Hz").pack(side=tk.LEFT, padx=(8, 2))
        tk.Spinbox(
            toolbar,
            from_=0.1,
            to=100.0,
            increment=1.0,
            width=5,
            textvariable=self.publish_rate_hz,
        ).pack(side=tk.LEFT, padx=2)
        tk.Label(toolbar, text="Cov").pack(side=tk.LEFT, padx=(8, 2))
        tk.Spinbox(
            toolbar,
            from_=0.0,
            to=20.0,
            increment=0.1,
            width=5,
            textvariable=self.default_covariance,
        ).pack(side=tk.LEFT, padx=2)
        tk.Label(toolbar, text="Draw r").pack(side=tk.LEFT, padx=(8, 2))
        tk.Spinbox(
            toolbar,
            from_=0.1,
            to=20.0,
            increment=0.1,
            width=5,
            textvariable=self.display_radius,
            command=self.redraw,
        ).pack(side=tk.LEFT, padx=2)
        tk.Label(toolbar, text="Selected ID").pack(side=tk.LEFT, padx=(12, 2))
        tk.Entry(toolbar, textvariable=self.selected_id, width=10).pack(side=tk.LEFT, padx=2)
        tk.Button(toolbar, text="Apply ID", command=self.apply_selected_id).pack(
            side=tk.LEFT, padx=2, pady=2
        )

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
        self.bind("<Control-s>", lambda _event: self.save_scene())
        self.bind("<Control-o>", lambda _event: self.load_scene())
        self.bind("<f>", lambda _event: self.fit_view())

    def _set_status(self, extra: str = "") -> None:
        selected = "--"
        if self.selected_index is not None and 0 <= self.selected_index < len(self.vehicles):
            selected = self.vehicles[self.selected_index].vehicle_id
        base = (
            f"topic=/v2x/vehicle_positions | vehicles={len(self.vehicles)} | "
            f"selected={selected} | publish={self.publish_enabled.get()} | "
            f"traj={self.trajectory_path or '--'} | osm={self.osm_path or '--'}"
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
        points: List[Point] = list(self.trajectory_points)
        for rail in self.rails:
            points.extend(rail)
        points.extend((vehicle.x, vehicle.y) for vehicle in self.vehicles)
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
                self.canvas.create_line(*coords, fill="#6f7782", width=1, smooth=False)

        traj_coords: List[float] = []
        for point in self.trajectory_points:
            sx, sy = self.world_to_screen(point)
            traj_coords.extend([sx, sy])
        if len(traj_coords) >= 4:
            self.canvas.create_line(*traj_coords, fill="#3aa0ff", width=2, smooth=False)

        draw_radius_px = max(4.0, self._display_radius() * self.scale)
        for idx, vehicle in enumerate(self.vehicles):
            sx, sy = self.world_to_screen((vehicle.x, vehicle.y))
            selected = idx == self.selected_index
            fill = "#ffb02e" if selected else self._vehicle_color(vehicle.vehicle_id)
            outline = "#ffffff" if selected else "#1b2735"
            self.canvas.create_oval(
                sx - draw_radius_px,
                sy - draw_radius_px,
                sx + draw_radius_px,
                sy + draw_radius_px,
                fill=fill,
                outline=outline,
                width=2,
                stipple="" if selected else "gray25",
            )
            self.canvas.create_oval(
                sx - 5.0,
                sy - 5.0,
                sx + 5.0,
                sy + 5.0,
                fill=fill,
                outline="#ffffff",
                width=1,
            )
            self.canvas.create_text(
                sx + 8.0,
                sy - 8.0,
                text=vehicle.vehicle_id,
                anchor="sw",
                fill="#e8eef7",
                font=("TkDefaultFont", 10, "bold"),
            )

    def _vehicle_color(self, vehicle_id: str) -> str:
        return {
            "d1": "#3366ff",
            "d2": "#ffe533",
            "d3": "#33ff66",
            "d4": "#ff3333",
        }.get(vehicle_id, "#f0f3f7")

    def _display_radius(self) -> float:
        try:
            return max(0.1, float(self.display_radius.get()))
        except (tk.TclError, ValueError):
            return 1.2

    def _covariance(self) -> float:
        try:
            return max(0.0, float(self.default_covariance.get()))
        except (tk.TclError, ValueError):
            return 0.0

    def _publish_rate_hz(self) -> float:
        try:
            return max(0.1, float(self.publish_rate_hz.get()))
        except (tk.TclError, ValueError):
            return 10.0

    def _next_vehicle_id(self) -> str:
        preferred = ["d2", "d3", "d4", "d1"]
        used = {vehicle.vehicle_id for vehicle in self.vehicles}
        for vehicle_id in preferred:
            if vehicle_id not in used:
                return vehicle_id
        index = len(self.vehicles) + 1
        while f"debug_{index}" in used:
            index += 1
        return f"debug_{index}"

    def _nearest_vehicle(self, x: float, y: float, max_px: float = 18.0) -> Optional[int]:
        best_index: Optional[int] = None
        best_dist_sq = max_px * max_px
        for idx, vehicle in enumerate(self.vehicles):
            sx, sy = self.world_to_screen((vehicle.x, vehicle.y))
            dist_sq = (sx - x) ** 2 + (sy - y) ** 2
            if dist_sq <= best_dist_sq:
                best_dist_sq = dist_sq
                best_index = idx
        return best_index

    def _on_left_down(self, event: tk.Event) -> None:
        self.focus_set()
        index = self._nearest_vehicle(event.x, event.y)
        if index is None:
            wx, wy = self.screen_to_world(event.x, event.y)
            covariance = self._covariance()
            self.vehicles.append(
                VirtualVehicle(
                    vehicle_id=self._next_vehicle_id(),
                    x=wx,
                    y=wy,
                    covariance_x=covariance,
                    covariance_y=covariance,
                    covariance_z=0.0,
                )
            )
            index = len(self.vehicles) - 1
        self.selected_index = index
        self.selected_id.set(self.vehicles[index].vehicle_id)
        self.dragging_vehicle = True
        self.redraw()
        self._set_status()

    def _on_left_drag(self, event: tk.Event) -> None:
        if not self.dragging_vehicle or self.selected_index is None:
            return
        if not (0 <= self.selected_index < len(self.vehicles)):
            return
        wx, wy = self.screen_to_world(event.x, event.y)
        vehicle = self.vehicles[self.selected_index]
        vehicle.x = wx
        vehicle.y = wy
        self.redraw()
        self._set_status(f"x={wx:.3f}, y={wy:.3f}")

    def _on_left_up(self, _event: tk.Event) -> None:
        self.dragging_vehicle = False

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

    def apply_selected_id(self) -> None:
        if self.selected_index is None or not (0 <= self.selected_index < len(self.vehicles)):
            return
        new_id = self.selected_id.get().strip()
        if not new_id:
            messagebox.showwarning("Vehicle ID", "vehicle_id must not be empty")
            return
        self.vehicles[self.selected_index].vehicle_id = new_id
        self.redraw()
        self._set_status("vehicle_id updated")

    def delete_selected(self) -> None:
        if self.selected_index is None or not (0 <= self.selected_index < len(self.vehicles)):
            return
        del self.vehicles[self.selected_index]
        self.selected_index = None
        self.selected_id.set("")
        self.redraw()
        self._set_status("deleted")

    def clear_vehicles(self) -> None:
        if self.vehicles and not messagebox.askyesno("Clear", "Clear all virtual V2X vehicles?"):
            return
        self.vehicles.clear()
        self.selected_index = None
        self.selected_id.set("")
        self.redraw()
        self._set_status("cleared")

    def open_trajectory(self) -> None:
        initial = self.trajectory_path.parent if self.trajectory_path is not None else Path.cwd()
        filename = filedialog.askopenfilename(
            title="Open trajectory CSV",
            initialdir=str(initial),
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return
        self.trajectory_path = Path(filename)
        try:
            self.trajectory_points = list(load_trajectory(self.trajectory_path).points)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Open Trajectory", str(exc))
            self.trajectory_points = []
        self.fit_view()
        self._set_status()

    def open_osm(self) -> None:
        initial = self.osm_path.parent if self.osm_path is not None else Path.cwd()
        filename = filedialog.askopenfilename(
            title="Open Lanelet2 OSM",
            initialdir=str(initial),
            filetypes=[("OSM files", "*.osm"), ("All files", "*.*")],
        )
        if not filename:
            return
        self.osm_path = Path(filename)
        try:
            self.rails = load_osm_rails(self.osm_path)
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Open OSM", str(exc))
            self.rails = []
        self.fit_view()
        self._set_status()

    def save_scene(self) -> None:
        filename = filedialog.asksaveasfilename(
            title="Save V2X positions",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if not filename:
            return
        payload = {
            "version": 1,
            "topic": "/v2x/vehicle_positions",
            "vehicles": [asdict(vehicle) for vehicle in self.vehicles],
        }
        Path(filename).write_text(json.dumps(payload, indent=2) + "\n")
        self._set_status(f"saved {filename}")

    def load_scene(self) -> None:
        filename = filedialog.askopenfilename(
            title="Load V2X positions",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
        )
        if not filename:
            return
        try:
            payload = json.loads(Path(filename).read_text())
            vehicles = []
            for item in payload.get("vehicles", []):
                vehicles.append(
                    VirtualVehicle(
                        vehicle_id=str(item["vehicle_id"]),
                        x=float(item["x"]),
                        y=float(item["y"]),
                        z=float(item.get("z", 0.0)),
                        covariance_x=float(item.get("covariance_x", 0.0)),
                        covariance_y=float(item.get("covariance_y", 0.0)),
                        covariance_z=float(item.get("covariance_z", 0.0)),
                    )
                )
        except Exception as exc:  # noqa: BLE001
            messagebox.showerror("Load V2X positions", str(exc))
            return
        self.vehicles = vehicles
        self.selected_index = None
        self.selected_id.set("")
        self.fit_view()
        self._set_status(f"loaded {filename}")

    def _spin_ros(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            self.after(20, self._spin_ros)

    def _publish_tick(self) -> None:
        if not rclpy.ok():
            return
        now_sec = self.node.get_clock().now().nanoseconds * 1e-9
        period = 1.0 / self._publish_rate_hz()
        if self.publish_enabled.get() and now_sec - self.last_publish_time_sec >= period:
            self.publish()
            self.last_publish_time_sec = now_sec
        self.after(20, self._publish_tick)

    def publish(self) -> None:
        stamp = self.node.get_clock().now().to_msg()
        msg = V2XVehiclePositionArray()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        for vehicle in self.vehicles:
            item = V2XVehiclePosition()
            item.header.stamp = stamp
            item.header.frame_id = "map"
            item.vehicle_id = vehicle.vehicle_id
            item.position = PointMsg(x=vehicle.x, y=vehicle.y, z=vehicle.z)
            item.covariance = Vector3(
                x=vehicle.covariance_x,
                y=vehicle.covariance_y,
                z=vehicle.covariance_z,
            )
            msg.vehicles.append(item)
        self.publisher.publish(msg)
        self._set_status(f"published {len(msg.vehicles)} vehicles")

    def on_close(self) -> None:
        self.destroy()


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    default_trajectory, default_osm = _default_paths("mpc")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path, default=default_trajectory)
    parser.add_argument("--osm", type=Path, default=default_osm)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--covariance", type=float, default=0.0)
    parser.add_argument("--display-radius", type=float, default=1.25)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> None:
    args = parse_args(argv)
    rclpy.init(args=None)
    node = rclpy.create_node("v2x_position_editor")
    app = V2XPositionEditor(
        node=node,
        trajectory_path=args.trajectory,
        osm_path=args.osm,
        publish_rate_hz=args.rate_hz,
        default_covariance=args.covariance,
        display_radius=args.display_radius,
    )
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    try:
        app.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
