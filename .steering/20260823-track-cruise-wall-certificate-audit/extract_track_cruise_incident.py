#!/usr/bin/env python3
"""Extract reproducible Track/Cruise incident evidence from an AI Challenge bag."""

from __future__ import annotations

import argparse
import bisect
import math
from pathlib import Path
import sys

import cv2
import yaml


TOOLS_DIR = Path(__file__).resolve().parents[2] / (
    "aichallenge/workspace/src/aichallenge_submit/"
    "multi_purpose_mpc_ros/tools/localization_scope"
)
sys.path.insert(0, str(TOOLS_DIR))

from localization_scope.bag_reader import read_bag  # noqa: E402


TOPICS = {
    "pitstop_condition": "/aichallenge/pitstop/condition",
    "ekf_pose": "/localization/kinematic_state",
    "control": "/control/command/control_cmd",
    "imu_raw": "/sensing/imu/imu_raw",
    "vehicle_velocity": "/vehicle/status/velocity_status",
    "steering": "/vehicle/status/steering_status",
    "runtime_trajectory": "/planning/scenario_planning/trajectory",
}

DEFAULT_MAP = Path(
    "/aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/"
    "env/final_ver3/occupancy_grid_map.yaml"
)


class WallGrid:
    def __init__(self, yaml_path: Path):
        metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        image = cv2.imread(str(yaml_path.parent / metadata["image"]), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError(f"cannot load wall map {yaml_path}")
        if len(image.shape) > 2:
            image = image[:, :, 0]
        normalized = image.astype(float) / float(image.max())
        occupancy = 1.0 - normalized if int(metadata.get("negate", 0)) == 0 else normalized
        self.free = occupancy < float(metadata["free_thresh"])
        self.resolution = float(metadata["resolution"])
        self.origin_x = float(metadata["origin"][0])
        self.origin_y = float(metadata["origin"][1])
        self.height, self.width = self.free.shape

    def footprint_clear(self, x: float, y: float, yaw: float, extra: float, isotropic: bool):
        margin = 0.05
        front = 1.49 + margin + (extra if isotropic else 0.0)
        rear = 0.51 + margin + (extra if isotropic else 0.0)
        left = 0.725 + margin + extra
        right = 0.725 + margin + extra
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -forward_y
        left_y = forward_x
        center_x = x + 0.5 * (front - rear) * forward_x + 0.5 * (left - right) * left_x
        center_y = y + 0.5 * (front - rear) * forward_y + 0.5 * (left - right) * left_y
        half_forward = 0.5 * (front + rear)
        half_left = 0.5 * (left + right)
        corners = [
            (
                center_x + fsign * half_forward * forward_x + lsign * half_left * left_x,
                center_y + fsign * half_forward * forward_y + lsign * half_left * left_y,
            )
            for fsign in (-1.0, 1.0)
            for lsign in (-1.0, 1.0)
        ]
        cell_half = 0.5 * self.resolution
        min_x = min(point[0] for point in corners)
        max_x = max(point[0] for point in corners)
        min_y = min(point[1] for point in corners)
        max_y = max(point[1] for point in corners)
        min_column = max(0, math.floor((min_x - self.origin_x) / self.resolution) - 1)
        max_column = min(self.width - 1, math.ceil((max_x - self.origin_x) / self.resolution) + 1)
        min_y_index = max(0, math.floor((min_y - self.origin_y) / self.resolution) - 1)
        max_y_index = min(self.height - 1, math.ceil((max_y - self.origin_y) / self.resolution) + 1)
        abs_forward_x = abs(forward_x)
        abs_forward_y = abs(forward_y)
        world_half_x = half_forward * abs_forward_x + half_left * abs_forward_y
        world_half_y = half_forward * abs_forward_y + half_left * abs_forward_x
        cell_projection = cell_half * (abs_forward_x + abs_forward_y)
        for y_index in range(min_y_index, max_y_index + 1):
            row = self.height - 1 - y_index
            for column in range(min_column, max_column + 1):
                cell_x = self.origin_x + column * self.resolution
                cell_y = self.origin_y + y_index * self.resolution
                dx = cell_x - center_x
                dy = cell_y - center_y
                if abs(dx) > world_half_x + cell_half + 1e-9:
                    continue
                if abs(dy) > world_half_y + cell_half + 1e-9:
                    continue
                local_forward = dx * forward_x + dy * forward_y
                local_left = dx * left_x + dy * left_y
                intersects = (
                    abs(local_forward) <= half_forward + cell_projection + 1e-9
                    and abs(local_left) <= half_left + cell_projection + 1e-9
                )
                if intersects and not bool(self.free[row, column]):
                    return False
        return True

    def clearance(self, x: float, y: float, yaw: float, isotropic: bool) -> float:
        if not self.footprint_clear(x, y, yaw, 0.0, isotropic):
            return -0.0
        lower = 0.0
        upper = 1.0
        for _ in range(14):
            middle = 0.5 * (lower + upper)
            if self.footprint_clear(x, y, yaw, middle, isotropic):
                lower = middle
            else:
                upper = middle
        return lower


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def nearest_by_time(samples, timestamp: float):
    times = [sample.t for sample in samples]
    index = bisect.bisect_left(times, timestamp)
    candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(samples)]
    return min(candidates, key=lambda candidate: abs(times[candidate] - timestamp))


def project_to_trajectory(points, x: float, y: float):
    best = None
    for index in range(len(points) - 1):
        x0, y0 = points[index][:2]
        x1, y1 = points[index + 1][:2]
        dx = x1 - x0
        dy = y1 - y0
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            continue
        ratio = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / length_squared))
        projected_x = x0 + ratio * dx
        projected_y = y0 + ratio * dy
        error_x = x - projected_x
        error_y = y - projected_y
        distance_squared = error_x * error_x + error_y * error_y
        heading = math.atan2(dy, dx)
        cross_track = -math.sin(heading) * error_x + math.cos(heading) * error_y
        candidate = (distance_squared, index, ratio, cross_track, heading)
        if best is None or candidate < best:
            best = candidate
    if best is None:
        raise RuntimeError("runtime trajectory has no non-degenerate segment")
    return best


def find_abrupt_loss(samples, maximum_interval_sec: float = 0.12):
    best = None
    left = 0
    for right, sample in enumerate(samples):
        while left < right and sample.t - samples[left].t > maximum_interval_sec:
            left += 1
        for start in range(left, right):
            dt = sample.t - samples[start].t
            if dt <= 0.0:
                continue
            loss = float(samples[start].values["vx"]) - float(sample.values["vx"])
            candidate = (loss, dt, start, right)
            if best is None or candidate > best:
                best = candidate
    return best


def load(path: Path):
    run = read_bag(path, TOPICS)
    odometry = run.series.get("ekf_pose", [])
    controls = run.series.get("control", [])
    trajectories = run.series.get("runtime_trajectory", [])
    if not odometry or not controls or not trajectories:
        raise RuntimeError(
            f"missing required series in {path}: "
            f"odom={len(odometry)} control={len(controls)} trajectory={len(trajectories)}"
        )
    return run, odometry, controls, trajectories[0].values["points"]


def analyze(path: Path, radius_sec: float, summary_only: bool, wall: WallGrid):
    run, odometry, controls, points = load(path)
    abrupt = find_abrupt_loss(odometry)
    if abrupt is None:
        raise RuntimeError(f"no speed-loss interval found in {path}")
    loss, dt, start_index, end_index = abrupt
    event_time = odometry[end_index].t
    event_sample = odometry[end_index]
    print(f"RUN {path}")
    print(
        "EVENT "
        f"t={event_time:.6f}s loss={loss:.3f}mps dt={dt:.3f}s "
        f"from={odometry[start_index].values['vx']:.3f} "
        f"to={odometry[end_index].values['vx']:.3f}"
    )
    print(
        "LOCATION "
        f"x={event_sample.values['x']:.4f} y={event_sample.values['y']:.4f} "
        f"yaw={event_sample.values['yaw']:+.4f}"
    )
    lateral_clearance = wall.clearance(
        float(event_sample.values["x"]),
        float(event_sample.values["y"]),
        float(event_sample.values["yaw"]),
        False,
    )
    isotropic_clearance = wall.clearance(
        float(event_sample.values["x"]),
        float(event_sample.values["y"]),
        float(event_sample.values["yaw"]),
        True,
    )
    if summary_only:
        _, segment, ratio, cross_track, course_heading = project_to_trajectory(
            points, float(event_sample.values["x"]), float(event_sample.values["y"])
        )
        control = controls[nearest_by_time(controls, event_time)].values
        imu_samples = run.series.get("imu_raw", [])
        velocity_samples = run.series.get("vehicle_velocity", [])
        steering_samples = run.series.get("steering", [])
        condition_samples = run.series.get("pitstop_condition", [])
        imu = (
            imu_samples[nearest_by_time(imu_samples, event_time)].values
            if imu_samples else {}
        )
        vehicle_velocity = (
            velocity_samples[nearest_by_time(velocity_samples, event_time)].values
            if velocity_samples else {}
        )
        steering = (
            steering_samples[nearest_by_time(steering_samples, event_time)].values
            if steering_samples else {}
        )
        condition = (
            condition_samples[
                nearest_by_time(condition_samples, event_time)
            ].values["condition"]
            if condition_samples else None
        )
        print(
            "SUMMARY "
            f"segment={segment} ratio={ratio:.3f} e_y={cross_track:+.4f} "
            f"e_psi={wrap_angle(float(event_sample.values['yaw']) - course_heading):+.4f} "
            f"wall_lat={lateral_clearance:.4f} wall_iso={isotropic_clearance:.4f} "
            f"cmd_v={control['speed']:.3f} cmd_a={control['acceleration']:+.3f} "
            f"cmd_delta={control['steering']:+.4f} "
            f"vehicle_v={vehicle_velocity.get('vx', math.nan):.3f} "
            f"actual_delta={steering.get('steering', math.nan):+.4f} "
            f"imu_ax={imu.get('accel_x', math.nan):+.1f} "
            f"imu_ay={imu.get('accel_y', math.nan):+.1f} "
            f"pitstop_condition={condition if condition is not None else 'missing'}"
        )
        print()
        return run, odometry, controls, points, event_sample
    print("SAMPLES t vx segment ratio e_y e_psi x y cmd_v cmd_a cmd_delta")
    stride = max(1, int(round(len(odometry) / max(run.duration_sec, 1.0) / 20.0)))
    selected = [
        index
        for index, sample in enumerate(odometry)
        if event_time - radius_sec <= sample.t <= event_time + radius_sec
    ]
    for index in selected[::stride]:
        sample = odometry[index]
        x = float(sample.values["x"])
        y = float(sample.values["y"])
        yaw = float(sample.values["yaw"])
        _, segment, ratio, cross_track, course_heading = project_to_trajectory(points, x, y)
        control = controls[nearest_by_time(controls, sample.t)].values
        print(
            f"{sample.t:.6f} {sample.values['vx']:.3f} {segment:d} {ratio:.3f} "
            f"{cross_track:+.4f} {wrap_angle(yaw - course_heading):+.4f} "
            f"{x:.4f} {y:.4f} "
            f"{control['speed']:.3f} {control['acceleration']:+.3f} "
            f"{control['steering']:+.4f}"
        )
    print()
    return run, odometry, controls, points, event_sample


def compare_location(label: str, loaded, target_x: float, target_y: float, wall: WallGrid) -> None:
    _, odometry, controls, points, _ = loaded
    distances = [
        math.hypot(float(sample.values["x"]) - target_x, float(sample.values["y"]) - target_y)
        for sample in odometry
    ]
    minima = []
    for index in range(1, len(distances) - 1):
        if distances[index] <= distances[index - 1] and distances[index] < distances[index + 1]:
            if distances[index] > 2.0:
                continue
            if minima and odometry[index].t - odometry[minima[-1]].t < 10.0:
                if distances[index] < distances[minima[-1]]:
                    minima[-1] = index
                continue
            minima.append(index)
    print(f"CROSSINGS {label} target={target_x:.4f},{target_y:.4f}")
    print("t distance vx segment ratio e_y e_psi wall_lat wall_iso cmd_v cmd_a cmd_delta")
    for index in minima:
        sample = odometry[index]
        x = float(sample.values["x"])
        y = float(sample.values["y"])
        yaw = float(sample.values["yaw"])
        _, segment, ratio, cross_track, course_heading = project_to_trajectory(points, x, y)
        control = controls[nearest_by_time(controls, sample.t)].values
        lateral_clearance = wall.clearance(x, y, yaw, False)
        isotropic_clearance = wall.clearance(x, y, yaw, True)
        print(
            f"{sample.t:.6f} {distances[index]:.4f} {sample.values['vx']:.3f} "
            f"{segment:d} {ratio:.3f} {cross_track:+.4f} "
            f"{wrap_angle(yaw - course_heading):+.4f} "
            f"{lateral_clearance:.4f} {isotropic_clearance:.4f} "
            f"{control['speed']:.3f} "
            f"{control['acceleration']:+.3f} {control['steering']:+.4f}"
        )
    print()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--radius-sec", type=float, default=0.8)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--summary-only", action="store_true")
    args = parser.parse_args()
    wall = WallGrid(args.map)
    loaded = [(bag, analyze(bag, args.radius_sec, args.summary_only, wall)) for bag in args.bags]
    target = loaded[0][1][4]
    if args.summary_only:
        return 0
    for bag, run in loaded:
        compare_location(
            str(bag), run, float(target.values["x"]), float(target.values["y"]), wall
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
