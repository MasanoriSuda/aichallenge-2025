"""Pure short-horizon manoeuvre rollout and LiDAR footprint certificate.

This module is intentionally independent from ROS and from runtime authority.
It is an offline diagnostic primitive: a certificate says that one current
LiDAR point cloud does not intersect the complete sampled kart footprint.  It
does not predict moving obstacles or unseen geometry.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class ManeuverRolloutConfig:
    """Physical and numerical contract for one candidate rollout."""

    wheelbase_m: float = 1.087
    front_extent_m: float = 1.554
    rear_extent_m: float = 0.510
    vehicle_half_width_m: float = 0.725
    lidar_offset_x_m: float = 1.65
    clearance_margin_m: float = 0.15
    maximum_steering_rad: float = 0.64
    maneuver_segment_sec: float = 0.45
    integration_step_sec: float = 0.10
    stop_deceleration_mps2: float = 1.0
    scan_half_fov_rad: float = math.pi / 2.0
    minimum_scan_range_m: float = 0.05
    maximum_scan_range_m: float = 30.0

    def __post_init__(self) -> None:
        positive = {
            "wheelbase_m": self.wheelbase_m,
            "front_extent_m": self.front_extent_m,
            "rear_extent_m": self.rear_extent_m,
            "vehicle_half_width_m": self.vehicle_half_width_m,
            "lidar_offset_x_m": self.lidar_offset_x_m,
            "clearance_margin_m": self.clearance_margin_m,
            "maximum_steering_rad": self.maximum_steering_rad,
            "maneuver_segment_sec": self.maneuver_segment_sec,
            "integration_step_sec": self.integration_step_sec,
            "stop_deceleration_mps2": self.stop_deceleration_mps2,
            "scan_half_fov_rad": self.scan_half_fov_rad,
            "minimum_scan_range_m": self.minimum_scan_range_m,
            "maximum_scan_range_m": self.maximum_scan_range_m,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if self.minimum_scan_range_m >= self.maximum_scan_range_m:
            raise ValueError("scan range bounds must be ordered")
        if self.integration_step_sec > self.maneuver_segment_sec:
            raise ValueError("integration step may not exceed a manoeuvre segment")


@dataclass(frozen=True)
class ManeuverCandidate:
    """One immutable rollout result and its physical certificate."""

    steering_offset_rad: float
    side_sign: int
    feasible: bool
    minimum_clearance_m: float
    required_clearance_m: float
    terminal_x_m: float
    terminal_y_m: float
    terminal_yaw_rad: float
    terminal_speed_mps: float
    sample_count: int
    states: np.ndarray


def scan_points_in_base(
    ranges_m: np.ndarray,
    config: ManeuverRolloutConfig,
) -> np.ndarray:
    """Convert one 180-degree scan into finite base-link XY obstacle points."""
    ranges = np.asarray(ranges_m, dtype=np.float64)
    if ranges.ndim != 1 or ranges.size < 3:
        raise ValueError("ranges must be a one-dimensional scan")
    if not np.all(np.isfinite(ranges)) or np.any(ranges < 0.0):
        raise ValueError("ranges must be finite and non-negative")
    angles = np.linspace(
        -config.scan_half_fov_rad,
        config.scan_half_fov_rad,
        ranges.size,
        dtype=np.float64,
    )
    # Max-range returns express absence of an obstacle, not a wall at exactly
    # the sensor range.  Exclude them from the footprint point set.
    observed = (
        (ranges >= config.minimum_scan_range_m)
        & (ranges < config.maximum_scan_range_m - 1e-6)
    )
    selected_ranges = ranges[observed]
    selected_angles = angles[observed]
    return np.column_stack(
        (
            config.lidar_offset_x_m
            + selected_ranges * np.cos(selected_angles),
            selected_ranges * np.sin(selected_angles),
        )
    )


def _validated_scalar(value: float, name: str, *, non_negative: bool = False) -> float:
    result = float(value)
    if not math.isfinite(result) or (non_negative and result < 0.0):
        suffix = " and non-negative" if non_negative else ""
        raise ValueError(f"{name} must be finite{suffix}")
    return result


def rollout_lane_change_stop(
    speed_mps: float,
    base_steering_rad: float,
    steering_offset_rad: float,
    config: ManeuverRolloutConfig,
) -> np.ndarray:
    """Roll a shift/counter-shift followed by a straight full-stop suffix.

    State is rear-axle ``[x, y, yaw, speed]`` in the current base-link frame.
    The two equal steering-offset segments provide a bounded lateral motion;
    the zero-steer suffix is an explicit contingency successor rather than an
    implicit promise that another command will arrive.
    """
    speed = _validated_scalar(speed_mps, "speed_mps", non_negative=True)
    base = _validated_scalar(base_steering_rad, "base_steering_rad")
    offset = _validated_scalar(steering_offset_rad, "steering_offset_rad")
    if abs(base) > config.maximum_steering_rad + 1e-9:
        raise ValueError("base steering exceeds the physical steering bound")

    manoeuvre_time = 2.0 * config.maneuver_segment_sec
    stop_time = speed / config.stop_deceleration_mps2
    total_time = manoeuvre_time + stop_time
    step_count = max(1, int(math.ceil(total_time / config.integration_step_sec)))
    states = np.zeros((step_count + 1, 4), dtype=np.float64)
    states[0, 3] = speed

    for index in range(step_count):
        elapsed = index * config.integration_step_sec
        remaining = max(total_time - elapsed, 0.0)
        dt = min(config.integration_step_sec, remaining)
        if dt <= 0.0:
            states[index + 1] = states[index]
            continue
        if elapsed < config.maneuver_segment_sec:
            steering = np.clip(
                base + offset,
                -config.maximum_steering_rad,
                config.maximum_steering_rad,
            )
            acceleration = 0.0
        elif elapsed < manoeuvre_time:
            steering = np.clip(
                base - offset,
                -config.maximum_steering_rad,
                config.maximum_steering_rad,
            )
            acceleration = 0.0
        else:
            steering = 0.0
            acceleration = -config.stop_deceleration_mps2

        x, y, yaw, current_speed = states[index]
        next_speed = max(0.0, current_speed + acceleration * dt)
        distance = 0.5 * (current_speed + next_speed) * dt
        curvature = math.tan(float(steering)) / config.wheelbase_m
        yaw_mid = yaw + 0.5 * distance * curvature
        next_yaw = yaw + distance * curvature
        states[index + 1] = (
            x + distance * math.cos(yaw_mid),
            y + distance * math.sin(yaw_mid),
            next_yaw,
            next_speed,
        )

    # Floating step boundaries can leave a negligible residual speed.
    states[-1, 3] = 0.0
    return states


def point_clearance_to_footprint(
    points_xy_m: np.ndarray,
    state: np.ndarray,
    config: ManeuverRolloutConfig,
) -> np.ndarray:
    """Return signed point distance to the oriented kart rectangle.

    Positive values are outside the physical rectangle, zero is its boundary
    and negative values are inside.  The configured safety margin is applied
    separately by the candidate certificate.
    """
    points = np.asarray(points_xy_m, dtype=np.float64)
    vehicle_state = np.asarray(state, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or not np.all(np.isfinite(points)):
        raise ValueError("points must be a finite Nx2 array")
    if vehicle_state.shape != (4,) or not np.all(np.isfinite(vehicle_state)):
        raise ValueError("state must be a finite [x, y, yaw, speed] vector")
    if len(points) == 0:
        return np.empty(0, dtype=np.float64)

    dx = points[:, 0] - vehicle_state[0]
    dy = points[:, 1] - vehicle_state[1]
    cosine = math.cos(float(vehicle_state[2]))
    sine = math.sin(float(vehicle_state[2]))
    longitudinal = cosine * dx + sine * dy
    lateral = -sine * dx + cosine * dy

    outside_longitudinal = np.maximum.reduce(
        (
            -config.rear_extent_m - longitudinal,
            longitudinal - config.front_extent_m,
            np.zeros_like(longitudinal),
        )
    )
    outside_lateral = np.maximum(np.abs(lateral) - config.vehicle_half_width_m, 0.0)
    distance = np.hypot(outside_longitudinal, outside_lateral)
    inside = (
        (longitudinal >= -config.rear_extent_m)
        & (longitudinal <= config.front_extent_m)
        & (np.abs(lateral) <= config.vehicle_half_width_m)
    )
    if np.any(inside):
        penetration = np.minimum.reduce(
            (
                longitudinal[inside] + config.rear_extent_m,
                config.front_extent_m - longitudinal[inside],
                config.vehicle_half_width_m - np.abs(lateral[inside]),
            )
        )
        distance[inside] = -penetration
    return distance


def evaluate_candidate(
    points_xy_m: np.ndarray,
    speed_mps: float,
    base_steering_rad: float,
    steering_offset_rad: float,
    config: ManeuverRolloutConfig,
) -> ManeuverCandidate:
    """Evaluate one complete candidate against the current obstacle points."""
    states = rollout_lane_change_stop(
        speed_mps,
        base_steering_rad,
        steering_offset_rad,
        config,
    )
    points = np.asarray(points_xy_m, dtype=np.float64)
    if len(points) == 0:
        minimum_clearance = math.inf
    else:
        minimum_clearance = min(
            float(np.min(point_clearance_to_footprint(points, state, config)))
            for state in states
        )
    return _candidate_from_states(
        states,
        steering_offset_rad,
        minimum_clearance,
        config,
    )


def _candidate_from_states(
    states: np.ndarray,
    steering_offset_rad: float,
    minimum_clearance_m: float,
    config: ManeuverRolloutConfig,
) -> ManeuverCandidate:
    """Build one immutable result from a rollout and clearance proof."""
    offset = float(steering_offset_rad)
    side_sign = 0 if abs(offset) <= 1e-9 else (1 if offset > 0.0 else -1)
    terminal = states[-1]
    feasible = bool(
        terminal[3] <= 1e-6
        and minimum_clearance_m >= config.clearance_margin_m
        and np.all(np.isfinite(terminal))
    )
    return ManeuverCandidate(
        steering_offset_rad=offset,
        side_sign=side_sign,
        feasible=feasible,
        minimum_clearance_m=minimum_clearance_m,
        required_clearance_m=config.clearance_margin_m,
        terminal_x_m=float(terminal[0]),
        terminal_y_m=float(terminal[1]),
        terminal_yaw_rad=float(terminal[2]),
        terminal_speed_mps=float(terminal[3]),
        sample_count=int(len(states)),
        states=states,
    )


def evaluate_candidate_against_time_indexed_points(
    point_clouds_xy_m: Iterable[np.ndarray],
    speed_mps: float,
    base_steering_rad: float,
    steering_offset_rad: float,
    config: ManeuverRolloutConfig,
) -> ManeuverCandidate:
    """Evaluate one rollout against one obstacle point cloud per state.

    Point cloud `k` represents occupancy at the same time as rollout state
    `k`.  The caller owns synchronization and coordinate transforms.
    """
    states = rollout_lane_change_stop(
        speed_mps,
        base_steering_rad,
        steering_offset_rad,
        config,
    )
    point_clouds = [np.asarray(points, dtype=np.float64) for points in point_clouds_xy_m]
    if len(point_clouds) != len(states):
        raise ValueError("time-indexed point clouds must align with rollout states")
    minima = []
    for points, state in zip(point_clouds, states):
        if points.ndim != 2 or points.shape[1:] != (2,) or not np.all(np.isfinite(points)):
            raise ValueError("each time-indexed point cloud must be a finite Nx2 array")
        if len(points) == 0:
            continue
        minima.append(float(np.min(point_clearance_to_footprint(points, state, config))))
    minimum_clearance = math.inf if not minima else min(minima)
    return _candidate_from_states(
        states,
        steering_offset_rad,
        minimum_clearance,
        config,
    )


def evaluate_candidates(
    ranges_m: np.ndarray,
    speed_mps: float,
    base_steering_rad: float,
    steering_offsets_rad: Iterable[float],
    config: ManeuverRolloutConfig | None = None,
) -> list[ManeuverCandidate]:
    """Evaluate an ordered candidate bank from one immutable scan."""
    cfg = config or ManeuverRolloutConfig()
    offsets = [float(value) for value in steering_offsets_rad]
    if not offsets or not np.all(np.isfinite(offsets)):
        raise ValueError("candidate offsets must be a non-empty finite sequence")
    points = scan_points_in_base(ranges_m, cfg)
    return [
        evaluate_candidate(points, speed_mps, base_steering_rad, offset, cfg)
        for offset in offsets
    ]


def select_best_candidate(
    candidates: Iterable[ManeuverCandidate],
) -> ManeuverCandidate | None:
    """Select the most robust feasible candidate with bounded intervention."""
    feasible = [candidate for candidate in candidates if candidate.feasible]
    if not feasible:
        return None

    def score(candidate: ManeuverCandidate) -> tuple[float, float, float]:
        clearance = min(candidate.minimum_clearance_m, 1000.0)
        return (
            clearance,
            -abs(candidate.steering_offset_rad),
            candidate.terminal_x_m,
        )

    return max(feasible, key=score)
