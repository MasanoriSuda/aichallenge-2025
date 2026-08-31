"""Teacher-only LiDAR gap policy used to collect corrective imitation labels."""

from dataclasses import dataclass
import math
from typing import List, Tuple

import numpy as np


@dataclass(frozen=True)
class GapTeacherConfig:
    """Physical configuration for the bounded Follow-the-Gap residual."""

    trigger_distance_m: float = 7.0
    slow_distance_m: float = 3.0
    stop_distance_m: float = 1.5
    bubble_margin_m: float = 0.25
    vehicle_half_width_m: float = 0.725
    max_steering_angle_rad: float = 0.64
    steering_angle_gain: float = 0.75
    scan_half_fov_rad: float = math.pi / 2.0
    search_half_angle_rad: float = 1.20
    front_half_angle_rad: float = 0.18
    minimum_gap_angle_rad: float = 0.12
    side_start_angle_rad: float = 1.3
    side_trigger_distance_m: float = 1.8
    side_critical_distance_m: float = 0.9
    brake_acceleration_mps2: float = -1.0

    def __post_init__(self) -> None:
        positive = {
            "trigger_distance_m": self.trigger_distance_m,
            "slow_distance_m": self.slow_distance_m,
            "stop_distance_m": self.stop_distance_m,
            "bubble_margin_m": self.bubble_margin_m,
            "vehicle_half_width_m": self.vehicle_half_width_m,
            "max_steering_angle_rad": self.max_steering_angle_rad,
            "steering_angle_gain": self.steering_angle_gain,
            "scan_half_fov_rad": self.scan_half_fov_rad,
            "search_half_angle_rad": self.search_half_angle_rad,
            "front_half_angle_rad": self.front_half_angle_rad,
            "minimum_gap_angle_rad": self.minimum_gap_angle_rad,
            "side_start_angle_rad": self.side_start_angle_rad,
            "side_trigger_distance_m": self.side_trigger_distance_m,
            "side_critical_distance_m": self.side_critical_distance_m,
        }
        for name, value in positive.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not (
            self.stop_distance_m
            < self.slow_distance_m
            < self.trigger_distance_m
        ):
            raise ValueError(
                "gap distances must satisfy stop < slow < trigger"
            )
        if self.search_half_angle_rad > self.scan_half_fov_rad:
            raise ValueError("search angle must fit inside the LiDAR field of view")
        if self.front_half_angle_rad >= self.search_half_angle_rad:
            raise ValueError("front angle must be narrower than the search angle")
        if not (
            self.search_half_angle_rad
            < self.side_start_angle_rad
            < self.scan_half_fov_rad
        ):
            raise ValueError(
                "side start angle must be outside search and inside LiDAR field of view"
            )
        if self.side_critical_distance_m >= self.side_trigger_distance_m:
            raise ValueError("side distances must satisfy critical < trigger")
        if (
            not np.isfinite(self.brake_acceleration_mps2)
            or self.brake_acceleration_mps2 >= 0.0
        ):
            raise ValueError("brake_acceleration_mps2 must be finite and negative")


@dataclass(frozen=True)
class GapTeacherDecision:
    """One auditable teacher decision."""

    active: bool
    front_distance_m: float
    target_angle_rad: float
    base_steering_rad: float
    steering_rad: float
    acceleration_mps2: float
    left_side_distance_m: float
    right_side_distance_m: float
    reason: str


@dataclass(frozen=True)
class LongitudinalSafetyDecision:
    """Auditable acceleration authority derived from frontal LiDAR clearance."""

    active: bool
    front_distance_m: float
    requested_acceleration_mps2: float
    acceleration_mps2: float
    reason: str


class LidarLongitudinalSafety:
    """Limit longitudinal acceleration without taking lateral authority."""

    def __init__(self, config: GapTeacherConfig):
        self.config = config

    def front_distance(self, ranges_m: np.ndarray) -> float:
        ranges = np.asarray(ranges_m, dtype=np.float64)
        if ranges.ndim != 1 or ranges.size < 3:
            raise ValueError("LiDAR safety requires a one-dimensional scan")
        if not np.all(np.isfinite(ranges)) or np.any(ranges < 0.0):
            raise ValueError("LiDAR safety ranges must be finite and non-negative")
        angles = np.linspace(
            -self.config.scan_half_fov_rad,
            self.config.scan_half_fov_rad,
            ranges.size,
            dtype=np.float64,
        )
        front_values = ranges[
            np.abs(angles) <= self.config.front_half_angle_rad
        ]
        return float(np.percentile(front_values, 20.0))

    def decide_from_front_distance(
        self,
        front_distance_m: float,
        requested_acceleration_mps2: float,
    ) -> LongitudinalSafetyDecision:
        front_distance = float(front_distance_m)
        requested = float(requested_acceleration_mps2)
        if not np.isfinite(front_distance) or front_distance < 0.0:
            raise ValueError("front_distance_m must be finite and non-negative")
        if not np.isfinite(requested):
            raise ValueError("requested acceleration must be finite")

        if front_distance <= self.config.stop_distance_m:
            acceleration = self.config.brake_acceleration_mps2
            reason = "stop-clearance"
        elif front_distance <= self.config.slow_distance_m:
            acceleration = min(requested, 0.0)
            reason = "slow-clearance"
        else:
            acceleration = requested
            reason = "clear"
        return LongitudinalSafetyDecision(
            active=acceleration < requested,
            front_distance_m=front_distance,
            requested_acceleration_mps2=requested,
            acceleration_mps2=float(acceleration),
            reason=reason,
        )

    def decide(
        self,
        ranges_m: np.ndarray,
        requested_acceleration_mps2: float,
    ) -> LongitudinalSafetyDecision:
        return self.decide_from_front_distance(
            self.front_distance(ranges_m), requested_acceleration_mps2
        )


def _true_segments(mask: np.ndarray) -> List[Tuple[int, int]]:
    """Return inclusive-exclusive intervals for contiguous true values."""
    values = np.asarray(mask, dtype=bool)
    if values.ndim != 1:
        raise ValueError("mask must be one-dimensional")
    padded = np.concatenate(([False], values, [False])).astype(np.int8)
    transitions = np.diff(padded)
    starts = np.flatnonzero(transitions == 1)
    ends = np.flatnonzero(transitions == -1)
    return list(zip(starts.tolist(), ends.tolist()))


class LidarGapTeacher:
    """Blend a lane-following network toward the best current LiDAR opening."""

    def __init__(self, config: GapTeacherConfig):
        self.config = config
        self.longitudinal_safety = LidarLongitudinalSafety(config)

    def decide(
        self,
        ranges_m: np.ndarray,
        base_steering_rad: float,
        base_acceleration_mps2: float,
    ) -> GapTeacherDecision:
        ranges = np.asarray(ranges_m, dtype=np.float64)
        if ranges.ndim != 1 or ranges.size < 3:
            raise ValueError("gap teacher requires a one-dimensional LiDAR scan")
        if not np.all(np.isfinite(ranges)) or np.any(ranges < 0.0):
            raise ValueError("gap teacher ranges must be finite and non-negative")

        cfg = self.config
        angles = np.linspace(
            -cfg.scan_half_fov_rad,
            cfg.scan_half_fov_rad,
            ranges.size,
            dtype=np.float64,
        )
        base_steering = float(
            np.clip(
                base_steering_rad,
                -cfg.max_steering_angle_rad,
                cfg.max_steering_angle_rad,
            )
        )
        front_distance = self.longitudinal_safety.front_distance(ranges)
        longitudinal = self.longitudinal_safety.decide_from_front_distance(
            front_distance, base_acceleration_mps2
        )
        left_side_values = ranges[angles >= cfg.side_start_angle_rad]
        right_side_values = ranges[angles <= -cfg.side_start_angle_rad]
        left_side_distance = float(np.percentile(left_side_values, 10.0))
        right_side_distance = float(np.percentile(right_side_values, 10.0))
        left_side_risk = float(
            np.clip(
                (cfg.side_trigger_distance_m - left_side_distance)
                / (cfg.side_trigger_distance_m - cfg.side_critical_distance_m),
                0.0,
                1.0,
            )
        )
        right_side_risk = float(
            np.clip(
                (cfg.side_trigger_distance_m - right_side_distance)
                / (cfg.side_trigger_distance_m - cfg.side_critical_distance_m),
                0.0,
                1.0,
            )
        )
        side_risk = max(left_side_risk, right_side_risk)
        if front_distance >= cfg.trigger_distance_m and side_risk <= 0.0:
            return GapTeacherDecision(
                active=False,
                front_distance_m=front_distance,
                target_angle_rad=0.0,
                base_steering_rad=base_steering,
                steering_rad=base_steering,
                acceleration_mps2=longitudinal.acceleration_mps2,
                left_side_distance_m=left_side_distance,
                right_side_distance_m=right_side_distance,
                reason="front-clear",
            )

        side_escape_steering = float(
            cfg.max_steering_angle_rad * (right_side_risk - left_side_risk)
        )
        if front_distance >= cfg.trigger_distance_m:
            steering = float(
                np.clip(
                    (1.0 - side_risk) * base_steering
                    + side_risk * side_escape_steering,
                    -cfg.max_steering_angle_rad,
                    cfg.max_steering_angle_rad,
                )
            )
            return GapTeacherDecision(
                active=True,
                front_distance_m=front_distance,
                target_angle_rad=side_escape_steering,
                base_steering_rad=base_steering,
                steering_rad=steering,
                acceleration_mps2=longitudinal.acceleration_mps2,
                left_side_distance_m=left_side_distance,
                right_side_distance_m=right_side_distance,
                reason="side-clearance",
            )

        search = np.abs(angles) <= cfg.search_half_angle_rad
        search_indices = np.flatnonzero(search)
        closest_local = int(np.argmin(ranges[search]))
        closest_index = int(search_indices[closest_local])
        closest_distance = max(float(ranges[closest_index]), 0.10)
        bubble_half_angle = math.atan2(
            cfg.vehicle_half_width_m + cfg.bubble_margin_m,
            closest_distance,
        )
        clearance_threshold = min(
            cfg.trigger_distance_m,
            max(cfg.slow_distance_m, closest_distance + cfg.bubble_margin_m),
        )

        navigable = search & (ranges > clearance_threshold)
        navigable &= np.abs(angles - angles[closest_index]) > bubble_half_angle
        angle_step = float(abs(angles[1] - angles[0]))
        minimum_points = max(
            1, int(math.ceil(cfg.minimum_gap_angle_rad / angle_step))
        )
        segments = [
            segment
            for segment in _true_segments(navigable)
            if segment[1] - segment[0] >= minimum_points
        ]

        acceleration = longitudinal.acceleration_mps2

        if not segments:
            return GapTeacherDecision(
                active=True,
                front_distance_m=front_distance,
                target_angle_rad=0.0,
                base_steering_rad=base_steering,
                steering_rad=base_steering,
                acceleration_mps2=acceleration,
                left_side_distance_m=left_side_distance,
                right_side_distance_m=right_side_distance,
                reason="no-gap",
            )

        best_score = -float("inf")
        best_angle = 0.0
        for start, end in segments:
            segment_ranges = ranges[start:end]
            segment_angles = angles[start:end]
            weights = np.clip(segment_ranges, 0.0, cfg.trigger_distance_m)
            target_angle = float(np.average(segment_angles, weights=weights))
            width = float(segment_angles[-1] - segment_angles[0] + angle_step)
            reserve = float(np.percentile(segment_ranges, 25.0))
            forward_value = math.cos(target_angle)
            continuity_cost = abs(
                target_angle - np.clip(base_steering, -0.8, 0.8)
            )
            score = width * reserve + 0.75 * forward_value - 0.20 * continuity_cost
            if score > best_score:
                best_score = score
                best_angle = target_angle

        target_steering = float(
            np.clip(
                cfg.steering_angle_gain * best_angle,
                -cfg.max_steering_angle_rad,
                cfg.max_steering_angle_rad,
            )
        )
        if side_risk > 0.0:
            target_steering = float(
                np.clip(
                    (1.0 - side_risk) * target_steering
                    + side_risk * side_escape_steering,
                    -cfg.max_steering_angle_rad,
                    cfg.max_steering_angle_rad,
                )
            )
        severity = float(
            np.clip(
                (cfg.trigger_distance_m - front_distance)
                / (cfg.trigger_distance_m - cfg.stop_distance_m),
                0.0,
                1.0,
            )
        )
        steering = float(
            np.clip(
                (1.0 - severity) * base_steering + severity * target_steering,
                -cfg.max_steering_angle_rad,
                cfg.max_steering_angle_rad,
            )
        )
        return GapTeacherDecision(
            active=True,
            front_distance_m=front_distance,
            target_angle_rad=best_angle,
            base_steering_rad=base_steering,
            steering_rad=steering,
            acceleration_mps2=acceleration,
            left_side_distance_m=left_side_distance,
            right_side_distance_m=right_side_distance,
            reason="gap-selected",
        )
