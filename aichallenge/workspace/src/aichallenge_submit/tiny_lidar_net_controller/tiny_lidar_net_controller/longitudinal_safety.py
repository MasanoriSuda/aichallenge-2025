"""Stateless, speed-aware longitudinal safety for the E2E runtime."""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np

from tiny_lidar_net_controller.gap_teacher import (
    GapTeacherConfig,
    LidarLongitudinalSafety,
    LongitudinalSafetyDecision,
)


@dataclass(frozen=True)
class SpeedAwareLongitudinalSafetyConfig:
    """Physical assumptions for the bounded slow-zone speed envelope."""

    reaction_time_sec: float = 0.25
    effective_deceleration_mps2: float = 1.0
    speed_error_gain: float = 1.0

    def __post_init__(self) -> None:
        values = {
            "reaction_time_sec": self.reaction_time_sec,
            "effective_deceleration_mps2": self.effective_deceleration_mps2,
            "speed_error_gain": self.speed_error_gain,
        }
        for name, value in values.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")


@dataclass(frozen=True)
class SpeedAwareLongitudinalSafetyDecision(LongitudinalSafetyDecision):
    """One traceable speed-envelope decision."""

    speed_mps: Optional[float]
    safe_speed_mps: Optional[float]
    activation_distance_m: float


def safe_speed_from_clearance(
    front_distance_m: float,
    minimum_clearance_m: float,
    effective_deceleration_mps2: float,
    reaction_time_sec: float,
) -> float:
    """Return the maximum speed supported by reaction and braking distance."""
    values = (
        front_distance_m,
        minimum_clearance_m,
        effective_deceleration_mps2,
        reaction_time_sec,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("safe-speed inputs must be finite")
    if front_distance_m < 0.0:
        raise ValueError("front distance must be non-negative")
    if minimum_clearance_m <= 0.0 or effective_deceleration_mps2 <= 0.0:
        raise ValueError("clearance and deceleration must be positive")
    if reaction_time_sec <= 0.0:
        raise ValueError("reaction time must be positive")
    room = max(front_distance_m - minimum_clearance_m, 0.0)
    deceleration = effective_deceleration_mps2
    return float(
        math.sqrt(
            (deceleration * reaction_time_sec) ** 2
            + 2.0 * deceleration * room
        )
        - deceleration * reaction_time_sec
    )


class LidarSpeedAwareLongitudinalSafety:
    """Continuously limit acceleration inside the existing LiDAR slow zone."""

    def __init__(
        self,
        gap_config: GapTeacherConfig,
        config: SpeedAwareLongitudinalSafetyConfig | None = None,
    ) -> None:
        self.gap_config = gap_config
        self.config = config or SpeedAwareLongitudinalSafetyConfig()
        if self.config.effective_deceleration_mps2 > abs(
            self.gap_config.brake_acceleration_mps2
        ):
            raise ValueError(
                "effective deceleration may not exceed the published brake command"
            )
        self._front_observer = LidarLongitudinalSafety(gap_config)

    def front_distance(self, ranges_m: np.ndarray) -> float:
        return self._front_observer.front_distance(ranges_m)

    def decide_from_front_distance(
        self,
        front_distance_m: float,
        requested_acceleration_mps2: float,
        speed_mps: Optional[float],
    ) -> SpeedAwareLongitudinalSafetyDecision:
        front = float(front_distance_m)
        requested = float(requested_acceleration_mps2)
        if not np.isfinite(front) or front < 0.0:
            raise ValueError("front distance must be finite and non-negative")
        if not np.isfinite(requested):
            raise ValueError("requested acceleration must be finite")

        speed = None if speed_mps is None else float(speed_mps)
        speed_valid = speed is not None and np.isfinite(speed) and speed >= 0.0
        safe_speed = None
        if front <= self.gap_config.stop_distance_m:
            acceleration = min(
                requested, self.gap_config.brake_acceleration_mps2
            )
            reason = "stop-clearance"
        elif front > self.gap_config.slow_distance_m:
            acceleration = requested
            reason = "clear"
        elif not speed_valid:
            acceleration = min(requested, 0.0)
            reason = "missing-or-stale-speed"
        else:
            safe_speed = safe_speed_from_clearance(
                front,
                self.gap_config.stop_distance_m,
                self.config.effective_deceleration_mps2,
                self.config.reaction_time_sec,
            )
            acceleration_limit = max(
                self.gap_config.brake_acceleration_mps2,
                self.config.speed_error_gain * (safe_speed - speed),
            )
            acceleration = min(requested, acceleration_limit)
            reason = (
                "safe-speed-limit"
                if acceleration < requested
                else "safe-speed-clear"
            )

        return SpeedAwareLongitudinalSafetyDecision(
            active=acceleration < requested,
            front_distance_m=front,
            requested_acceleration_mps2=requested,
            acceleration_mps2=float(acceleration),
            reason=reason,
            speed_mps=speed if speed_valid else None,
            safe_speed_mps=safe_speed,
            activation_distance_m=self.gap_config.slow_distance_m,
        )

    def decide(
        self,
        ranges_m: np.ndarray,
        requested_acceleration_mps2: float,
        speed_mps: Optional[float],
    ) -> SpeedAwareLongitudinalSafetyDecision:
        return self.decide_from_front_distance(
            self.front_distance(ranges_m),
            requested_acceleration_mps2,
            speed_mps,
        )
