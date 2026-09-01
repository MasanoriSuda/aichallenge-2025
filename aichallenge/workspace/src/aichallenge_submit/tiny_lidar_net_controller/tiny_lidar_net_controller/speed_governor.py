"""Deterministic positive-acceleration governor for E2E runtime authority."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class ForwardSpeedGovernorDecision:
    """Auditable result of applying the forward-speed authority boundary."""

    active: bool
    speed_mps: Optional[float]
    requested_acceleration_mps2: float
    acceleration_mps2: float
    reason: str


class ForwardSpeedGovernor:
    """Limit positive acceleration without weakening downstream braking.

    A fixed acceleration command otherwise also becomes an implicit terminal-speed
    choice through simulator drag.  This governor separates those responsibilities:
    acceleration retains launch/transient authority while ``maximum_speed_mps``
    bounds the operating-speed distribution seen by the frozen steering policy.
    """

    def __init__(self, maximum_speed_mps: float) -> None:
        maximum_speed_mps = float(maximum_speed_mps)
        if not np.isfinite(maximum_speed_mps) or maximum_speed_mps <= 0.0:
            raise ValueError("maximum_speed_mps must be finite and positive")
        self.maximum_speed_mps = maximum_speed_mps

    def decide(
        self,
        speed_mps: Optional[float],
        requested_acceleration_mps2: float,
    ) -> ForwardSpeedGovernorDecision:
        requested = float(requested_acceleration_mps2)
        if not np.isfinite(requested):
            raise ValueError("requested acceleration must be finite")

        if speed_mps is None:
            bounded = min(requested, 0.0)
            return ForwardSpeedGovernorDecision(
                active=bounded < requested,
                speed_mps=None,
                requested_acceleration_mps2=requested,
                acceleration_mps2=bounded,
                reason="missing-or-stale-speed",
            )

        speed = float(speed_mps)
        if not np.isfinite(speed) or speed < 0.0:
            raise ValueError("speed must be finite and non-negative")

        # Unit gain converts the speed error [m/s] into an acceleration ceiling
        # [m/s^2].  It is intentionally deterministic and has no retained state.
        positive_acceleration_ceiling = max(0.0, self.maximum_speed_mps - speed)
        bounded = min(requested, positive_acceleration_ceiling)
        return ForwardSpeedGovernorDecision(
            active=bounded < requested,
            speed_mps=speed,
            requested_acceleration_mps2=requested,
            acceleration_mps2=bounded,
            reason="speed-cap" if bounded < requested else "below-speed-cap",
        )
