"""Speed-aware, temporally committed diagnostic LiDAR teacher.

This module deliberately wraps the immutable historical pre-contact teacher.
It separates geometric proposal generation from physical recoverability and
encounter state so the old teacher and its recorded label provenance do not
change.
"""

from dataclasses import dataclass, replace
import math

import numpy as np

from tiny_lidar_net_controller.gap_teacher import (
    GapTeacherConfig,
    GapTeacherDecision,
    LidarPrecontactTeacher,
)


@dataclass(frozen=True)
class SpeedCommittedTeacherConfig:
    """Physical timing and release contract for the successor teacher."""

    reaction_time_sec: float = 0.25
    preview_time_sec: float = 0.50
    release_confirmation_samples: int = 5
    side_switch_confirmation_samples: int = 2
    minimum_commit_speed_mps: float = 0.50

    def __post_init__(self) -> None:
        finite_positive = {
            "reaction_time_sec": self.reaction_time_sec,
            "preview_time_sec": self.preview_time_sec,
            "minimum_commit_speed_mps": self.minimum_commit_speed_mps,
        }
        for name, value in finite_positive.items():
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        integer_parameters = {
            "release_confirmation_samples": self.release_confirmation_samples,
            "side_switch_confirmation_samples": (
                self.side_switch_confirmation_samples
            ),
        }
        for name, value in integer_parameters.items():
            if (
                not isinstance(value, int)
                or isinstance(value, bool)
                or value <= 0
            ):
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class SpeedCommittedTeacherDecision(GapTeacherDecision):
    """A teacher decision with the physical and temporal proof inputs."""

    speed_mps: float
    required_stop_distance_m: float
    dynamic_slow_distance_m: float
    dynamic_trigger_distance_m: float
    committed_side_sign: int
    proposed_side_sign: int
    clear_confirmation_samples: int
    pending_side_sign: int
    pending_side_samples: int
    supervisor_reason: str


class LidarSpeedCommittedTeacher:
    """Supervise a geometric LiDAR proposal with speed and side continuity.

    The class is stateful only for one obstacle encounter.  It does not retain a
    path or scan: every geometric proposal is rebuilt from the current scan.
    """

    def __init__(
        self,
        gap_config: GapTeacherConfig,
        config: SpeedCommittedTeacherConfig | None = None,
    ) -> None:
        self.gap_config = gap_config
        self.config = config or SpeedCommittedTeacherConfig()
        self._committed_side_sign = 0
        self._clear_confirmation_samples = 0
        self._pending_side_sign = 0
        self._pending_side_samples = 0

    @property
    def committed_side_sign(self) -> int:
        return self._committed_side_sign

    def reset(self) -> None:
        """Clear encounter state at an explicit runtime reset boundary."""
        self._committed_side_sign = 0
        self._clear_confirmation_samples = 0
        self._pending_side_sign = 0
        self._pending_side_samples = 0

    def dynamic_distances(self, speed_mps: float) -> tuple[float, float, float]:
        """Return stop, slow and trigger distances for current forward speed."""
        speed = self._validated_speed(speed_mps)
        deceleration = abs(self.gap_config.brake_acceleration_mps2)
        required_stop = (
            self.gap_config.stop_distance_m
            + speed * self.config.reaction_time_sec
            + speed * speed / (2.0 * deceleration)
        )
        dynamic_slow = max(
            self.gap_config.slow_distance_m,
            required_stop + 0.25 * speed,
        )
        dynamic_trigger = max(
            self.gap_config.trigger_distance_m,
            dynamic_slow + self.config.preview_time_sec * speed,
        )
        return float(required_stop), float(dynamic_slow), float(dynamic_trigger)

    @staticmethod
    def _validated_speed(speed_mps: float) -> float:
        if speed_mps is None:
            raise ValueError("speed-committed teacher requires fresh wheel speed")
        speed = float(speed_mps)
        if not np.isfinite(speed) or speed < 0.0:
            raise ValueError(
                "speed-committed teacher speed must be finite and non-negative"
            )
        return speed

    def _proposal_side_sign(self, decision: GapTeacherDecision) -> int:
        deadband = 0.5 * self.gap_config.minimum_gap_angle_rad
        value = float(decision.target_angle_rad)
        if abs(value) <= deadband:
            return 0
        return 1 if value > 0.0 else -1

    def _update_side_state(
        self,
        proposed_side_sign: int,
        speed_mps: float,
        clear_scene: bool,
        front_distance_m: float,
    ) -> tuple[str, bool]:
        if speed_mps < self.config.minimum_commit_speed_mps:
            self.reset()
            return "low-speed-uncommitted", False

        if clear_scene:
            self._pending_side_sign = 0
            self._pending_side_samples = 0
            if self._committed_side_sign == 0:
                self._clear_confirmation_samples = 0
                return "clear", False
            self._clear_confirmation_samples += 1
            if (
                self._clear_confirmation_samples
                >= self.config.release_confirmation_samples
            ):
                self.reset()
                return "side-released", False
            return "clear-release-pending", False

        self._clear_confirmation_samples = 0
        if proposed_side_sign == 0:
            self._pending_side_sign = 0
            self._pending_side_samples = 0
            return "no-side-proposal", False
        if self._committed_side_sign == 0:
            self._committed_side_sign = proposed_side_sign
            return "side-acquired", False
        if proposed_side_sign == self._committed_side_sign:
            self._pending_side_sign = 0
            self._pending_side_samples = 0
            return "side-maintained", False

        # A side change first seen inside the historical slow envelope is too
        # late to execute as a cross-corridor manoeuvre.  Brake and wait for a
        # new low-speed decision instead.
        if front_distance_m <= self.gap_config.slow_distance_m:
            self._pending_side_sign = proposed_side_sign
            self._pending_side_samples = 1
            return "late-side-switch-blocked", True

        if self._pending_side_sign != proposed_side_sign:
            self._pending_side_sign = proposed_side_sign
            self._pending_side_samples = 1
        else:
            self._pending_side_samples += 1
        if (
            self._pending_side_samples
            >= self.config.side_switch_confirmation_samples
        ):
            self._committed_side_sign = proposed_side_sign
            self._pending_side_sign = 0
            self._pending_side_samples = 0
            return "side-switch-confirmed", False
        return "side-switch-pending", False

    def decide(
        self,
        ranges_m: np.ndarray,
        base_steering_rad: float,
        base_acceleration_mps2: float,
        speed_mps: float,
    ) -> SpeedCommittedTeacherDecision:
        speed = self._validated_speed(speed_mps)
        required_stop, dynamic_slow, dynamic_trigger = self.dynamic_distances(speed)

        historical = LidarPrecontactTeacher(self.gap_config).decide(
            ranges_m,
            base_steering_rad,
            base_acceleration_mps2,
        )
        proposal = historical
        if historical.active and historical.front_distance_m < dynamic_trigger:
            # Extend geometric look-ahead only after the immutable teacher has
            # detected a real front/side hazard.  Applying the dynamic trigger
            # to every curve caused course walls to dominate normal driving.
            proposal_config = replace(
                self.gap_config,
                trigger_distance_m=dynamic_trigger,
            )
            proposal = LidarPrecontactTeacher(proposal_config).decide(
                ranges_m,
                base_steering_rad,
                base_acceleration_mps2,
            )
        proposed_side_sign = self._proposal_side_sign(proposal)
        bilateral_pinch = (
            speed >= self.config.minimum_commit_speed_mps
            and proposal.left_side_distance_m < self.gap_config.side_trigger_distance_m
            and proposal.right_side_distance_m < self.gap_config.side_trigger_distance_m
        )
        side_state_reason, late_switch_blocked = self._update_side_state(
            proposed_side_sign,
            speed,
            not historical.active,
            proposal.front_distance_m,
        )

        steering = proposal.steering_rad
        acceleration = proposal.acceleration_mps2
        supervisor_reason = side_state_reason

        if (
            proposed_side_sign != 0
            and proposed_side_sign == self._committed_side_sign
            and side_state_reason
            in {"side-acquired", "side-maintained", "side-switch-confirmed"}
        ):
            # The historical severity blend can leave the published steering on
            # the opposite side even after a gap candidate changes homotopy.
            # Once the new side is confirmed, the published command must agree
            # with the selected candidate rather than merely logging its sign.
            selected_steering = float(np.clip(
                self.gap_config.steering_angle_gain
                * proposal.target_angle_rad,
                -self.gap_config.max_steering_angle_rad,
                self.gap_config.max_steering_angle_rad,
            ))
            if proposed_side_sign > 0:
                steering = max(float(steering), selected_steering, 0.0)
            else:
                steering = min(float(steering), selected_steering, 0.0)

        if late_switch_blocked:
            steering = 0.0
            acceleration = self.gap_config.brake_acceleration_mps2
        elif side_state_reason == "side-switch-pending":
            steering = 0.0
            acceleration = min(float(acceleration), 0.0)
        elif bilateral_pinch:
            acceleration = self.gap_config.brake_acceleration_mps2
            supervisor_reason = "bilateral-side-pinch"
        elif proposal.reason == "no-gap" and proposal.front_distance_m <= dynamic_trigger:
            acceleration = self.gap_config.brake_acceleration_mps2
            supervisor_reason = "no-recoverable-gap"
        elif proposal.front_distance_m <= self.gap_config.stop_distance_m:
            acceleration = self.gap_config.brake_acceleration_mps2
            supervisor_reason = "static-stop-envelope"

        if not all(
            math.isfinite(value)
            for value in (
                steering,
                acceleration,
                required_stop,
                dynamic_slow,
                dynamic_trigger,
            )
        ):
            raise ValueError("speed-committed teacher produced a non-finite decision")

        return SpeedCommittedTeacherDecision(
            active=bool(
                proposal.active
                or acceleration < float(base_acceleration_mps2)
                or late_switch_blocked
                or bilateral_pinch
            ),
            front_distance_m=proposal.front_distance_m,
            target_angle_rad=proposal.target_angle_rad,
            base_steering_rad=proposal.base_steering_rad,
            steering_rad=float(np.clip(
                steering,
                -self.gap_config.max_steering_angle_rad,
                self.gap_config.max_steering_angle_rad,
            )),
            acceleration_mps2=float(acceleration),
            left_side_distance_m=proposal.left_side_distance_m,
            right_side_distance_m=proposal.right_side_distance_m,
            reason=proposal.reason,
            speed_mps=speed,
            required_stop_distance_m=required_stop,
            dynamic_slow_distance_m=dynamic_slow,
            dynamic_trigger_distance_m=dynamic_trigger,
            committed_side_sign=self._committed_side_sign,
            proposed_side_sign=proposed_side_sign,
            clear_confirmation_samples=self._clear_confirmation_samples,
            pending_side_sign=self._pending_side_sign,
            pending_side_samples=self._pending_side_samples,
            supervisor_reason=supervisor_reason,
        )
