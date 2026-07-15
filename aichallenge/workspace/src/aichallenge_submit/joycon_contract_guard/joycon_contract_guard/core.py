"""ROS-independent Boost edge guard for deterministic tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class GuardDecision:
    allowed: bool
    reason: str


class BoostGuard:
    """Fail closed unless a fresh AWSIM status proves an edge is allowed."""

    def __init__(self, status_timeout_sec: float) -> None:
        if not math.isfinite(status_timeout_sec) or status_timeout_sec <= 0.0:
            raise ValueError("status_timeout_sec must be finite and positive")
        self._status_timeout_sec = status_timeout_sec
        self._status_valid = False
        self._remaining = 0.0
        self._active = False
        self._last_status_time = 0.0
        self._pulse_pending = False
        self._remaining_before_pulse = 0.0
        self._finish_seen = False

    @staticmethod
    def normalize_state(raw: str) -> str:
        return "".join(character for character in (raw or "").lower() if character.isalnum())

    def on_state(self, raw: str) -> None:
        state = self.normalize_state(raw)
        if state == "finish":
            self._finish_seen = True
            self._status_valid = False
            return
        if state == "spawned":
            self._status_valid = False
            if self._finish_seen:
                self._finish_seen = False
                self._remaining = 0.0
                self._active = False
                self._pulse_pending = False
                self._remaining_before_pulse = 0.0

    def on_status(self, data: Sequence[float], receipt_time: float) -> GuardDecision:
        if self._finish_seen:
            self._status_valid = False
            return GuardDecision(False, "waiting for Finish -> Spawned")
        if len(data) != 7 or not math.isfinite(receipt_time):
            self._status_valid = False
            return GuardDecision(False, "invalid /awsim/status")
        try:
            values = tuple(float(value) for value in data)
        except (TypeError, ValueError):
            self._status_valid = False
            return GuardDecision(False, "invalid /awsim/status")
        if not all(math.isfinite(value) for value in values):
            self._status_valid = False
            return GuardDecision(False, "invalid /awsim/status")

        remaining = values[5]
        active = values[6] >= 0.5
        if (
            self._pulse_pending
            and not active
            and remaining > self._remaining_before_pulse - 0.5
        ):
            self._status_valid = False
            return GuardDecision(False, "previous Boost pulse is not confirmed")

        self._pulse_pending = False
        self._remaining = remaining
        self._active = active
        self._last_status_time = receipt_time
        self._status_valid = True
        return GuardDecision(True, "status accepted")

    def try_trigger(self, now: float) -> GuardDecision:
        if self._finish_seen:
            return GuardDecision(False, "AWSIM session has finished")
        if not self._status_valid:
            return GuardDecision(False, "no valid /awsim/status")
        age = now - self._last_status_time
        if not math.isfinite(age) or age < 0.0 or age > self._status_timeout_sec:
            self._status_valid = False
            return GuardDecision(False, "stale /awsim/status")
        if self._remaining < 1.0:
            return GuardDecision(False, "no remaining Boost")
        if self._active:
            return GuardDecision(False, "Boost is already active")

        self._remaining_before_pulse = self._remaining
        self._pulse_pending = True
        self._status_valid = False
        return GuardDecision(True, "Boost pulse allowed")
