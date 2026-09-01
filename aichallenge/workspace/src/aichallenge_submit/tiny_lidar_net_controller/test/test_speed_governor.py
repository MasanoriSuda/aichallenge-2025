#!/usr/bin/env python3
"""Unit contract for the isolated E2E forward-speed authority."""

import pytest

from tiny_lidar_net_controller.speed_governor import ForwardSpeedGovernor


def test_positive_acceleration_is_unchanged_below_speed_band() -> None:
    decision = ForwardSpeedGovernor(4.6).decide(3.0, 0.8)
    assert not decision.active
    assert decision.acceleration_mps2 == pytest.approx(0.8)
    assert decision.reason == "below-speed-cap"


def test_positive_acceleration_tapers_and_stops_at_limit() -> None:
    governor = ForwardSpeedGovernor(4.6)
    tapered = governor.decide(4.4, 0.8)
    stopped = governor.decide(4.7, 0.8)

    assert tapered.active
    assert tapered.acceleration_mps2 == pytest.approx(0.2)
    assert stopped.active
    assert stopped.acceleration_mps2 == pytest.approx(0.0)


def test_negative_brake_is_never_weakened() -> None:
    decision = ForwardSpeedGovernor(4.6).decide(5.0, -1.0)
    assert not decision.active
    assert decision.acceleration_mps2 == pytest.approx(-1.0)


def test_missing_speed_prohibits_positive_acceleration() -> None:
    decision = ForwardSpeedGovernor(4.6).decide(None, 0.8)
    assert decision.active
    assert decision.acceleration_mps2 == pytest.approx(0.0)
    assert decision.reason == "missing-or-stale-speed"


@pytest.mark.parametrize("maximum", [0.0, -1.0, float("nan"), float("inf")])
def test_invalid_limit_is_rejected(maximum: float) -> None:
    with pytest.raises(ValueError):
        ForwardSpeedGovernor(maximum)
