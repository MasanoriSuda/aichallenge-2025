from joycon_contract_guard.core import BoostGuard


def status(remaining: float, active: float) -> list[float]:
    return [0.0, 0.0, 0.0, 0.0, 0.0, remaining, active]


def test_requires_fresh_valid_status() -> None:
    guard = BoostGuard(0.5)
    assert not guard.try_trigger(1.0).allowed
    assert guard.on_status(status(1.0, 0.0), 1.0).allowed
    assert guard.try_trigger(1.5).allowed


def test_rejects_depleted_active_and_stale_status() -> None:
    guard = BoostGuard(0.5)
    guard.on_status(status(0.0, 0.0), 1.0)
    assert not guard.try_trigger(1.1).allowed
    guard.on_status(status(2.0, 1.0), 2.0)
    assert not guard.try_trigger(2.1).allowed
    guard.on_status(status(2.0, 0.0), 3.0)
    assert not guard.try_trigger(3.6).allowed


def test_requires_confirmation_before_second_pulse() -> None:
    guard = BoostGuard(0.5)
    guard.on_status(status(2.0, 0.0), 1.0)
    assert guard.try_trigger(1.1).allowed
    assert not guard.on_status(status(2.0, 0.0), 1.2).allowed
    assert not guard.try_trigger(1.3).allowed
    assert guard.on_status(status(1.0, 0.0), 1.4).allowed
    assert guard.try_trigger(1.5).allowed


def test_finish_latch_rearms_only_on_spawned() -> None:
    guard = BoostGuard(0.5)
    guard.on_status(status(2.0, 0.0), 1.0)
    guard.on_state("Finish")
    assert not guard.on_status(status(2.0, 0.0), 1.1).allowed
    assert not guard.try_trigger(1.2).allowed
    guard.on_state("Spawned")
    assert guard.on_status(status(2.0, 0.0), 1.3).allowed
    assert guard.try_trigger(1.4).allowed


def test_spawned_invalidates_cached_status() -> None:
    guard = BoostGuard(0.5)
    assert guard.on_status(status(2.0, 0.0), 1.0).allowed
    guard.on_state("Spawned")
    assert not guard.try_trigger(1.1).allowed
    assert guard.on_status(status(2.0, 0.0), 1.2).allowed
    assert guard.try_trigger(1.3).allowed


def test_rejects_short_and_nonfinite_status() -> None:
    guard = BoostGuard(0.5)
    assert not guard.on_status([0.0], 1.0).allowed
    assert not guard.on_status(status(float("nan"), 0.0), 1.0).allowed
    assert not guard.on_status([float("nan")] + status(2.0, 0.0)[1:], 1.0).allowed


def test_rejects_status_with_extra_fields() -> None:
    guard = BoostGuard(0.5)
    assert not guard.on_status(status(2.0, 0.0) + [0.0], 1.0).allowed
    assert not guard.try_trigger(1.1).allowed
