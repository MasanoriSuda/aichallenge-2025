# Stop emergency authority boundary requirements

## Purpose

Remove the legacy normal-solver fall-through from a `SafetyBrake` decision that has already resolved
to `ControlIntent::Stop`. Route that intent through the existing canonical emergency supervisor as
one explicit authority decision.

## Confirmed root cause

The authority orchestrator maps `Action::SafetyBrake` to `ControlIntent::Stop`, but `get_control()`
has no Stop boundary. It therefore solves a normal legacy/progress controller and only later applies
the zero-speed SafetyBrake limit and maximum deceleration. The final command combines emergency
longitudinal ownership with a normal solver selected under another execution path.

## Required invariants

- SafetyBrake/Stop is an emergency supervisor action, not a normal trajectory authority.
- A Stop cycle cannot reach legacy MPC, three-state MPCC, five-state normal solve or low-speed direct
  control.
- Stop preserves bounded current steering while requesting zero speed and configured maximum
  braking through the existing final emergency publication path.
- Recovery may still override Stop in the final arbitration layer.
- Track, Cruise, Follow and overtake intent behavior is unchanged.
- No parameter, timeout, lease, fallback or migration feature flag is added.
- Hold is not inferred from Stop or DynamicWait.
- The user-owned `aichallenge/result-summary.json` is not modified or committed.

## Exit gate

- Pure tests prove Stop is exclusively emergency-owned and non-Stop is not captured.
- Static control flow returns before every normal solver for Stop.
- Package tests and `make autoware-build` pass.
- Deterministic replay reports `intent=stop`, `authority=emergency-override`, canonical emergency
  output, and no `legacy-mpc-solved` Stop trace.

