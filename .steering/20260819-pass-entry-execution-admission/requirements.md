# Requirements

## Goal

Prevent `ShiftOut -> Pass` from being admitted only because the V2X target
prediction is fresh when the path that will actually own execution is not
wall- and lateral-acceleration-feasible from the measured vehicle state.

## Observed failure

In `output/20260819-104749`, domain 1 entered Pass with `ey=2.47 m`.  The
committed Pass continuation failed its physical preflight approximately 2.5 ms
later, the Mission was invalidated at `ey=2.90 m`, and the physical wall guard
reported intersection at `ey=3.61 m`.  The subsequent waypoint association
loss, solver failure, and Recovery were downstream effects.

## Constraints

- Fresh V2X data is necessary but is not proof of an executable Pass path.
- Use the measured lateral state and the execution source that will own the
  next control horizon.
- Physical wall contact and unavailable wall samples retain higher priority.
- Keep the existing bounded hold/reselection lifecycle; do not create an
  unbounded ShiftOut hold.
- Do not change ROS topics, messages, services, launch entry points, result
  schemas, wall clearance parameters, or acceleration limits.
- Preserve unrelated user changes, including `aichallenge/result-summary.json`.

## Definition of Done

- `ShiftOut -> Pass` requires both a fresh dynamic target horizon and a
  physically executable Pass-entry horizon.
- An unavailable execution horizon keeps ShiftOut ownership and requests the
  existing bounded replanning hold.
- The actual active DP/solved execution prefix is validated when available.
- Unit tests and the package build pass.
