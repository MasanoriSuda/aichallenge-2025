# Low-speed direct authority retirement

## Purpose

Retire the hand-written `LowSpeedDirect` controller from production stopped-vehicle avoidance so
that the existing receding MPCC problem remains the only normal execution path for that scenario.

## Repaired invariant

A normal Dynamic Escape command and the trajectory used by the final physical wall admission must
come from the same solver result and prediction. A controller which clears its prediction may not
claim Dynamic Escape execution authority.

## Failing evidence

- Replay: `output/20260823-214300-stop-authority-replay-v2/d1/autoware.log`
- First violation: decision `1944`
- `LowSpeedDirect` entered for stopped vehicle `d2`, cleared `current_prediction`, and was then
  classified by the final Dynamic Escape wall admission as `prediction-unavailable`.
- The resulting final decision simultaneously reported `intent=shiftout`,
  `formulation=low-speed-direct`, `authority=legacy-normal-bypass`, and wall-hold deceleration.

## Scope

- Remove production activation and publication of `LowSpeedDirect` for stopped-vehicle avoidance.
- Keep the stopped-vehicle local path/gap planner as a constraint/reference producer.
- Let the existing progress MPCC consume that path in the same control cycle.
- Add a structural regression test which forbids production assignment to direct authority.

## Non-scope

- Do not tune speed, wall clearance, weights, solver tolerances, timeouts or cooldowns.
- Do not change V2X target selection or gap geometry.
- Do not yet remove five-state to three-state/legacy fallback; that remains Slice 5 work.
- Do not change emergency Stop or Stuck/Reverse Recovery.

## Acceptance

- Static search finds no production assignment which activates `low_speed_shift_control_active_`.
- Package build and tests pass.
- Deterministic replay has no `formulation=low-speed-direct` or `low-speed-direct-control` final
  decision.
- Stopped-front Dynamic Escape either publishes a physically certified MPCC solution or an explicit
  safe failure; it must not produce `prediction-unavailable` by clearing its own prediction.
- Existing canonical Track/Cruise, Follow and Stop authority remains unchanged.

## Rollback

Rollback to `7d4c991` if stopped-front replay cannot obtain an MPCC solution or if the replacement
creates wall/contact regression. Do not restore direct authority behind a new flag.
