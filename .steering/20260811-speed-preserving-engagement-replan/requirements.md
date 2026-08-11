# Requirements

## Purpose

Prevent a detected overtake target from momentarily falling back to Cruise, and
avoid discarding speed immediately when a committed Pass enters a recoverable
SafeSeparation revalidation window.

## Scope

- Preserve target-scoped pre-arm evidence across a short front/side
  classification dropout.
- Keep Behavior in Follow, not Cruise, during that bounded engagement lease.
- Preserve the committed Pass line and forward speed for a bounded tactical
  revalidation window when current bodies and the wall corridor are clear.
- Re-evaluate the existing same-side and alternate-side Mission candidates
  during the lease.
- Return smoothly to the base line, without FollowPrepare, when the target is
  confirmed clear ahead and the Return corridor is physically clear.

## Constraints

- A stale target must never authorize a lateral ShiftOut or Pass.
- V2X invalidity, position jump, emergency braking, wall faults, current body
  overlap, corridor loss, and solver recovery remain hard guards.
- Existing acceleration, speed, steering, wall, and footprint limits remain.
- Do not change ROS topic/service contracts or evaluation output schemas.
- Preserve user-generated `aichallenge/result-summary.json`.

## Definition of Done

- A one-cycle target classification miss does not produce Follow -> Cruise ->
  Follow or reset target-scoped pre-arm evidence.
- Recoverable SafeSeparation does not command target speed minus 2 m/s during
  its bounded tactical revalidation lease.
- Confirmed target-clear-ahead with a clear Return corridor leaves Pass through
  Return rather than the four-second FollowPrepare path.
- Hard-fault cases retain the existing fail-closed behavior.
- Package build and unit tests pass.
