# Design

## Scope

This is a narrow ownership fix in the runtime wall preplan policy. It does not
alter wall clearance, target clearance, acceleration limits, or race-interface
contracts.

## Policy change

Add an explicit `connected_rearward_execution_hold_available` input to
`RuntimeWallPreplanRequest`. The controller sets it only when all of the
following already hold:

- phase is Pass and rear-clear is not yet confirmed;
- the locked target is continuous and at or behind the ego in course progress;
- current body footprints are separated;
- the target-bound replan hold is active and its physically validated prefix is
  executing.

Fresh same-side replacement, centerward contraction, and a validated Return
remain preferred. If the contraction was evaluated but unavailable, this
evidence changes only `ExitCurrentMission` to `HoldCurrentSide`.

## Safety ordering

`resolve_runtime_wall_preplan()` continues to return no override for hard wall
faults or invalid target state. The normal wall/transition guard therefore
retains ownership. The hold is also bounded by the existing target-bound hold
lifecycle and its time/distance budgets.

## Logging

The current-side hold log distinguishes a connected rearward execution hold
from the older pre-rear-clear Return suppression path.
