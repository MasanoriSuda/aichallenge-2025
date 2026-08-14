# Design

## Problem

The controller enters `FollowPrepare` with the rolling-replan cause
`DynamicMissionWait`, but the executor is currently owned by a branch guarded
with `!tactical_rolling_replan_runtime_active`.  The two conditions are
mutually exclusive in the failing run, so the forward-prefix resolver never
executes and the vehicle falls back to the 0.5 m/s closing limit.

## Change

Introduce a small pure core predicate for runtime ownership:

```text
behavior_overtake &&
(!tactical_rolling_replan_runtime_active || dynamic_mission_wait_active)
```

The controller uses this predicate after all fully preflighted current-side,
cross-side, and opponent-side replacement commits.  Therefore a fresh atomic
replacement still wins; otherwise an active wait reaches its existing
resolver and forward-prefix logic.

## Safety boundary

- Tactical rolling replan without an active dynamic wait remains excluded.
- Non-overtake behavior remains excluded.
- The existing executor continues to fail closed for target discontinuity,
  actual footprint overlap, wall infeasibility, and controller hard faults.

