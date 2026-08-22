# Design

## Immutable plan value

`CanonicalExecutionPlan` is a complete, immutable-by-convention value:

```text
plan_id
problem context
solution certificate
solved_sec
predicted states[0..N]
control stages[0..N-1]
```

The predicted state schema is exactly `[e_y, e_lag, e_psi, v, theta]`. The control schema is
exactly `[a, kappa, v_theta]` plus the stage duration used to advance the executable cursor.

## Store semantics

`CanonicalExecutionPlanStore::replace()` validates the complete value while holding the same mutex
used for replacement. It accepts only a plan ID newer than the stored plan. Failed validation or a
stale async result leaves the old shared immutable snapshot untouched.

`clear_if_plan_id()` prevents a delayed callback from clearing a newer plan.

## Cursor semantics

The cursor is resolved from `now_sec - solved_sec` and the exact per-stage durations. It returns a
real first stage and remaining count. Future time, expired certificate and a fully consumed control
sequence fail explicitly. The last control is never repeated by clamping.

## Current execution proof

A current-pose revalidation names:

```text
current decision ID
plan ID
first executable stage
remaining stage count
wall/obstacle physical certificate
```

Only an exact match to the cursor can create a `CanonicalNormalCandidate`. The authority selector
also checks the embedded current physical proof so a hand-built candidate cannot bypass the
factory's invariant.
