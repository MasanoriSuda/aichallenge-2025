# Design

## Current problem

The latest-only worker computes the full tactical rollout, but the live
`evaluate_v2x_behavior()` invocation still computes the same left/right
`assess_side()` paths before adopting the asynchronous result. The latest run
therefore showed healthy worker completion/adoption while callbacks overlapping
worker activity still overran roughly half the time.

The asynchronous result also currently assumes that a synchronous
`SideAssessment` already exists and overwrites only a subset of it. Simply
skipping live assessment would lose curve permission, side clearance and
execution-admission inputs.

## Change

1. Promote the local side-assessment record to a copyable tactical snapshot
   carried by `V2XBehaviorOutput`.
2. A worker evaluation publishes both complete left/right assessments.
3. The live callback takes and validates the latest result before tactical side
   comparison, then imports both assessments atomically.
4. When the async worker is enabled, live `assess_side()` exits before static
   envelope, gap rollout, Frenet-DP and Mission generation. Cheap frozen
   ShiftOut/Pass continuity paths remain live.
5. The existing late authority block continues to select entry/replacement and
   cache the last feasible path. Submission remains latest-only and nonblocking.
6. Start-grid breakout retains its dedicated live corridor logic because it is
   a launch safety/coordination exception rather than ordinary tactical MPCC.

## Failure behavior

- No result or stale result: do not generate a synchronous replacement. A
  committed ShiftOut/Pass keeps its frozen Mission; an uncommitted entry stays
  in Follow until a fresh worker result arrives.
- Hard live fault: discard the worker result and let the current guard own the
  response.
- Async disabled: retain the existing synchronous path unchanged.

## Expected effect

The live callback no longer duplicates the worker's 70--180 ms tactical work.
Its remaining responsibilities are observation/context validation, result
admission, last-feasible execution and hard safety guards.
