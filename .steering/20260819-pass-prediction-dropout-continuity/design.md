# Design

## Scope

This is an execution-authority integration fix. It does not add a new tactical
mode or make obstacle constraints softer.

## Dynamic-horizon classification

Replace the single conjunction used by `fresh_dynamic_horizon_available` with
a pure resolver that returns both availability and a typed failure reason:

- source/observation dropout: `TargetPredictionUnavailable`;
- identity discontinuity or position jump: `TargetDiscontinuous`;
- rejected course progress: `CourseProgressRejected`;
- blocked execution corridor: `ExecutionCorridorBlocked`;
- pass-side intrusion or lost lateral separation: `PredictedOverlap`.

This prevents a real corridor or overlap failure from masquerading as a
harmless prediction dropout.

## Bounded dropout continuation

When refresh fails only with `TargetPredictionUnavailable`, retain Pass for one
existing receding-horizon continuity lease if all of the following hold:

1. the target observation and last clear prediction are recent;
2. the committed dynamic prefix has not expired in time or distance;
3. the cached MPCC execution lease is still authoritative for the same Mission
   generation, side, and phase;
4. the current target is either absent or still physically separated; and
5. no target, map, emergency, solver, or immutable-budget hard fault exists.

No new path is invented during this lease. The controller continues only the
last feasible solution already accepted by the receding-horizon layer. If a
fresh target horizon returns, normal replanning resumes. If it does not, the
existing SafeSeparation/DynamicMissionWait/Recovery path remains in force.

## State and diagnostics

Track whether the dropout lease was active only to emit one start and one
release/revocation log. The state is reset whenever Pass ownership ends.
