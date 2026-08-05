# Design

## Root cause

The admission planner currently stores only the transition side and distance
window. At runtime `try_refresh_committed_pass_horizon()` derives a new goal
from the latest target lateral estimate and forces target-center separation.
For a behind-target cross-track handoff this can require a goal outside the
wall-feasible interval even though the actual vehicle footprints do not
overlap.

## Frozen transition contract

The admitted mission additionally stores:

- the fixed opposite-side lateral goal;
- the statically validated lateral-ramp distance.

The goal is a minimum-motion mirrored continuation of the initial outside
goal, clamped only inside the wall-feasible bounds at the planned handoff
origin. It must still lie on the requested Frenet side.

The admission planner starts a second static preflight at the scheduled
handoff origin. It validates:

1. old outside goal to frozen new outside goal;
2. the remaining Pass distance to predicted rear-clear;
3. Return to the racing line;
4. wall footprint, curvature and lateral-acceleration limits.

Target-center separation is not imposed on this second lateral goal because
the handoff crosses behind a longitudinally separated target. Runtime retains
the stronger conditions: fresh continuous target, separated current and
predicted footprints, no emergency risk, and target longitudinal distance at
shift completion greater than `target_intrusion_guard_distance`.

## Execution

The scheduled handoff calls the existing atomic replacement/commit path with
the frozen goal and validated shift-distance cap. The path is rechecked against
the current wall horizon and dynamic rollout, but the goal itself is not
recomputed from target lateral motion.

The rolling fallback may continue to calculate a new goal because it was not
part of the admitted schedule. It remains bounded by its existing cooldown and
count limits.

## Diagnostics

Mission selection and `PassPlan frozen` logs include the frozen handoff goal
and shift distance. Candidate rejection distinguishes transition static
preflight failure from the existing outer-window failure.

