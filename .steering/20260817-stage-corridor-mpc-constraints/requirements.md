# Requirements

## Purpose

Promote the hard lateral corridor already resolved by the live overtake
receding-horizon layer into the actual OSQP tracking problem.  The Frenet-DP
path remains a soft reference, while per-stage wall/opponent bounds constrain
the predicted `e_y` state directly.

## Evidence

- The 20260817-015957 run removed 40 Hz callback overruns, but only one of six
  D1 overtake entries completed `Pass -> Return -> Idle`.
- Failures were dominated by physical target/wall contradictions and
  post-entry physical revalidation, not tactical-worker latency.
- A six-metre progressive continuation preflight already exists, so extending
  another admission distance would duplicate policy and make entry more
  conservative without closing the execution-layer gap.

## Constraints

- Keep DP/Frenet output as the preferred lateral reference; do not turn it
  into an equality constraint.
- Intersect the live stage corridor with the existing track state bounds.  A
  malformed or contradictory corridor must fail closed before OSQP.
- Apply the new bounds only to active ShiftOut/Pass receding-horizon execution.
- Keep current wall, vehicle, emergency and actual-footprint guards.
- Keep persistent OSQP sparsity, warm-start behaviour, ROS interfaces and
  evaluation outputs unchanged.
- Provide a YAML switch for immediate A/B rollback.

## Target-loss finding

Do not infer rear-clear merely because the V2X target disappears.  Immediately
before the observed `SafeSeparation aborted: invalid input`, the last target
observation was still about 20 m ahead.  The existing fail-closed result was
therefore correct for that run; changing it to Return would create an unsafe
false pass completion.

## Acceptance criteria

- A validated receding horizon publishes one lower/upper lateral bound for
  every execution stage.
- The OSQP `e_y` bounds are the intersection of track and overtake stage
  corridors when the feature is enabled.
- The reference is clipped into the resulting interval, while remaining a
  soft quadratic objective.
- Invalid sizes, non-finite values or empty intersections are rejected
  deterministically.
- Unit tests cover disabled, valid intersection, clipping and invalid corridor
  cases.
- Package build and tests pass.
