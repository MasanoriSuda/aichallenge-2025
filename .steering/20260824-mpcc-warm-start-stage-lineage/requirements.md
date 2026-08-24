# Requirements

## Objective

Bind every certified five-state MPCC warm start to the exact physical stage
advance between its source geometry and the current problem.  Repeated solves
of an unchanged stage geometry must not advance the warm trajectory, and a
multi-stage geometry advance must not be represented as a fixed one-stage
shift.

## Failure-first evidence

Baseline commit: `66a76c9`.

Run `output/20260824-165722/d1/autoware.log`, Domain 1:

- active ShiftOut began at decision 5997 / waypoint 172;
- approximately 20 fresh canonical solutions were stored while the tracking
  waypoint advanced only to 176;
- the first rejected bounded QP had a non-empty first curvature intersection
  and a primal-feasible maximum-iteration final iterate;
- dual residual `0.0233`, rather than a physical row violation, prevented a
  solved certificate;
- the retained plan subsequently failed current-world proof and Emergency /
  Recovery took authority.

Source inspection shows that rolling geometry compatibility computes an
overlap offset but returns only a boolean.  The solver then applies the generic
one-stage warm shift on every compatible solve.

## Repaired invariant

The warm-start primal and every dual stage block must be shifted by exactly the
same physical stage offset proven by the previous/current stage identities.
The applied offset is part of the warm-start resolution and runtime telemetry.

## Constraints

- Do not accept maximum-iteration iterates as executable solutions.
- Do not change OSQP iterations, tolerances, weights, wall margins, steering
  limits, controller rates or tactical thresholds.
- Do not add fallback, retry, feature flag or another normal authority.
- Preserve Emergency and Recovery behavior.
- Do not modify or commit `aichallenge/result-summary.json`.

## Definition of Done

- Deterministic tests cover zero-, one- and multi-stage geometry advance.
- State, input and all dual stage blocks use the resolved offset.
- Incompatible geometry still resets instead of guessing an offset.
- Static gates pass.
- Bounded dev2 shows the applied offsets and either removes the observed dual
  convergence discontinuity or falsifies this cause without tuning.
