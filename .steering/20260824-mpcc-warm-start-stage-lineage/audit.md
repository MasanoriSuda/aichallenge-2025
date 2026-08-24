# Root-cause audit

## Observed symptom

The first active ShiftOut in `output/20260824-165722/d1/autoware.log`
produced many fresh canonical solutions at nearly unchanged tracking geometry,
then reached OSQP maximum iterations with a primal-feasible final iterate and
dual residual `0.0233`.  Retained current-world proof and normal authority
failed downstream, after which Emergency and Recovery appeared in the log.

## Causal chain

1. The canonical producer recomputes faster than the horizon's first physical
   course stage advances.
2. `rolling_stage_geometry_compatible()` finds the exact overlap offset but
   reduces it to a boolean.
3. `solve_extended_progress_problem()` consumes the certified solution and
   calls a helper that unconditionally shifts it by one stage.
4. Repeated solves on unchanged geometry therefore progressively misalign the
   primal and all dual stage blocks from the current QP rows.
5. OSQP receives a structurally valid but physically misaligned warm start.
   The observed first failure is dual-convergence failure, not a demonstrated
   empty curvature or wall feasible set.
6. Retained-plan rejection, Emergency and Recovery are downstream masks, not
   the producing defect.

## Competing hypotheses

### Empty first-stage curvature intersection

Refuted for the first failure.  Logged curvature box and curvature-rate
intervals have a non-empty intersection.

### Physical wall/corridor infeasibility

Not supported for the first solver rejection.  The final iterate's worst
physical constraint row is within its logged tolerance.  A later retained-plan
current-world rejection is separate downstream evidence.

### Insufficient OSQP iterations or loose tuning

Not established.  Raising iterations or changing tolerances would mask a
lineage error and is excluded from this Slice.

### Warm-start physical stage misalignment

Supported by both source and runtime cadence.  The source loses the computed
offset, while roughly twenty accepted solves occurred across only a few
tracking-waypoint advances.

## Repaired invariant

Compatibility and alignment are one certificate: a warm artifact is reusable
only with the exact overlap offset returned from the same previous/current
stage identities.  The state primal, input primal and every dual stage block
use that one offset.

## Removed legacy assumption

`one successful solver invocation == one physical horizon stage` is removed
from canonical MPCC execution.  No timeout, retry, fallback, flag or tuning
parameter replaces it.
