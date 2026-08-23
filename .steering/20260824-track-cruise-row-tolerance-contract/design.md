# Design

## Causal chain

```text
five-state Track/Cruise QP mixes metres, radians, speed and progress
-> dedicated Track/Cruise context is default-constructed
-> default PersistentOsqpSolver policy uses one global residual scale
-> a small actuator/input identity row can exceed its local tolerance while
   the whole QP is still returned as solved
-> execution-primal boundary checks the exact row and rejects it
-> fresh canonical plan is absent
-> emergency authority stops the vehicle
```

The strict execution boundary is the detector, not the cause. Clamping an
out-of-contract value or enlarging its tolerance would hide the mismatch.

## Existing counterexample and scope boundary

`.steering/20260822-osqp-rowwise-residual-admission` proved that changing the
shared solver boundary breaks the legacy three-state curvature-rate solve. The
selected change does not touch that shared/default solver. It applies the
already implemented row-normalized preconditioning only to a dedicated
canonical five-state context, matching Follow and Overtake.

## Experimental change

```text
ExtendedBranchSolverContext(policy)  // policy becomes mandatory

Track/Cruise -> kCanonicalPhysicalRowTolerancePolicy
Follow       -> kCanonicalPhysicalRowTolerancePolicy (unchanged)
Overtake L/R -> kCanonicalPhysicalRowTolerancePolicy (unchanged)
legacy MPC   -> default PersistentOsqpSolver policy (unchanged)
```

Making the constructor argument mandatory would convert a runtime omission into
a compile-time error. No second solver, authority, or fallback path was created
during the experiment.

## Dynamic result and decision

The experiment reproduced the previously recorded counterexample. In
`output/20260824-011002`, Track/Cruise alternated between certified and
solve-failure outcomes:

- Domain 1: 33 certified / 32 solve failures.
- Domain 2: 10 certified / 9 solve failures.
- Representative failures were input-bound rows 210/212 and 270 with
  `maximum_normalized_violation > 1`.

The vehicles could move only because successful and emergency cycles
alternated. This is not stable canonical authority. The experiment is removed
from production code. The strict boundary exposed an upstream QP convergence
or formulation defect; it did not cause that defect.

Extending retained authority is not the first correction: early retained
windows were also rejected as `invalid-progress-evolution`, while later
windows were rejected solely because the current proof requires an empty V2X
world. Broadening retained eligibility now would mask two separate upstream
contracts.

## Validation order

1. Source audit: all context constructors are explicit.
2. Existing mixed-unit row-normalization tests.
3. Full package test and build.
4. Bounded closed-loop `make dev2` gate.
5. Remove the experiment when the failure criterion is met.
