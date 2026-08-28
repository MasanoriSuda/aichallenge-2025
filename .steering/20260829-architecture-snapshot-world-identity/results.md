# Results

## Detection-gap repair

- Build: 25 packages succeeded.
- Focused package test result: 2017 tests, zero failures.
- Run `output/20260829-070314` recorded current-world ShiftOut failure
  `seq649 / fingerprint a6f7c37f1de517c1` below the run output directory.
- The path contains the same immutable fingerprint stored in the v2 snapshot;
  an older run with the same sequence can no longer masquerade as current
  evidence.

## Frozen-world comparison

Snapshot:

`output/20260829-070314/d1/mpcc_architecture_snapshots/000000000649-a6f7c37f1de517c1-shiftout-wall-refinement-solve-rejected/snapshot.yaml`

- Persistent A: solver-rejected.
- Stateless left/right B: solver-rejected.
- Rough lattice C: no certified bundle.
- Existing offline continuation D: no certified bundle; every attempted
  continuation stopped at fraction zero because it invokes the same
  single-SQP wall-refined QP first.
- Production direct-side population and wall-restoration arms: solver-rejected.

The existing D arm is not an independent nonlinear feasibility oracle, so
all-arm failure is `Unknown`, not physical infeasibility.

## Independent convex feasibility check

SciPy HiGHS was applied observation-only to the exact frozen constraints
`l <= A*x <= u`, with the objective removed.

- wall-refinement snapshot: infeasible, 183 iterations;
- coupled wall/obstacle snapshot: infeasible, 183 iterations;
- dynamic-obstacle refinement snapshot: infeasible in presolve.

Removing all trailing wall rows did not restore feasibility. Removing either
the stage-wise heading state boxes or the stage-wise lag state boxes did.
Therefore the first empty-set producer is the simultaneous hard
heading/lag bucket emitted by physical wall refinement, not OSQP tolerance or
iteration count.

## Architecture conclusion

The next experiment must be an audit-only elastic/virtual-control feasibility
restoration or a genuinely nonlinear solve, followed by the unchanged exact
wall/dynamic certificate. Production must not soften the physical wall or
opponent proof. The direct cursor-zero experiment remains rejected and has
been deleted.
