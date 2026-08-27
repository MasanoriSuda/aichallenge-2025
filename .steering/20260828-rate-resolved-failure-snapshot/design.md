# Design: Rate-resolved failure snapshot

## Earliest violated evidence invariant

An architectural classification requires the exact problem seen by the
solver.  Current text logs retain the worst row and residual but not the full
matrices, bounds, warm start, semantic reference or wall-grid bytes.

## Producer

`mpcc_rate_resolved_shadow::SolverContext` is the earliest owner that has all
of the following at once:

- immutable semantic `Snapshot`;
- current refined `AssemblyRequest`;
- exact assembled sparse QP;
- warm start supplied to OSQP;
- typed solve outcome.

Therefore the recorder is called at that boundary.  The controller is not
allowed to reconstruct a problem later from logs.

## Artifact layout

Each captured failure is a directory below
`mpcc_architecture_snapshots/`:

- `snapshot.yaml`: schema, identity, semantic request, refinement inputs,
  assembled QP, warm start and outcome;
- `wall-grid.bin`: exact signed row-major occupancy bytes when a grid exists.

Sparse matrices are stored as deterministic `(row, column, value)` triplets.
Vectors preserve IEEE double values through 17-digit text precision.

## Replay

The offline replay executable reads the assembled QP and invokes the same
`PersistentOsqpSolver`.  It supports the recorded warm start and a cold replay
of the same problem.  This separates a live warm-start/scheduling failure from
a failure of the frozen convex problem without altering solver settings.

## Non-goals

- This Slice does not implement B/C/D candidate planners.
- A QP replay alone cannot prove physical infeasibility.
- The recorder does not publish ROS topics or expose a control API.
