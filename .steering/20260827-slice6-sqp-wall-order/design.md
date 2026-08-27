# Design

## Root cause

The old pipeline solved a broad affine problem, built narrow physical wall
trust buckets around that provisional solution, and only then replaced the
temporal dynamics with a nonlinear tangent from the refined iterate. The
buckets therefore certified one trajectory while the equality rows described
another. Reusing the previous primal and dual also supplied warm-start
provenance from the equality system that had just been removed.

Small certified negative residuals at zero velocity/progress-speed were a
second, independent representation mismatch: they are acceptable solver
residuals but invalid nonlinear tangent inputs.

## Canonical pipeline

1. Assemble and solve the broad semantic seven-state problem.
2. Project that iterate to exact state/input boxes only for tangent selection.
3. Replace temporal dynamics once.
4. Bootstrap the new affine problem by rolling out its own equalities.
5. Solve the relinearized problem.
6. Build progress-aligned and physical wall refinements around that result.
7. Add stage-wise dynamic-obstacle rows and solve.
8. Perform exact nonlinear actuator and swept-wall replay.

No alternate formulation or relaxed authority is introduced.

## Regression protection

- Adapter test covers a certified negative box residual at a zero bound.
- Shadow result validity requires the current-problem bootstrap.
- Existing wall, dynamic-obstacle, exact replay and retained-proof suites
  execute the reordered pipeline.
