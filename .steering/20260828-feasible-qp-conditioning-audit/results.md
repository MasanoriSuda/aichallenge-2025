# Results

## Observed failure

Frozen Follow snapshot 5575 is an exact 207-variable, 594-row QP. The
unchanged C++ replay reproduced the production failure:

| replay | status | iterations | dominant residual |
|---|---|---:|---|
| warm | maximum iterations | 4000 | progress-rate box row 317 |
| cold | maximum iterations | 4000 | acceleration box row 294 |

SciPy HiGHS independently found an affine feasible point with maximum row
violation `3.33e-15`. Warm-start provenance and mathematical infeasibility are
therefore falsified for this snapshot.

## Numerical audit

The recorded problem already supplies two explicit, physically traceable
preconditioners:

1. variable coordinates derived from physical box widths;
2. constraint-row scaling derived from each row's physical acceptance
   tolerance.

After those transforms, OSQP still applied its default ten Ruiz scaling
iterations. The resulting audit measurements were:

- variable-scale ratio: `230`;
- row-scale ratio: `17.42`;
- physical Hessian condition estimate: `6.02e6`;
- explicit solver-coordinate Hessian condition estimate: `2.37e7`;
- explicit solver equality condition estimate: `570.9`.

Removing duplicate row expressions did not restore convergence. Removing the
recorded warm start did not restore convergence. Merely allowing more
iterations solved only after approximately 10,175 iterations and would hide
the ownership error.

The decisive falsification was performed on the same explicitly transformed
`P/q/A/l/u`:

| copied problem | internal OSQP scaling | result |
|---|---:|---|
| snapshot 5575 | 10 | maximum iterations at 4000 |
| snapshot 5575 | 0 | solved at 2050 |

Across six recent mathematically feasible frozen snapshots, internal scaling
10 solved two and scaling 0 solved four. Two were solved only with scaling 0;
none was solved only with scaling 10. Two remained unsolved under both and are
not claimed to be fixed by this Slice.

## Root cause

`RowToleranceNormalized` already owned the complete coordinate transformation
presented to OSQP, but OSQP's internal scaler was allowed to transform it a
second time using a different numerical objective. This split
preconditioning ownership and caused some feasible, tightly active MPCC QPs
to stall on dual convergence.

This is not a request to loosen tolerance or raise the iteration limit. It is
a dataflow correction: one policy now owns one numerical transform.

## Implemented correction

- `RowToleranceNormalized` disables OSQP internal scaling after applying its
  explicit variable and row transforms.
- The unconditioned/default solver policy retains OSQP's internal scaling.
- A regression assertion records both sides of that policy boundary.

No physical constraint, objective weight, clearance, solver tolerance,
iteration limit, authority, fallback, lease or timeout changed.

## Verification

- `make autoware-build`: passed, 25 packages.
- focused solver/rate-resolved tests: 5/5 passed.
- full package CTest: 52/52 passed.
- corrected C++ replay of snapshot 5575:
  - warm: solved, 2000 iterations;
  - cold: solved, 2400 iterations.
- offline audit scripts compile and reproduce the aggregate comparison.
- `git diff --check`: passed.

## Remaining concern

Two other affine-feasible recent snapshots remain unsolved with either
internal scaling policy. They must be classified separately; this correction
must not be expanded into an iteration or tolerance change. Dynamic race
acceptance also remains blocked by the separately recorded dev2 startup
handshake issue.
