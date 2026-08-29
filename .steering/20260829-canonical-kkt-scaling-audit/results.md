# Results: canonical KKT scaling audit

## Global toggle falsification

A fixed explicit Ruiz iteration count is not a canonical replacement:

- ShiftOut fingerprint `9845010060330222052` needs at least three passes in
  the tested explicit implementation (`1150` iterations to solve);
- the known Follow sequence 5575 solves under the current formulation in
  `2325` iterations and after one extra pass in `2575` iterations, but three
  passes report primal infeasible and ten passes reach maximum iterations.

Raw physical coordinates with OSQP owning scaling are also not canonical.  On
ShiftOut they report solved in `125` iterations and pass exact proof, but on
Follow 5575 the returned primal exceeds the physical row contract by a
normalized factor `3.0985`.

Therefore neither a global internal-scaling toggle nor a fixed additional
Ruiz count can replace the current solver across all intents.

## Problem-class comparison

The typed observation-only policy keeps the existing physical box-coordinate
and row-tolerance transforms and then enables OSQP's standard ten modified-
Ruiz iterations.  The policy is source-contract isolated from the controller.

Using the recorded production warm start, the latest ShiftOut corpus produced:

| fingerprint | recorded stage | result | total solve | exact proof |
|---|---|---|---:|---|
| `9845010060330222052` | wall refinement | solved | `39.49 ms` | accepted |
| `7896913873338064473` | coupled wall/obstacle refinement | solved | `22.39 ms` | accepted |
| `5862539731343104692` | dynamic-obstacle refinement | maximum iterations | n/a | no bundle |

The two accepted results retained every recorded affine row and passed exact
nonlinear trajectory, swept wall, timed obstacle and terminal successor proof.
The dynamic-obstacle problem remained strongly infeasible (`0.705841` primal
residual and normalized physical violation `413.779`), showing that the policy
does not turn arbitrary failed problems into executable artifacts.

## Root-cause refinement

The numerical mismatch is formulation-specific.  Post-wall-refinement
ShiftOut QPs have a KKT balance which benefits from modified Ruiz
equilibration; the earlier Follow family does not.  A single global setting is
therefore the wrong abstraction.

The next production Slice may assign one canonical solver workspace to wall
and coupled wall refinement while keeping the existing canonical workspace for
initial, successive-linearization and dynamic-only problems.  This is a
problem-class ownership split, not a retry: each QP is submitted to exactly one
solver policy and no rejected result is promoted.

## Primary references

- OSQP documents `scaling` as the number of scaling iterations and uses ten by
  default: <https://osqp.org/docs/interfaces/solver_settings.html>
- The OSQP paper defines modified Ruiz equilibration over the KKT matrix plus
  an objective cost scale and the corresponding primal/dual transforms:
  <https://web.stanford.edu/~boyd/papers/pdf/osqp.pdf>
- Upstream implementation: <https://github.com/osqp/osqp>

## Verification

- observation-only solver, proof-chain and source-isolation tests: three
  suites, zero failures;
- both newly converged ShiftOut primals: exact proof accepted;
- physically infeasible dynamic ShiftOut: no bundle;
- production authority, solver selection, commands and parameters: unchanged.

## Decision

Accept the formulation-specific classification.  Reject global scaling
changes.  Proceed with a separate production Slice which atomically moves only
wall-refinement ownership to the equilibrated policy and leaves no retry or
legacy wall-solver branch.
