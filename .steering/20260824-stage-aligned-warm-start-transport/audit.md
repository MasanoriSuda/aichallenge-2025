# Audit

## Pre-change evidence

- `output/20260824-014849` produced 165 certified and 168 rejected D2
  Track/Cruise solves.
- Every rejected result used a certified warm start; every following cold solve
  certified, proving rejected history no longer persists.
- 151/168 rejected rows were stage-zero acceleration and 16 were predicted
  velocity.
- `resolve_shadow_warm_start()` accepts both identical stage geometry and
  rolling overlaps, but returns only a boolean.
- `solve_extended_progress_problem()` consequently calls
  `shift_mpc_warm_start()` with an unconditional one-stage transform.

## Root-cause statement

The current warm-start compatibility check proves one spatial correspondence,
then the transport applies a different hard-coded correspondence.  The
resulting primal and dual belong to the wrong stage rows.  Execution-primal
rejection is the downstream detector, not the source of the defect.

## Post-change evidence

The experiment built successfully (25 packages) and its full controller test
suite passed after correcting one test fixture layout mistake (40 programs,
1679 tests, zero failures).

Bounded runtime evidence is `output/20260824-020904`. AWSIM did not expose the
admin start subscriber, so this is the same pre-race closed-loop condition as
the immediately preceding warm-start evidence. Domain 2 produced:

- 200 certified Track/Cruise outcomes;
- 212 execution-primal rejects;
- 195 warm rejects and 17 cold rejects;
- 189 warm outcomes used `stage_shift=0`, while only eight used
  `stage_shift=1`;
- among rejected warm/zero-shift outcomes, 119 were predicted velocity, 44
  acceleration, 17 virtual progress speed and eight curvature.

The intended no-shift transport was therefore active in almost every warm
cycle, but it preserved rather than removed the failure.  Cold curvature and
velocity rejects also demonstrate that warm transport is not the full cause.

## Conclusion

The fixed one-stage transform is semantically incomplete, but it is not the
root cause of the current production rejection loop.  The hypothesis is
rejected for this migration gate and all source/test changes were removed.
No tolerance, bound, retry, fallback or configuration change was retained.

The next root-cause Slice must inspect why OSQP success is accepted under its
configured convergence contract while individual physical box rows still
exceed the downstream tolerance, including cold solves.  It must compare
solver settings, scaled/unscaled residuals and the exact row certificate before
changing any numerical parameter.
