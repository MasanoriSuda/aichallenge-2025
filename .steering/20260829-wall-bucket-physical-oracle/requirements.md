# Requirements: wall-bucket physical oracle

## Objective

Classify the frozen ShiftOut failure without changing production authority.
An independently solved primal with one artificial post-hoc wall pose bucket
removed must be checked by the unchanged exact trajectory, physical wall,
dynamic-obstacle and terminal-successor proofs.  If neither single-bucket
relaxation certifies, an exact-dynamics control-only oracle may omit both
artificial pose buckets, but it must return to those same C++ proofs.

## Constraints

- Do not change production candidates, solver settings, tolerances, clearance,
  Mission lifecycle or command publication.
- Relax only the selected lag or heading state-box rows in the recorded QP.
- Preserve dynamics, actuator, lateral/progress wall, swept-wall and opponent
  constraints.
- A relaxed numerical solution is not accepted unless all exact proofs pass.
- The audit API has no store, mailbox, controller or publisher access.
- Physical-oracle mode may skip recorded affine-row residuals only for states
  rebuilt by the exact nonlinear seven-state rollout. It may not skip the
  artifact, exact trajectory, wall, obstacle or successor proof.

## Definition of Done

1. Exact mode continues to reject a primal that violates any recorded row.
2. Bucket-oracle mode identifies its modified problem in the fingerprint.
3. Both bucket variants run the same exact proof chain.
4. The current frozen failure is classified from immutable evidence.
5. A nonlinear-oracle primal cannot create a bundle if an exact proof rejects.
