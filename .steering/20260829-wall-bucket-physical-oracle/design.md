# Design: wall-bucket physical oracle

The recorded wall-refined QP contains a state-box block after the initial
state equalities. The oracle copies the frozen problem and replaces only the
selected lag or heading box rows with infinite bounds. It then verifies an
external primal against that copied affine problem.

After affine verification, the existing execution-artifact adapter reconstructs
the exact nonlinear trajectory. The existing physical wall and current-world
dynamic proofs remain authoritative. Therefore the result distinguishes:

- relaxed QP plus exact proofs succeeds: post-hoc bucket formulation defect;
- relaxed QP solves but exact proof fails: model/certificate mismatch;
- relaxed QP remains unsolved: candidate/backend limitation.

This is observation-only architecture evidence, not a production bucket
bypass or fallback.

## Nonlinear physical oracle

The two independently solved one-bucket QPs are only seeds.  A bounded
control-only optimizer replays the same nonlinear seven-state model at at most
10 ms substeps.  It retains input limits, physical lateral/velocity/progress
and actuator bounds, the steering prefix, progress-aligned and swept wall
rows, and dynamic-obstacle rows.  Lag and heading state boxes are excluded
because those are the post-hoc wall buckets being audited.

The resulting knot states and controls form an external primal.  The C++
comparison mode deliberately does not claim that this primal satisfies the
recorded affine QP; instead it runs the normal execution-artifact validation,
exact nonlinear trajectory reconstruction, physical wall sweep, current-world
dynamic proof and terminal-successor proof.  It owns no production API.

This creates the decisive classification:

- nonlinear primal and exact proofs accept: physical maneuver exists and the
  single-SQP/post-hoc-bucket formulation is defective;
- nonlinear optimizer reports feasible but C++ proof rejects: offline model
  or certificate mismatch;
- nonlinear search cannot find feasibility: inconclusive until a stronger
  offline NLP/global solve is attempted.
