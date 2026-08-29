# Design: ShiftOut rejected-iterate proof

`PersistentOsqpSolver` already reconstructs the finite physical-coordinate
primal at maximum iterations to report its worst row.  The vector is currently
local to the diagnostic lambda and is discarded.  Preserve the same vector in
`SolveOutcome::rejected_primal` while keeping `result == nullopt`.

The observation-only architecture CLI adds one mode which replays the frozen
exact QP, selects its rejected primal, and passes it to the existing external
primal proof chain.  This reuses the original affine row check and every exact
physical/terminal proof.  No controller code consumes the field.

Classification:

- affine and exact proofs accept: numerical/KKT acceptance mismatch;
- affine accepts but exact proof rejects: model/certificate mismatch;
- affine rejects: failed iterate is not a feasible candidate; continue the
  offline nonlinear/multi-SQP audit;
- replay yields no finite rejected primal: evidence gap remains `Unknown`.
