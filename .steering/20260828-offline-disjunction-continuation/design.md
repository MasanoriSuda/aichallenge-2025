# Design: Offline disjunction continuation candidate D

## Evidence boundary

- Rollback baseline: `e9323a1b`
- Frozen run: `output/20260828-094214`, Domain 1
- Frozen failure: decision 1566, Pass wall-refinement solve rejected
- Interaction fingerprint: `7246006054995400977`
- Evidence type: deterministic offline replay

## Direct versus continuation comparison

For one `(side, first_side_stage, first_ahead_stage)` schedule, the final
dynamic rows are always the complete physical disjuncts.  A cold SolverContext
evaluates that final problem first.

If direct evaluation fails, a separate SolverContext evaluates bounded
fractions `0.00, 0.25, 0.50, 0.75`, followed by the exact `1.00` problem.
At an intermediate fraction each hard bound interpolates from the current
wall-only witness to the final complete-disjunction bound.  Dynamics, wall
refinement and SQP are re-evaluated on every step.  This is an offline
constraint homotopy; it is not a clearance change because the final candidate
uses the original exact bound.

The direct and continued final candidates share the same final fingerprint.
Continuation success is valid only when direct final failed, every
intermediate SQP solved, and final fraction `1.00` passes all exact proofs.

## Classification

- direct final succeeds: the rough C reference/candidate producer was the
  blocker; do not credit multi-SQP;
- direct final fails, continuation final succeeds: single-SQP basin/path
  limitation;
- numerical final solve succeeds, exact proof fails: model/certificate
  mismatch;
- all bounded schedules fail: physical feasibility remains `Unknown`, then
  use a nonlinear feasibility solve or improve the bounded certificate.

## Non-scope

- No production retry, solver-iteration or tolerance change.
- No new Mission lifecycle rule, lease, timeout, fallback or authority.
- No parameter tuning.
- No promotion of an offline artifact.
