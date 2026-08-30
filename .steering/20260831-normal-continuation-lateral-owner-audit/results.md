# Results

## Observation

The frozen D3 Cruise snapshot fails in the post-solve physical wall
refinement, not in Mission selection or current-world obstacle binding.  Warm
and cold replay both reach OSQP maximum iterations.  The worst recorded row
is row 158, the stage-1 progress state box, but that row is a symptom of an
already empty refined feasible set.

An independent HiGHS feasibility check over the immutable recorded linear
constraints classified the QP as infeasible.  Isolating the row families
showed that the first physical swept-wall transitions and their narrow
physical pose buckets conflict with the seven-state dynamics and input
envelope.  At stage 1:

- refined lateral upper bound: `0.209006 m`
- minimum dynamics/input-reachable lateral: `0.235208 m`
- unreachable gap: `0.026202 m`

Removing solver tolerances or increasing iterations therefore cannot repair
this snapshot.

## Physical oracle

The comparison tool was extended only on the observation side:

- target-free Track/Cruise receives an audit-only terminal Replan/Stop
  successor so a missing target cannot masquerade as physical infeasibility;
- a physical nonlinear oracle reconstructs predicted states from the sealed
  current state and the candidate controls;
- artificial affine wall-bucket rows are not allowed to own this oracle;
- exact occupancy-grid wall, timed dynamic-obstacle and terminal Stop proofs
  remain mandatory.

Both the broad-problem solved warm start and the wall-refinement rejected
iterate pass the nonlinear model and every exact downstream proof:

| Input | Exact result | Terminal progress | Terminal velocity |
|---|---|---:|---:|
| broad-problem warm start | accepted | 12.9592 m | 7.9968 m/s |
| rejected refined iterate controls | accepted | 12.9253 m | 7.6498 m/s |

The second result uses only the rejected iterate's control sequence; its
affine predicted states are deliberately discarded and rebuilt by the
canonical nonlinear seven-state transition.

## Classification

`solve succeeds but proof fails: model/certificate mismatch`, with the
direction reversed at the duplicated boundary: a physically certified
control sequence exists, while the post-hoc physical wall bucket makes the
refined affine QP infeasible before exact proof can run.

This is not:

- a persistent Mission lifecycle failure (the source intent is target-free
  Cruise);
- a missing opposite homotopy (there is no active obstacle target);
- an OSQP tolerance or iteration-limit issue;
- evidence that the course is physically infeasible.

The production correction belongs in a separate Slice.  It must remove the
physical bucket as a second hard wall authority while retaining the broad
progress-aligned planning corridor and exact swept-footprint proof.  No
production authority was changed by this audit Slice.

## Verification

- Target build: `mpcc_architecture_compare` and
  `test_mpcc_architecture_comparison` passed.
- Target test: 29/29 comparison tests passed.
- Frozen warm/cold exact-QP replay: deterministic rejection.
- Frozen prepared-suffix families: no certified solution.
- Frozen physical nonlinear oracle: both control sequences accepted by exact
  proof.
