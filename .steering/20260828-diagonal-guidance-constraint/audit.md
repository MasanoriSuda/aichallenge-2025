# Root-cause audit

## Observed phenomenon

The production Pass snapshot fails to form a certified artifact.  Stateless B
can solve only through partial-side rows and then collides in exact proof;
complete-axis C/D candidates fail before proof.

## Causal chain

```text
stage-wise obstacle producer
  -> only complete behind / complete side / complete ahead are representable
  -> steering and yaw-response lag leave no feasible direct axis transition
  -> strict C/D SQP becomes infeasible
  -> production partial escape substitutes wall-only lateral witness
  -> neither physical axis disjunct is guaranteed
  -> B solves an obstacle-unsafe trajectory
  -> exact dynamic proof rejects
  -> normal authority disappears
```

## Root cause

The convex candidate representation has no physically certified diagonal
transition between longitudinal and lateral separation.

## Contributing causes

- The left pass must begin within the first three stages in this frozen world.
- Steering/yaw-response lag makes complete side separation at the next stage
  unreachable.
- The right homotopy is wall-constrained before obstacle refinement.

## Mask and detection gap

- Mask: `partial_side_escape` lowers the obstacle row to the obstacle-free wall
  witness.
- Detection gap: that row is named and consumed as dynamic-obstacle
  refinement even though it proves reachability only; exact proof detects the
  mismatch downstream.

## Deterministic falsifier

If every diagonal E schedule had failed the unchanged exact proof, the
candidate-representation hypothesis would remain unproven.  Instead schedules
`0 -> 3` and `1 -> 3` formed complete certified bundles.

## Repair gate

The next production Slice must first reproduce one accepted E topology with a
physical-geometry-derived separating plane.  Promotion and removal of
`partial_side_escape` are one atomic change.  No new fallback, lease, grace,
clearance or solver tolerance is permitted.
