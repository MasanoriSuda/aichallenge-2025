# Requirements

## Objective

Remove the post-hoc physical wall bucket as a second production wall
authority.  A normal seven-state candidate which passes the canonical
nonlinear rollout and exact swept-footprint wall proof must not be rejected
solely because the local affine bucket is unreachable.

## Frozen evidence

- Source run: `output/20260831-043038/d3`.
- Snapshot: sequence 2177, target-free Cruise, wall-refinement solve rejected.
- Recorded refined QP is linearly infeasible.
- Stage-1 refined lateral upper is `0.209006 m`; dynamics/input reachable
  minimum is `0.235208 m`.
- Broad-problem and rejected-iterate controls both pass canonical nonlinear
  reconstruction, exact wall, dynamic-obstacle and terminal Stop proof.
- Root cause: model/certificate mismatch, not Mission lifecycle, solver
  tolerance or physical infeasibility.

## Constraints

- Do not change wall clearance, footprint, solver settings, weights, horizon,
  lease, grace, timeout or fallback behavior.
- Keep the progress-aligned planning corridor in the QP.
- Keep exact occupancy-grid swept-footprint proof as mandatory final wall
  authority.
- Preserve old hard physical bucket variants only behind observation-only
  architecture audit entry points.
- Do not alter publisher or production authority selection.
