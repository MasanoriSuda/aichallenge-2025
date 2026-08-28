# Design: separate Mission-path proof from MPCC trajectory proof

## Root cause

`target_exclusion_certified` proves the upstream Mission/overtake-line path,
but `resolve_dynamic_obstacle_contract()` used it to remove the target from the
canonical MPCC problem.  The MPCC is free to produce a different trajectory,
so the certificate and the trajectory do not share one immutable problem
fingerprint.

The result is a proof-order inversion:

```text
Mission path excludes target
  -> target removed from MPCC
  -> MPCC optimizes only against wall
  -> final exact verifier is the first component to see the target again
```

The frozen Pass failure records exactly this split: target identity and fresh
replay-world data exist, while the lower QP has no dynamic-obstacle rows.

## Repair

`resolve_dynamic_obstacle_contract()` will use only:

- canonical normal scope and supported intent;
- complete stage-corridor target tube, preferred for ShiftOut/Pass; or
- complete current-world target tube.

It will no longer accept `target_exclusion_certified` as a release input.
The upstream certificate remains part of the Mission/execution identity and
other policy decisions, but cannot certify a trajectory optimized by a
different owner.

## Non-goals

This Slice does not yet promote the stateless ManeuverBundle or alter the
wall-before-obstacle refinement order.  Those are evaluated against the same
frozen snapshot after the ownership defect is removed.  If wall refinement
still fails before the now-required obstacle rows are solved, the next Slice
will replace that pipeline ordering rather than weaken its bounds.
