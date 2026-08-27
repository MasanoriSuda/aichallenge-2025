# Design: Pass partial-escape reachability audit

## Current hypothesis

The Pass-side dynamic refinement enters `partial_side_escape` because the
longitudinal stay-behind branch is already empty.  It then requires every
predicted lateral separation to be no smaller than the separation at the
current state.  The seven-state model contains steering and yaw-response lag,
so immediate monotonic lateral escape is not generally reachable.

The sequence-1594 exact QP is feasible when its six dynamic-obstacle rows are
removed and infeasible when they are present.  Its first rows require the
current positive-side separation to be preserved immediately, although the
wall-only trajectory initially moves about 3.6 cm toward the target.

## Offline nonlinear check

`nonlinear_feasibility.py` loads the immutable snapshot, eliminates state
variables by rolling the exact temporal Frenet dynamics, and searches only the
bounded acceleration, steering-rate and virtual-progress controls.  It checks
the recorded state boxes, steering prefix, progress-aligned wall rows, swept
wall rows and dynamic-obstacle rows without modifying them.

This is a bounded D probe, not a proof of infeasibility.  A feasible result
proves that the affine problem/correction schedule lost a physical solution;
failure remains inconclusive.
