# Requirements

## Objective

Classify why a physically feasible wall-bucket-relaxed ShiftOut problem still
cannot optimize the original racing objective within the fixed OSQP boundary.

## Frozen invariants

- Production authority and normal seven-state solve remain unchanged.
- Racing objective, physical rows and exact proof tolerances remain unchanged.
- No iteration, tolerance, clearance, timeout, lease, fallback or Mission
  change is permitted.
- Every independent primal must be checked in physical coordinates and then
  passed to the existing exact wall, dynamic-obstacle and terminal-successor
  proof chain.

## Comparison

For fingerprints `a6f7c37f1de517c1` and `145d1159f38a6ea9`, omit only the
artificial lag bucket already isolated by the preceding Slice and compare:

1. current explicit box/row scaling with OSQP internal scaling disabled;
2. raw physical coordinates with OSQP owning its standard scaling;
3. the explicitly transformed problem with OSQP internal scaling enabled,
   observation-only, to test rather than promote double scaling;
4. independent qpOASES, ProxQP and HiGHS conic backends where available.

## Exit classification

- independent backend solves and exact proof accepts: current OSQP/backend or
  preconditioning mismatch;
- all convex backends fail but nonlinear solve succeeds: convexification or
  single-SQP limitation;
- solve succeeds but exact proof fails: model/certificate mismatch;
- all exact formulations fail: physical infeasibility remains possible.

## Deletion milestone

Offline comparison scripts may remain as steering evidence.  Any C++ external-
primal bucket-proof hook must be deleted when this Slice closes, unless it is
promoted as the sole canonical formulation and replaces the old bucket path in
the same production Slice.
