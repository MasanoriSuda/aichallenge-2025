# Requirements: time-aligned prepared-QP feedback

## Objective

Test whether the immutable refined QP produced by asynchronous preparation can
be reduced to its unconsumed absolute-time suffix and solved as a bounded
feedback step.

The preceding semantic-suffix experiment proved formulation correctness but a
second full solve would repeat 68--110 ms of work.  This Slice tests derivative
and constraint reuse.  Production authority remains frozen.

## Requirements

- Consume one common stage count from state, input, nominal path, wall and
  obstacle data.
- Shorten the active first stage; never restart its old duration.
- Replace x0 with the latest seven-state observation.
- Recompute the physical steering prefix for the shorter first stage.
- Relinearize the sliced problem around a current-problem-owned primal; do not
  reuse affine equalities from a different time origin.
- Preserve all surviving refined wall and dynamic-obstacle rows.
- Reject malformed row provenance instead of dropping a constraint class.
- No Store, publisher or production mailbox connection in this Slice.

## Forbidden fixes

- solver tolerance or maximum-iteration changes
- wall/vehicle clearance changes
- new lease, grace, timeout or fallback
- progress-only rebasing
- dropping wall/obstacle rows to obtain a feasible QP

## Exit gate

The prepared-suffix arm must solve the deterministic mixed-origin fixture and
must be materially cheaper than a second full `SolverContext::evaluate` before
runtime shadow integration is considered.
