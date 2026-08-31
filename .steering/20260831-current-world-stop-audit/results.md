# Results: current-world Stop audit

## Observed failure chain

In `output/20260831-134900/d1`, decision 1563 was still in committed Pass.
The retained normal artifact first lost rate/steering reachability.  Its fixed
terminal contingency then hit the exact wall.  No fresh normal candidate
joined, so the single publisher emitted the explicit emergency Stop and
cleared the normal execution ledger.

## Frozen A/B/C/D result

Frozen source:

`output/20260831-134900/d1/mpcc_architecture_snapshots/000000001563-8903ab412b2e24f9-pass-side-negative-physical-proof-terminal-contingency-unavailable/snapshot.yaml`

The unchanged comparison rejected persistent A, stateless B, rough/lattice C,
offline multi-SQP D and production G.  All reached the same OSQP maximum-
iteration family at a steering-rate input row.  The old Stop comparison then
reported both Stop arms as unavailable because it first required the same
normal solve to succeed.  Therefore the old all-fail result did not prove
physical Stop infeasibility.

## Root cause and correction

The observation-only Stop audit used publisher-boundary construction while the
live independent worker used current-world construction.  This was a model
comparison blind spot, not a reason to change authority or parameters.

The comparison now derives maximum braking directly from the immutable
current-world snapshot.  A regression fixture makes the normal future-speed
schedule unreachable and verifies that the direct Stop and shared 68-candidate
control lattice are still evaluated.

## Reclassification

After the correction:

- direct current-world seven-state Stop: solver rejected;
- 68-candidate steering-rate Stop lattice: all rejected, best failure at the
  stage-5 steering-rate prefix row;
- rejected-iterate physical nonlinear oracle: exact wall contact about
  5.624 m ahead, with terminal velocity effectively zero;
- KKT equilibration audit: still solver rejected.

Decision 1563 is already beyond the viable region for every implemented live
and offline candidate.  It does not establish that every nonlinear control is
physically impossible, but it does establish that adding a resume rule or
relaxing clearance at decision 1563 is not a causal fix.

The same run had an earlier certified current-world Stop plan, but by decision
1563 its retained join failed as `steering-unreachable`.  The next Slice must
join producer decision, source age and consumer decision and freeze the first
state where a certified Stop becomes unreachable.  That is the remaining
scheduling/lifecycle hypothesis.

## Verification

- `make autoware-build`: 25/25 packages succeeded.
- Package CTest: 59/59 passed.
- Aggregate package test result: 2321 tests, zero errors/failures.
- Frozen terminal Stop comparison exercised both current-world Stop arms.
- Production authority and all solver/wall/vehicle parameters are unchanged.
