# Audit: Rate-resolved model/proof unification

## Observation

Snapshot 890 solves its final QP but exact publication proof rejects sample
289. Production replay ends at lateral `-2.457419 m` against lower bound
`-2.241118 m` (tolerance `0.020937 m`).

The corrected offline D probe now uses the same <=10 ms midpoint replay and
interpolated bounds. It finds a feasible command sequence with minimum hard
slack within numerical zero and dense lateral reserve `0.147893 m`.

## Causal chain

1. SQP linearizes a coarse one-step stage model.
2. That model holds the stage-start heading in the lateral update.
3. Publication proof integrates steering/yaw response in <=10 ms substeps.
4. The optimized QP state and certified nonlinear state diverge over the
   horizon.
5. Re-linearizing the coarse map cannot remove a mismatch to a different map.
6. Publication correctly rejects the unsafe artifact.

## Classification

`solve succeeds but proof fails: model/certificate mismatch`.

## Verification

### Frozen snapshot

The offline replay script now evaluates the exact production-equivalent
midpoint transition.  Eight SLSQP starts found a physically feasible sequence
for the frozen bounds, with dense lateral reserve `0.147893 m`.  This rules out
physical infeasibility for the representative failure and classifies it as a
model/certificate mismatch.

### Static verification

- `make autoware-build`: passed (`25` packages).
- Focused C++ suites:
  - rate-resolved model: `7/7`
  - physical adapter: `12/12`
  - request adapter: `16/16`
  - shadow pipeline: `26/26`
  - problem assembly: `15/15`
- package CTest discovery: `49/49` targets passed.

The first-stage feasibility diagnostic was also corrected.  The canonical
nonlinear stage map couples virtual progress with the other controls, so a
separable-row check may now be *inconclusive*.  Coupled rows are no longer
misreported as a proof of infeasibility.

### Dynamic Gate

`make dev2` produced `output/20260828-022111`.

- Track/Cruise exact physical proofs were accepted repeatedly.
- The previous `exact-trajectory-rejected` lateral-bound signature did not
  recur in the run.
- Control callback examples remained below the 25 ms control period
  (`max=13.851 ms`, no callback overrun in the emitted runtime summaries).
- Overtake reached `Idle -> ShiftOut -> Pass`, proving that the canonical
  transition did not prevent production entry.

The run also exposed two later, distinct defects which are intentionally not
hidden inside this repair:

1. One refined Track/Cruise solve was reported as
   `invalid-artifact/exact=accepted`.  The builder currently discards its
   concrete artifact validation reason, so this is a diagnostics/data-flow
   defect to fix before changing behavior.
2. The observed Pass was interrupted by the external front-risk Emergency
   authority.  This is not evidence against the transition unification; its
   final decision inputs must be audited as the next integration Slice.

## Removed duplicate responsibility

The publication proof no longer owns a second implementation of yaw-response
and Frenet propagation.  SQP tangent construction and physical replay now call
the same canonical nonlinear transition.  The proof remains an independent
consumer of the transition and therefore still rejects an unsafe solved
trajectory; only the conflicting model implementation was removed.
