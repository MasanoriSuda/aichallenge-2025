# Audit

## Observed symptom

`output/20260826-170617` contains 961 D1 and 522 D2 canonical Emergency
decisions.  These totals include downstream accident and Recovery tails and
must not be treated as one root cause.

The first independently useful D2 Cruise sequence is:

| decision | observation |
|---:|---|
| 3536 | certified six-state Cruise candidate selected |
| 3542 | retained proof `delay-prefix-blocked`; immediate Emergency Stop |
| 3553 | older artifact becomes `velocity-unreachable` after deceleration |
| 3570 | newer six-state candidate selected |
| 3571 | one-cycle authority hole; immediate Emergency Stop |
| 3572 | newer six-state candidate selected |

## Initial hypothesis

The normal owner consumes asynchronous retained proof before submitting the
next problem, while the exact synchronous six-state admission transaction is
owned by an intent-transition-only branch.  We initially hypothesized that a
same-intent retained rejection was only an asynchronous continuity hole.

## Dynamic falsification

The hypothesis was tested in `output/20260826-180846` by running the same exact
six-state certification/current-world transaction whenever retained authority
was unavailable.

- D1: 381 transactions, 25 joined, 356 rejected.
- D2: 312 transactions, 24 joined, 288 rejected.
- Rejections were predominantly real `dynamic-path-blocked` results or later
  solver failures, not missing asynchronous evidence.
- When the solver reached `maximum iterations reached`, the synchronous retry
  consumed roughly 60--70 ms in a 25 ms callback and repeated every cycle.

The generalized synchronous retry therefore violates the 40 Hz execution
contract and does not remove the physical blocker.  It is rejected and the
production source/test changes were removed.

## Confirmed contributing cause

Emergency's discontinuous speed command can invalidate the reachability
premise of older artifacts.  This amplifies a short proof gap, but the run
proves it cannot be repaired by solving synchronously on every rejection.

## Existing masks

- Emergency Stop hides the missing production transaction as a safety action.
- Stuck Recovery later moves the vehicle, which makes the original normal
  authority hole difficult to identify from the final stopped behavior.

## Next causal boundary

The next Gate investigation must classify the first normal-authority loss by
semantic reason before changing code:

- `dynamic-path-blocked`: determine whether the obstacle is a true forward
  blocker or an irrelevant rear/side vehicle included in the whole horizon;
- `delay-prefix-blocked`: determine whether the already committed prefix is
  actually wall-unsafe or the measured-to-control reconstruction is wrong;
- `steering/velocity-unreachable`: treat only as a downstream consequence
  when preceded by an Emergency discontinuity;
- solver failure: keep outside the control callback and preserve the last
  physically/current-world certified async result where it remains valid.

This slice is a documented rejected experiment, not a production fix.
