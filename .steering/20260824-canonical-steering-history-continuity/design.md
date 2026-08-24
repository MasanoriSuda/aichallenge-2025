# Design

## Root cause

The five-state QP constrains its first curvature from the worker snapshot's
`previous_steering`. The resulting immutable plan is later revalidated against
current pose, wall, obstacle and Mission provenance, but not against the
current actuation history. A newer/other plan may have published meanwhile.
Consequently an internally certified asynchronous plan can replace the active
plan with a first steering command outside the current one-cycle reachable
set.

The first dynamic replay after adding selection-time rejection exposed the
upstream half of the same defect: pre-entry admission checked identity, age and
physical path proof, then raised `ShiftOut` before checking current steering
reachability. Rejection after authority elevation therefore produced an
Emergency stop. Gate A was incomplete, not the Emergency supervisor.

The corridor rejection and Emergency brake are downstream detection and
recovery. The source defect is the missing actuation-history proof at canonical
selection.

## Repair

1. Add a pure `CanonicalActuationContinuity` contract next to the immutable
   canonical execution-plan contract.
2. Its inputs are the current published normal steering, candidate steering,
   and the existing physical one-cycle steering step.
3. Reject malformed inputs or a candidate outside the reachable interval;
   never clamp the candidate.
4. Add the certificate to `CanonicalNormalSelection`; incomplete selections
   cannot reach publication.
5. Apply the contract after exact actuation extraction and before command
   construction in every fresh/retained canonical path.
6. Extend the pre-entry/replacement artifact admission contract to extract the
   live cursor actuation and apply the same reachability proof.
7. For a new entry, complete Gate A before freezing Mission state or changing
   phase. A rejection keeps Track/Follow; for a replacement it keeps the old
   Mission.

## Why this is not a patch

This does not special-case Overtake or the observed waypoint. It repairs the
shared async handoff invariant at the producer/selection boundary and prevents
the same stale-snapshot defect in Track/Cruise, Follow and Rejoin.

The Overtake-specific part is only authority ordering: the shared contract is
reused at the boundary where a tactical artifact would otherwise acquire
production authority. No steering clamp or secondary normal producer is added.

## Retained progress certificate alignment

The first instrumented rerun exposed a second contract inconsistency in the
same retained-plan boundary.  The fresh five-state solution had already been
accepted with a finite numerical constraint residual, but retained execution
later required progress to be monotone to a separate fixed `1e-9 m`
tolerance.  Observed regressions were only `-2.95e-8 m` and `-4.21e-9 m`,
while the corresponding immutable solver certificates allowed
`6.11e-5` and `2.05e-6` respectively.

The retained window now carries the accepted solution's maximum certified
constraint residual as immutable evidence.  Progress regression within that
certificate is treated as zero advance; regression beyond it still fails
closed and reports the exact stage and signed delta.  This changes neither a
runtime parameter nor solver acceptance: it removes a contradictory second
acceptance threshold downstream of an already certified solution.

## Non-scope

- actual steering-feedback model;
- MPCC weights or solver convergence;
- wall/corridor margin tuning;
- Overtake tactical side selection;
- Stuck Recovery behavior.
- atomic Cruise/Follow intent handoff while the first Follow worker result is
  still pending.  The final validation run made this separate ownership defect
  observable; it belongs to the next Slice rather than another exception in
  retained-plan validation.
