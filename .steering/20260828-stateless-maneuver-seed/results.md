# IM-2 results

## Outcome

Accepted as an observation-only architecture component.  A replay-ready
Interaction Snapshot can now produce independent left and right seven-state
pre-solve seeds without reading persistent Mission geometry, phase state,
retained candidates, leases or runtime clocks.

This Slice does not claim that either seed is feasible.  It supplies the
missing B-arm problem input; IM-3 must run the unchanged SQP and exact proof
pipeline before any candidate can become a `ManeuverBundle`.

## Root-cause finding

The previous A/B comparison was blocked before solver invocation.  Its
supposed fresh candidate was optional output from the persistent Mission
lifecycle, so active ShiftOut snapshots commonly contained no independent B
candidate at all.  The earliest violated invariant was candidate ownership,
not an OSQP tolerance, wall margin or Mission resume timing issue.

## Implemented contract

- The producer is a pure function of one immutable Interaction Snapshot and a
  requested side.
- State zero remains the measured equality.
- Future lateral references are rebuilt from the target stage tube and the
  requested homotopy; Mission lateral and heading references are not reused.
- Velocity/progress references, costs, limits, timing, wall evidence and target
  stages remain unchanged so IM-3 isolates lifecycle/candidate geometry.
- The rebuilt problem context and complete candidate snapshot are resealed.
- A terminal successor is explicit: Return when visible in-horizon, otherwise
  semantic Stop only when the same problem permits braking to zero.
- Mixed observation generations, invalid sides, unavailable target horizons and
  absent terminal successors are rejected with typed reasons.

## Authority and complexity audit

- Production command authority changed: no.
- Controller link or runtime hook added: no.
- Publisher, mailbox, certified-plan store, lease, timeout, retry or fallback
  added: no.
- Solver policy, weights, clearance and configuration changed: no.
- New production exceptional path: zero.

The new library is linked only by its offline replay CLI and tests.  The
`mpc_controller_cpp` target does not link it.

## Verification

- Failure-first build failed on the intentionally absent stateless producer
  header before implementation.
- `make autoware-build`: passed, 25 packages.
- Focused `test_mpcc_stateless_maneuver`: passed, 5 tests.
- Full `multi_purpose_mpc_ros` package suite: 50/50 CTest targets and 1987
  tests passed with zero failures.
- Tests cover both homotopies, persistent-Mission reference independence,
  Return and Stop intent, invalid side, fingerprint mismatch, mixed epoch and
  terminal-successor rejection.
- `git diff --check`: passed.

## Dynamic evidence boundary

This Slice deliberately changes no live behavior.  A same-snapshot A/B result
does not exist yet; that is the single objective of IM-3.  No parameter tuning
or authority promotion is justified by IM-2 alone.
