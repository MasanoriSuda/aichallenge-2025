# Audit: terminal retained execution identity

## Observed failure

In `output/20260828-005426`, the live OvertakeLine FSM completed Return and
entered Idle at `1787846142.118`.  A retained Return artifact nevertheless
became the tactical identity for each successor problem.  Sequences 2332 to
2398 recursively regenerated Return until the continuation became infeasible
next to the wall.

## Causal chain

1. A certified Return artifact remained numerically executable.
2. The caller checked live Mission identity only to calculate traveled
   distance.
3. The identity resolver treated cursor availability as tactical authority.
4. A fresh Return problem was built and published from the retained artifact.
5. That publication replaced the retained artifact and reset the same edge.
6. The synthetic Return chain survived the live Return -> Idle transition.

## Root cause and A-D classification

This is an authority/lifecycle defect.  Pipeline A fails because persistent
artifact lifetime was incorrectly allowed to create tactical lifetime.
Pipeline B does not retain Return after the current world enters Idle and
therefore removes the causal edge.  C and D are irrelevant to this family:
the seven-state solver repeatedly solved the synthetic problem.

Classification: **A fails, B succeeds — Mission lifecycle defect**.

## Change

The retained identity now requires two independent facts:

- its exact artifact cursor is executable; and
- target, generation, homotopy and phase still equal the live tactical state.

Idle, Recovery or any different phase reports
`retained-executed-artifact-superseded`.  The resolver then contributes no
tactical identity.  No timeout, lease, grace period, fallback, solver setting
or clearance was added.

## Verification

- `make autoware-build`: 25 packages succeeded.
- Full `multi_purpose_mpc_ros` tests: 49 targets, 2004 tests, zero failures.
- `make dev2`: `output/20260828-011708`.
- Recursive `Canonical executed-intent replenishment` records: zero in that
  run, versus the explicit sequence chain in `output/20260828-005426`.

The dynamic run did not reach a successful Return completion, so a direct
Return -> Idle replay remains part of the later integration Gate.  The run
instead froze two separate failures: Pass short-horizon unsafe and ShiftOut
hard-wall infeasibility.  They are not treated as regressions of this repair.

## Removed legacy behavior

An executable artifact can no longer manufacture a new Mission phase after
the live tactical state supersedes it.  Retention remains only a same-phase,
same-identity execution bridge.
