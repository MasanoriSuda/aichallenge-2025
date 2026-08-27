# Requirements: terminal retained execution identity

## Objective

Remove the lifecycle defect which allows a published Overtake artifact to
recreate its own tactical intent after the live Mission has completed.

## Evidence

In `output/20260828-005426`, Return completed at decision 2918/waypoint 210 and
the live FSM entered Idle.  Nevertheless sequences 2332 through 2398 were
published as Return because each newly executed artifact replenished the next
problem identity.  At decision 2997 that synthetic Return chain became
physically unrecoverable near the right wall.

## Constraints

- Do not change clearance, solver tolerance, horizon, timeout, lease, grace or
  fallback policy.
- Do not change production authority except to remove authority which survived
  a terminal tactical transition.
- A retained artifact may bridge only the same live target, generation,
  homotopy and phase.
- Idle, Recovery and a different Overtake phase supersede the retained
  execution identity.

## Definition of Done

- The pure identity resolver distinguishes a current retained bridge from a
  superseded artifact.
- Unit tests cover same-phase bridging and terminal/different-phase rejection.
- `make autoware-build` and the package test suite pass.
- A dynamic run shows that Return completion cannot self-replenish Return.

