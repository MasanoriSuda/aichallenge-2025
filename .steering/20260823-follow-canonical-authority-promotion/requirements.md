# Follow canonical authority promotion requirements

## Purpose

Promote the dynamically accepted asynchronous Follow canonical plan to the sole normal command
authority for `ControlIntent::Follow`. Delete the same-cycle fall-through into the older
three-state/five-state normal solve path in the same Slice.

## Root cause

The worker and current-world proof already establish a complete canonical Follow command, but
`FollowShadowCycleResult` retains only readiness booleans. It discards the selected command,
problem, certified solution, plan cursor and world prediction. `get_control()` consequently logs the
proof and then continues into another normal solver. This is an ownership defect, not a parameter or
QP tuning defect.

## Required invariants

- A coherent `Follow` intent is owned only by the async canonical producer plus current-world proof.
- A current-world-certified command carries one matching problem, solution, plan, cursor and
  prediction to the final publication adapter.
- A Follow cycle never falls through to legacy MPC, three-state MPCC or a second five-state solve.
- Missing, stale, malformed or unsafe canonical Follow evidence fails closed through the canonical
  emergency supervisor; it does not activate a normal scalar/legacy fallback.
- Recovery and emergency overrides remain higher priority than canonical normal authority.
- Track/Cruise semantics and low-speed/overtake authority are not expanded in this Slice.
- No parameter, timeout, lease or migration feature flag is added.
- The user-owned `aichallenge/result-summary.json` is not modified or committed.

## Exit gate

- Static search shows no Follow-eligible path continuing into the legacy normal solve.
- Deterministic tests cover canonical selection, incomplete evidence and no-front fail-closed routing.
- `make autoware-build` and the package tests pass.
- Deterministic Follow replay publishes certified canonical Follow commands with matching intent and
  no legacy-normal authority during Follow intervals.
- Callback overruns, stale-result adoption and command mutation remain zero in the observed gate.

