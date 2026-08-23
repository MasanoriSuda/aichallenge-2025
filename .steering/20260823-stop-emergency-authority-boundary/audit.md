# Stop emergency authority boundary audit

## Observed evidence before correction

Deterministic Follow production replay
`output/20260823-202408-follow-production-replay/d1/autoware.log` contains two
`legacy-mpc-solved` final traces. Both are SafetyBrake decisions with `intent=stop`; neither is a
Follow leak.

One trace reports:

```text
action=safety-brake
intent=stop
formulation=legacy-spatial-mpc-3state
authority=legacy-normal-bypass
```

This proves that Stop provenance is correct but its authority boundary is missing.

## Causal chain

```text
front risk selects SafetyBrake
  -> orchestrator resolves Stop
  -> get_control has no Stop branch
  -> old normal solver supplies a command
  -> final speed/brake post-processing turns it into a stop
  -> final trace exposes split emergency/normal ownership
```

## Rejected alternatives

- Relabeling DynamicWait as Hold.
- Adding a nominal Stop QP for an emergency-only producer.
- Retaining the legacy solve as a steering fallback.
- Adding a configuration switch or tuning braking/clearance parameters.

## Implemented correction

- `resolve_stop_authority_action()` defines an exclusive boundary: only Stop is captured and it can
  only select EmergencyStop.
- `get_control()` returns at that boundary before low-speed direct control and every normal solver.
- The existing canonical emergency path requests zero speed, bounded held physical steering and
  configured maximum braking. Final Recovery arbitration remains unchanged.
- `FinalControlDecisionRequest` now carries a separate `supervisor_intent`. Emergency provenance no
  longer depends on inventing or completing a normal MPCC problem context.

The first post-change replay proved that legacy Stop had been removed, but exposed
`authority=emergency-override, intent=unknown`. That was not accepted as complete: the missing
supervisor provenance was fixed structurally and the replay was repeated.

## Static evidence

- `StopEmergencyAuthorityNeverBorrowsNormalControl` covers Stop, Hold, Follow and Cruise routing.
- `EmergencyOverridePreservesExplicitSupervisorIntent` proves Stop provenance without solver
  identity.
- `make autoware-build` completed 25 packages successfully after the final change.
- Package CTest completed 39/39 test programs; the final result base reports 1617 tests, zero errors
  and zero failures.
- No parameter, timeout, lease, fallback or migration feature flag changed.

## Deterministic replay evidence

Accepted replay:

`output/20260823-214300-stop-authority-replay-v2/d1/autoware.log`

Results:

- 10 final Stop traces were observed.
- 10/10 reported `authority=emergency-override`, `intent=stop`,
  `formulation=unresolved`, `canonical=satisfied` and
  `solver=canonical-stop-emergency/safety-brake-stop-authority`.
- 0/10 Stop traces used `legacy-mpc-solved`; the complete replay contained zero
  `legacy-mpc-solved` traces for any intent.
- Commands requested 0 m/s and -3.00 m/s2 while retaining bounded physical steering.
- Follow canonical publication remained active in the same replay (six emitted traces).
- Canonical command mutation and async identity reject counts remained zero.
- One 25.576 ms callback overrun occurred in the initial bag-ingestion interval about five seconds
  before the first Stop. No Stop interval reported an overrun.

## Root-cause conclusion

The issue was not insufficient Stop-QP tuning. SafetyBrake already had correct Stop semantics, but
the absence of an authority boundary allowed an unrelated normal solver to execute before the
emergency override. The correction removes that owner rather than adding another formulation.

## Remaining concern

There is still no legitimate nominal Hold producer. DynamicWait retains ShiftOut/Pass provenance,
and this Slice deliberately does not relabel or promote it. A future nominal Hold requires a typed
zero-progress producer and its own shadow/dynamic evidence.
