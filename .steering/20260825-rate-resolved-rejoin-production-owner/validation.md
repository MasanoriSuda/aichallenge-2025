# Validation

## Root-cause result

Rejoin was not missing a distinct control model. `OvertakeLinePhase::Recovery`
already supplied its recovery-line reference, bounds and velocity horizon to
the canonical semantic problem. The defect was an authority split: Rejoin
alone bypassed the shared steering-rate-resolved producer and owned a private
five-state solver, plan store, evaluator and publisher lifecycle.

The first dynamic run also exposed a second upstream identity defect. The
shared request builder copied the previous overtake target into every intent,
including targetless Rejoin. Since Rejoin correctly had no current target
generation, this manufactured an incomplete identity and rejected the
six-state request. The target provenance is now assembled only for intents
whose semantics require a target. This also prevents Track and Cruise from
borrowing stale overtake identity.

## Failure-first evidence

- Rejoin was initially absent from the shared six-state intent/scope contract.
- Source-contract tests initially found the private five-state Rejoin solver,
  store, evaluator, telemetry and explicit dispatch.
- A dedicated targetless-intent test initially reproduced stale target
  provenance in Track, Cruise and Rejoin.
- After the changes, all 48 directly invoked source-contract checks pass and
  the old Rejoin owner is absent from the source contract.

## Static validation

- `make autoware-build`: passed, 25 packages.
- Full rebuilt package test suite: passed, 49/49 test targets.
- Source-contract test functions: passed, 48/48.
- No parameter, solver, horizon, wall-margin, timeout or ROS-interface change.

## Dynamic validation

The first bounded run, `output/20260825-174422`, intentionally failed closed:

```text
intent=rejoin
reason=rate-resolved source context incomplete
```

This was the evidence that led to the stale target-provenance fix; no fallback
or exception was added.

The corrected bounded `make dev2` run is `output/20260825-175208`. Domain 1
exercised Rejoin and recorded:

```text
Rate-resolved canonical atomic admission:
  previous=follow, intent=rejoin, attempted=1, certified=1,
  solver=solved, physical=accepted, elapsed_ms=2.426

MPCC execution contract:
  authority=certified-normal-solution,
  intent=rejoin,
  formulation=velocity-steering-progress-6state,
  canonical_source=retained-certified,
  retained=1,
  identity=complete
```

In that run:

- Rejoin six-state accepted publication: observed.
- `Rate-resolved normal submission unavailable: intent=rejoin`: 0.
- unresolved Rejoin execution contract: 0.
- five-state Rejoin production publication: 0; its producer was physically
  deleted.

The run also contained control-callback overruns, including intervals in which
the average callback exceeded the 25 ms period. They are not a Rejoin
formulation fallback and are deliberately recorded as separate real-time
quality work rather than hidden by this authority Slice.

## Remaining audit boundary

`VelocityProgress5State` still has source representations outside the deleted
Rejoin production lifecycle. They must be classified by reachability and
responsibility before Slice 6 closes. This Slice does not claim that every
five-state planning or exceptional-context representation has already been
removed.
