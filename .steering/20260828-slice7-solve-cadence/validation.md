# Validation

## Baseline

`output/20260828-044759`, `N=20`, solve submission at 40 Hz:

- complete `ShiftOut -> Pass -> Return -> Idle` observed;
- Overtake Recovery: 0;
- callback overruns: 102/5713 (1.785%);
- mean/max MPCC callback window: 3.810/56.310 ms;
- worker compute periodically rose above 30 ms and replaced pending jobs.

## Static candidate Gate

- `make autoware-build`: passed, 25 packages.
- `test_latest_only_worker`: 18/18 passed, including five cadence cases.
- `test_single_authority_source_contract`: 66/66 passed.

## Dynamic candidate run

Run: `output/20260828-052038`, `N=20`, full solve at 20 Hz, publication and
current-world retained validation at 40 Hz.

The scheduler itself operated as designed.  Typical two-second windows changed
from about 80 submissions to 29--36 submissions plus 44--52 explicitly deferred
cycles.  Stable Track/Cruise windows had no authority loss and lower callback
cost.

The overtake/rejoin integration did not preserve authority continuously:

```text
episode 1: ShiftOut -> FollowPrepare -> Idle
episode 2: ShiftOut -> FollowPrepare -> Recovery -> Idle
episode 3: ShiftOut -> FollowPrepare -> Recovery -> Idle
```

No episode reached Pass.  Episode 2 failed with `static wall clamp exceeds
lateral acceleration limit`; episode 3 failed with `static wall clearance
margin infeasible`.  More importantly, Recovery/Rejoin alternated between a
certified command and:

```text
canonical-rejoin-emergency/
rate-resolved normal admission unavailable/
rejoin_reason=live-progress-already-active
```

That authority hole occurred 40 times in the bounded run.  Callback maxima also
still reached 61.687 ms, so the candidate did not remove the relevant tail.

## Decision

Rejected after the first independent run; a second run cannot rescue a hard
authority-continuity violation.  The experiment proved that the current
published artifact is not a sufficient 50 ms bridge across all Rejoin and
changing-wall states.  The complete cadence implementation, tests, config and
telemetry were removed rather than retained as a disabled production branch.

The accepted baseline remains `N=20` with production submission on every 40 Hz
callback.  This also closes cadence as a Slice 7 tuning lever under the current
artifact lifecycle: revisiting it requires a new certified multi-cycle suffix
contract, which is an architecture change rather than parameter tuning.
