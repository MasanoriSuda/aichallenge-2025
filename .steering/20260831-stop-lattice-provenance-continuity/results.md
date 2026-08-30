# Results

## Failure-first proof

Before the implementation change, the focused test failed because the accepted Stop-lattice plan contained a null `solver_source_snapshot`.

```text
LiveStopShadowBuildsCertifiedObservation
Expected: solver_source_snapshot != nullptr
actual: nullptr
```

This reproduced the runtime `plan=1, artifact=1, source=0` failure without relying on timing.

## Implementation result

The publisher-boundary-rebased Stop candidate is materialized once as an immutable snapshot. That exact object is now used for:

- anytime lattice schedule generation;
- fixed-steering-rate private solve;
- exact trajectory adaptation;
- physical wall proof;
- dynamic-obstacle proof;
- certified-plan solver provenance.

No source-validation rule, authority owner, Store, lease, timeout, margin or solver setting changed.

## Static acceptance

- `make autoware-build`: 25 packages passed.
- focused provenance/mismatch/supersession tests: 3 passed.
- package test suite: 59/59 targets, 2188 tests, 0 errors/failures.
- source authority contract: 92 passed as part of the package suite.
- `git diff --check`: passed.

## Dynamic acceptance

Run: `output/20260831-002556/d1/autoware.log`

At decision 1344:

```text
Stop lattice current-world alternate: decision=1344, intent=shiftout,
source=1, normal=terminal-contingency-unavailable,
joined=1, reason=accepted

Stop lattice production bridge: decision=1344, intent=shiftout,
ordinary=terminal-contingency-unavailable, source=1,
join=accepted, authority=canonical-normal, selected=1
```

After selection:

- `Published Stop lattice source rejected`: 0 occurrences;
- the selected episode completed `ShiftOut -> Pass -> Return -> Idle`;
- mismatch artifacts at the ShiftOut-to-Pass boundary remained rejected;
- no actual wall-margin violation occurred in that completed episode.

This confirms that production publication no longer destroys the Stop-lattice provenance merely because the alternate was selected.

## Residual defect classification

The bridge did not remove every external Stop. At decision 1345, the retained source was approximately 0.84 s into its published clock. Its Stop trajectory expected about 1.99 m/s while the observed vehicle speed was about 4.91 m/s, so current-world revalidation failed. The published Stop successor also reported a static wall collision.

This is no longer `source=0` self-invalidation. It is a separate scheduling/timebase and Stop-suffix feasibility issue:

```text
selected provenance is complete
  -> artifact ages while asynchronous work continues
  -> current state diverges from the old maximum-braking trajectory
  -> current-world join or Stop suffix fails
  -> external Stop may still appear
```

The selected Stop-lattice evaluation itself cost about 33 ms in this run and caused a 40 ms callback. Later telemetry showed Stop-lattice worker evaluations ranging into the 100--160 ms class, with one longer superseded evaluation. This supports investigating scheduling and artifact age before changing geometry or configuration.

## Exit decision

The provenance Slice is accepted. The root cause it targeted is removed and dynamically verified. Remaining authority chatter belongs to a new scheduling/timebase Slice; the later wall feasibility question must remain separately observable.
