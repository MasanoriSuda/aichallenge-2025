# Tasklist

- [x] Re-audit the migration ledger and identify the next authority blocker.
- [x] Separate the fresh-solve authority defect from solver-miss continuity.
- [x] Add the typed Overtake current-corridor observation and proof.
- [x] Add focused proof tests for acceptance and fail-closed mutations.
- [x] Add the immutable Overtake canonical plan store.
- [x] Evaluate retained Overtake authority in shadow on fresh misses.
- [x] Add aggregated fresh/retained rejection telemetry.
- [x] Run focused tests and `make autoware-build`.
- [x] Run package tests.
- [x] Replay a saved Overtake bag and update evidence.
- [x] Update the migration ledger.
- [x] Commit the accepted shadow Slice.

## Definition of Done

- No parameter changes or new fallback path.
- No production command mutation.
- Retained readiness requires current-world proof, not plan age.
- The replay can quantify how many fresh misses are covered by a retained
  same-formulation plan and why all other misses are rejected.
