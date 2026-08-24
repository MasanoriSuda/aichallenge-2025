# Task list

- [x] Preserve the one-reject runtime evidence.
- [x] Trace where typed solver detail is lost.
- [x] Retain a separate failure result per telemetry window.
- [x] Emit immutable failure identity and detail.
- [x] Add/extend observation-only source contract.
- [x] Run build and full package tests.
- [x] Commit the diagnostic Slice.
- [x] Run `make dev2` until exercised or record NOT EXERCISED.
- [x] Classify the failure before any behavioral change (`NOT EXERCISED` after
  14,082 solved results; no behavioral change authorized).

## Definition of Done

- A later solved result cannot erase a failure in the same telemetry window.
- The log explains persistent-OSQP stage/status/row detail when exercised.
- No normal command or authority behavior changes.
