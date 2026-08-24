# Task list

- [x] Preserve the failing dynamic evidence and isolate the boundary.
- [x] Add typed actuation-sample evaluation without behavior change.
- [x] Carry reason/value provenance through shadow telemetry.
- [x] Add deterministic reason tests.
- [x] Run build, full package tests and authority audit.
- [ ] Commit the diagnostic Slice.
- [ ] Run `make dev2` and classify all sample rejects.
- [ ] Record the next root-cause decision.

## Definition of Done

- Every sample reject has one stable typed reason.
- Accepted/rejected behavior is identical to the previous optional validator.
- No new authority, fallback, clamp, parameter or configuration branch exists.
- Dynamic evidence identifies the earliest violated invariant.
