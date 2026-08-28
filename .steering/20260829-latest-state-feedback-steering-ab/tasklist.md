# Task list

- [x] Freeze the asynchronous connector evidence.
- [x] Add a pure bounded steering feedback contract.
- [x] Add unit tests for unchanged, projected and invalid envelopes.
- [x] Replay projected steering through the existing nonlinear continuation.
- [x] Add aggregate observation-only telemetry and authority deletion gates.
- [x] Run build and full package tests.
- [x] Run bounded `make dev2` and classify feedback outcomes.
- [x] Decide whether to extend the feedback artifact through current wall and
      dynamic-obstacle proof.

Decision: extend the corrected trajectory through the unchanged current wall,
timed dynamic-obstacle and successor proofs in a separate observation-only
Slice.  Do not promote the steering projection by itself.
