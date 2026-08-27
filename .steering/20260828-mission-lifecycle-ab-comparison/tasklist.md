# Mission lifecycle A/B comparison tasklist

- [x] Freeze requirements and controlled variables.
- [x] Identify the common isolated seven-state branch evaluator.
- [x] Add a failing unit test for strict A/B candidate isolation.
- [x] Implement bounded observation-only A/B evaluation.
- [x] Verify comparison output cannot affect production authority.
- [x] Build and run focused tests.
- [x] Run `make dev2` and capture the absence of a comparable fresh B.
- [x] Classify the result and update the experiment registry.
- [x] Record remaining B/C/D work.
- [x] Remove temporary live instrumentation and unused audit API after capture.

## Closure

The comparison was blocked before solver execution because the fresh receding
candidate is not an independent producer during active ShiftOut/Pass.  See
`results.md`.  IM-1 must create a replay-ready immutable snapshot before the
independent InteractionBundle B is implemented.
