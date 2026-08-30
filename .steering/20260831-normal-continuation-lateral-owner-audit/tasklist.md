# Task list

- [x] Freeze first D3 retained continuation failure.
- [x] Confirm generic A/B/C/D target-free Cruise coverage gap.
- [x] Replay the recorded QP warm and cold.
- [x] Run prepared-suffix comparisons at publication-relevant times.
- [x] Classify the earliest causal owner.
- [x] Add only missing observation support if classification is impossible.
- [x] Add deterministic regression tests for the classified invariant.
- [x] Update canonical integration documentation.
- [x] Build and run the full test suite.
- [x] Commit only Slice-owned files.

## Definition of Done

- The failure has one evidence-backed owner classification.
- No parameter or production-authority change is made without that evidence.
- Any audit-platform change remains observation-only.

## Classification

- Recorded wall-refined QP: linearly infeasible.
- First-stage physical lateral upper: `0.209006 m`.
- First-stage dynamics/input reachable minimum: `0.235208 m`.
- Broad-solve and rejected-iterate controls: accepted after canonical
  nonlinear reconstruction and exact wall/dynamic/Stop proof.
- Owner: physical wall bucket model/certificate mismatch.
- Production authority changes: none in this Slice.

## Verification

- `make autoware-build`: 25 packages succeeded.
- `ctest --output-on-failure`: 59/59 passed.
- Architecture comparison target: 29/29 passed.
