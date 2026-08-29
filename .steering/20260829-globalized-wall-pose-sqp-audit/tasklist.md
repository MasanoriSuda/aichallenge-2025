# Task list

- [x] Freeze the D-success snapshot and production boundary.
- [x] Add a joint lag/heading observation-only bucket arm.
- [x] Replay the first racing iterate through exact proof.
- [x] Add bounded proof-guided pose-globalized depths if required (not
  required: the first joint-pose iterate certified on the frozen source).
- [x] Replay accepted counterexamples to prevent regression.
- [x] Build, test, classify and register the result.
- [x] Decide whether a production replacement is justified.

## Verification

- `make autoware-build`: 25 packages passed.
- Focused CTest: 2,085 tests, zero failures.
- Six frozen ShiftOut wall failures: four certified bundles, one exact-proof
  rejection and one numerical rejection.
- Frozen dynamic counterexample `12107242968934788374`: all three relaxed
  arms were rejected by the unchanged timed dynamic-obstacle proof.
