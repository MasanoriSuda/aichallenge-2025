# Tasklist

- [x] Join the first accepted ShiftOut episode by timestamp and identity.
- [x] Trace DP source creation, refresh, lease and release owners in source.
- [x] Trace six-state fresh/retained rejection reasons after admission.
- [x] Classify callback overrun ordering and cost centers.
- [x] State the earliest root cause and rule out competing hypotheses.
- [x] Add a failure-first test for the confirmed defect.
- [x] Implement one root-cause correction without tuning/fallback.
- [x] Run build and full package tests.
- [x] Run bounded `make dev2` and compare the same causal chain.
- [x] Update evidence/spec/migration map and commit.

## Closure

- Root defect fixed: six-state certified execution trajectory had no rolling
  ShiftOut source consumer.
- Dynamic acceptance: `output/20260825-233538` promoted exact six-state
  sources with the same target, generation, side and ShiftOut intent.
- Independent debt: stage-zero virtual-progress input row 254 becomes
  continuously unsolved during some ShiftOut intervals.  It is reproduced in
  both old and new runs and moves to a separate formulation Slice.
