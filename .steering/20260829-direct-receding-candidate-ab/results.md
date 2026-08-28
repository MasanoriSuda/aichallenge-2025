# Results

## Static verification

- `make autoware-build`: 25 packages finished successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 54 test targets,
  2104 tests, zero errors, failures or skips.
- `colcon test-result --verbose` also reported an unrelated stale
  `build/joycon_contract_guard/package.xml` lookup warning; it did not change
  the package test summary.

## Authority invariant

`DirectRecedingCandidateObservation` resolves cursor zero for physical A/B
evidence, but `mpcc_rate_resolved_production_adapter` rejects it with
`AuthorityRejected`. Existing TimeAligned/Published production behavior is
unchanged.

## Dynamic result

Bounded run: `output/20260829-065458`, Domain 1.

- While moving in Cruise/Follow/ShiftOut, the existing time-aligned A path was
  accepted far more often than direct cursor-zero B. Representative windows
  were A `49/50`, B `0/50`, with roughly `70 ms` average result age and
  `0.351 m` latest-state pose residual; and A `20/22`, B `1/22`, with roughly
  `123 ms` average result age and `0.436 m` pose residual.
- B became acceptable mainly at zero or very low speed. That does not support
  moving production adoption.
- A direct cursor-zero interpretation of the existing asynchronous artifact is
  therefore rejected. A public receding-MPCC architecture would have to solve
  from a state predicted for publication/feedback time, not merely reset the
  cursor on an old prediction.
- The actual ShiftOut loss occurred upstream: new current-world direct-side
  populations became solver-rejected (`seq713`, `seq788`), so the last
  certified artifact exhausted naturally.

## Deletion

The observation clock, production rejection branch, cache, telemetry and
tests were removed after classification. No second clock or authority remains
in production code.
