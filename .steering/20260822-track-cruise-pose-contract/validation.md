# Validation

## Failure-first evidence

Two focused tests were added before implementation:

- `ExtractsFiveStatePoseWithoutReconstructingHeading`
- `RejectsNonfiniteFiveStatePoseWithStageProvenance`

The focused build first failed because `extract_extended_execution_trajectory` did not exist. The
first implementation then exposed a second contract defect: a whole-vector `allFinite()` rejection
lost the failing stage. Per-stage validation replaced it and the focused suite passed 52/52.

## Static validation

- `make autoware-build`: passed, 25 packages.
- Full `multi_purpose_mpc_ros` package tests: passed, 33/33.
- Focused `test_mpcc_progress`: passed, 52/52.
- Compiler warnings introduced by aggregate initializers were removed by explicitly preserving the
  empty-heading legacy path.
- `git diff --check`: passed.

## Dynamic validation

Baseline: `output/20260822-135649`, commit `1634c40`, four waypoint wraps.

Candidate: `output/20260822-142549`, four waypoint wraps, exact five-state heading supplied only to
the Track/Cruise shadow physical certificate.

| Metric | Baseline | Candidate |
|---|---:|---:|
| Eligible / physical checks | 7,579 | 7,505 |
| Certified | 7,491 (98.8389%) | 7,427 (98.9607%) |
| Hard-contact rejects | 67 | 62 |
| Swept-path rejects | 21 | 16 |
| Invalid/bound/heading/sample rejects | 0 | 0 |
| Pose-trajectory extraction rejects | N/A | 0 |
| Actuation joins rejected | 0 | 0 |
| Callback weighted average | 6.1631 ms | 6.1051 ms |
| Callback maximum | 29.364 ms | 30.061 ms |
| Callback overruns | 1 | 4 |
| `selected=1` | 0 | 0 |

All candidate outputs remained `authority=shadow, selected=0`; production continued to report
`legacy-normal-bypass`. There was no production authority or configuration change.

## Causal conclusion

Preserving solved `e_psi` removes an information-loss defect and improves physical acceptance by
0.1218 percentage points (about 10.5% fewer rejects after normalizing by physical checks). It does
not eliminate the defect: positive-reserve hard contacts and swept violations remain.

Therefore reconstructed heading was a contributing error, not the root cause of the remaining wall
certificate failures. The remaining upstream contract gap is that scalar Frenet center bounds do
not guarantee clearance for the solved yawed 2.0 m x 1.45 m footprint. The next slice should derive
heading-aware footprint-safe stage bounds for the shadow problem and separately account for the raw
current-pose-to-first-stage swept connection. The physical certificate must remain unchanged as the
acceptance oracle, and production authority must not be promoted yet.
