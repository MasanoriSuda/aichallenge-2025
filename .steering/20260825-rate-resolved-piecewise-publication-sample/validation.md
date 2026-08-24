# Validation

## Build

```bash
make autoware-build
```

PASS: 25 packages completed. The only stderr was the existing Python
`setup.py install` deprecation warning.

## Package tests

```bash
cd /aichallenge/workspace
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --verbose
```

PASS:

- 44 CTest targets;
- 1,832 tests/assertions;
- 0 errors, 0 failures, 0 skipped;
- exact first-boundary and second-stage partial samples pass;
- malformed sequence, whole-horizon overrun and actual integrated steering
  violations fail closed with typed reasons;
- all 24 single-authority source-contract tests pass.

The stale `joycon_contract_guard/package.xml` parser warning occurs after the
selected package tests and is not a test failure.

## Static gates

- `git diff --check`: PASS.
- no old certified single-stage call or type remains.
- strict standalone sample behavior remains unchanged.
- no clamp, stage timing, solver setting, parameter, fallback or authority
  change exists.

## Dynamic gate

### Preliminary run

Committed source `172e4d2` was exercised in `output/20260825-014314`.
All 5,844 consumed results solved and supplied a sample; every sample-rejection
reason was zero. However, the aggregate retained only the final sample-stage
index in each 2-second window. It could not prove that the transient stage-zero
crossing seen in the preceding run actually exercised stage one.

This run is therefore not the final Gate. Aggregate cross-stage count and
maximum-stage telemetry were added without changing calculation or authority.

### Final run

Committed source `02c7ee5` was exercised with `make dev2`.

Evidence: `output/20260825-015302`.

| Metric | Domain 1 | Domain 2 | Combined |
|---|---:|---:|---:|
| submitted | 451 | 5,840 | 6,291 |
| consumed | 408 | 5,756 | 6,164 |
| solved and publishable | 408 | 5,755 | 6,163 |
| sample rejected | 0 | 0 | 0 |
| cross-stage samples | 0 | 11 | 11 |
| maximum sampled stage | 0 | 1 | 1 |
| solve rejected | 0 | 1 | 1 |
| build / assembly / nonfinite / exception | 0 | 0 | 0 |
| maximum compute time | 3.565 ms | 12.371 ms | 12.371 ms |
| maximum solve time | 3.496 ms | 12.295 ms | 12.295 ms |

All 6,163 solved QPs supplied a physically valid publication sample. The 11
stage-zero crossings were explicitly sampled from stage one rather than being
discarded, and all sample-rejection categories remained zero. All 80 aggregate
records retained `authority=shadow, selected=0`; mailbox invalid, rollback and
unsubmitted counts were zero.

One independent `SolveRejected` result occurred among 6,164 consumed results.
The current 2-second aggregate retained only the last result, which was solved,
so its existing typed `Result::detail` was not preserved in the log. This does
not invalidate the cross-stage sampling proof, but it blocks production
authority promotion and defines the next failure-first diagnostic Slice.

## Decision

Accept the piecewise publication Slice. The QP-to-publication sample boundary
is complete for every solved result observed in this Gate, including actual
cross-stage cases. Do not promote authority yet. First preserve and classify
the isolated solver reject without adding retry, fallback or solver tuning.
