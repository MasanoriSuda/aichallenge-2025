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
Committed-source revalidation is pending.
