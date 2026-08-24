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

Pending committed-source `make dev2` evidence.
