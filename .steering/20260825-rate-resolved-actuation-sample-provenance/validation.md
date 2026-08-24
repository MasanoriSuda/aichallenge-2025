# Validation

## Build

Command:

```bash
make autoware-build
```

Result: PASS. All 25 packages completed. The only stderr was the existing
Python `setup.py install` deprecation warning.

## Package tests

Command inside the repository Docker overlay:

```bash
cd /aichallenge/workspace
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --verbose
```

Result: PASS.

- 44 CTest targets passed;
- 1,827 tests/assertions reported;
- 0 errors, 0 failures, 0 skipped;
- typed actuation-sample and shadow provenance tests passed;
- all 24 single-authority source-contract tests passed.

The reported stale `joycon_contract_guard/package.xml` parser warning occurs
after the selected package tests and does not represent a test failure.

## Source and formatting gates

- `git diff --check`: PASS.
- typed reason/value provenance is consumed only by shadow validation and
  aggregate telemetry.
- no configuration, tolerance, solver, fallback or authority branch changed.

## Dynamic gate

Pending `make dev2`. The run must classify every
`ActuationSampleRejected` result before any repair is selected.
