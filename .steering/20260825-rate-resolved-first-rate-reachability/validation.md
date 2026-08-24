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
- 1,830 tests/assertions;
- 0 errors, 0 failures, 0 skipped;
- exact semantic-boundary, solver-residual, empty-interior and immutable
  tolerance tests pass;
- all 24 single-authority source-contract tests pass.

The stale `joycon_contract_guard/package.xml` parser warning occurs after the
selected package tests and is not a test failure.

## Static gates

- `git diff --check`: PASS.
- strict publication sampling remains unchanged.
- first-stage reachability is represented in the QP input row rather than an
  output clamp.
- no parameter, solver setting, fallback or authority change exists.

## Dynamic gate

Pending committed-source `make dev2` evidence.
