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
- 1,828 tests/assertions;
- 0 errors, 0 failures, 0 skipped;
- the new certified QP sample test covers accepted solver-boundary rate,
  invalid certificate, publication-after-stage and sampled physical violation;
- all 24 single-authority source-contract tests pass.

The stale `joycon_contract_guard/package.xml` parser warning occurs after the
selected package tests and is not a test failure.

## Static gates

- `git diff --check`: PASS.
- strict standalone sampling behavior remains unchanged.
- no clamp, solver setting, parameter, fallback or authority change exists.

## Dynamic gate

Pending `make dev2`. Expected result: initial/rate/terminal duplicate rejects
fall to zero while genuine publication-time or sampled-output rejects remain
typed.
