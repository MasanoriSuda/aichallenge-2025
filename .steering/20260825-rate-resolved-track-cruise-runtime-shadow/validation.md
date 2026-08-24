# Validation

## Static gates

### Build

Command:

```bash
make autoware-build
```

Result: PASS. 25 packages completed. The only stderr was the existing Python
`setup.py install` deprecation warning.

### Package tests

Command (inside the repository Docker overlay):

```bash
cd /aichallenge/workspace
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --verbose
```

Result: PASS.

- 44 CTest targets passed;
- 1,827 tests/assertions reported;
- 0 errors, 0 failures, 0 skipped;
- the four new shadow solver/mailbox tests passed;
- the 24 single-authority source-contract tests passed.

The reported `joycon_contract_guard/package.xml` parser warning is an existing
stale result-artifact warning after all selected package tests passed; it is not
a test failure.

### Source and formatting gates

- `git diff --check`: PASS.
- Track/Cruise eligibility/wall-contract audit: PASS.
- Production-authority reachability audit: PASS; the result is consumed only
  by observation telemetry and is never a selector candidate.

## Dynamic gate

Pending committed-source `make dev2` evidence. The run must demonstrate:

- nonzero Track/Cruise shadow submissions and consumed results;
- typed solved/rejected outcomes with valid identity;
- zero mailbox rollback/unsubmitted/invalid-result events;
- bounded solve/result age without a new 40 Hz callback-overrun tail;
- `authority=shadow, selected=0` throughout;
- no production behavior or configuration change.
