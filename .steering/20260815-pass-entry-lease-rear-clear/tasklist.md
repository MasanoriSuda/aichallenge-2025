# Tasklist

- [x] Inspect the latest run and exact failure transitions.
- [x] Define bounded early-Pass gate eligibility.
- [x] Add Pass-entry lease configuration and controller integration.
- [x] Preserve SafeSeparation lifecycle across nested holds.
- [x] Retain healthy DynamicMissionWait until rear-clear or total budget.
- [x] Add/update deterministic core tests.
- [x] Synchronize local/cloud configuration.
- [x] Run build, tests and diff checks.
- [x] Record verification results.

## Verification

- `make autoware-build`
  - Passed: 25 packages built.
  - Existing setuptools deprecation warnings only.
- `colcon test --packages-select multi_purpose_mpc_ros`
  - Passed: 25/25 test targets, 1139 tests, zero failures.
  - `colcon test-result --verbose` also reported one pre-existing missing
    `build/joycon_contract_guard/package.xml` diagnostic while returning zero;
    the selected package test result itself was clean.
- Focused gate/pause tests
  - Passed: 4/4.
- `git diff --check`
  - Passed.
- Local/cloud Pass-entry lease block
  - Identical (`2.5 s`, `12.0 m`).

## Dynamic acceptance checks

- Startup log reports `pass_entry_gate=enabled lease=2.50 s/12.00 m`.
- Wall warnings in the first 2.5 seconds / 12 metres of Pass produce a gate
  hold/release or bounded reselect instead of immediate physical Pass failure.
- `SafeSeparation entered` appears once per continuous episode, not at 40 Hz.
- DynamicMissionWait expiry before rear-clear logs one retained-wait message and
  does not transition to Idle; rear-clear or the total Mission budget terminates
  it.
