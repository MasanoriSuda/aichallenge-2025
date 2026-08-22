# Validation

## Baseline reproduction

At baseline `c60f9ce`, the controller creates and stores a canonical plan but never calls
`resolve_execution_cursor()`, `build_canonical_normal_candidate()` or
`resolve_canonical_normal_authority()` for that plan. Therefore `status=certified` does not yet
prove that the canonical authority contract would accept the same runtime result.

## Runtime acceptance signal

The periodic shadow line must report cursor, candidate and fresh-authority acceptance counts. A
`status=certified` result is valid only after all three accept the same plan/decision/window.

## Implemented proof chain

- The accepted store snapshot is resolved at the current cycle timestamp.
- The exact cursor plan ID, first stage and remaining stage count are bound to the current decision's
  physical certificate.
- Candidate construction must accept that exact execution window.
- The production canonical selector is evaluated with the candidate in its fresh slot.
- Any failure returns an explicit `canonical-*-reject`; only `FreshCertified` reaches
  `status=certified`.
- The plan's `control_stages` remain unread by `mpc_controller_cpp.cpp`; the result is still shadow
  telemetry and cannot publish a command.

## Verification

- Fresh candidate/selector focused contract: passed.
- `make autoware-build`: 25 packages passed.
- Canonical focused CTest: 2/2 passed.
- Complete package CTest: 35/35 passed.
- `colcon test-result --verbose`: 1596 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: passed.

The pre-existing stale `build/joycon_contract_guard/package.xml` discovery warning remains
non-failing. No AWSIM container was running, so dynamic admission coverage remains for the next
user-started trial.
