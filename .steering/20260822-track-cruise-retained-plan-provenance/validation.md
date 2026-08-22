# Validation

## Failure-first case

Given current decision 42, a retained problem solved at decision 41 with an otherwise certified and
unexpired solution must not be selected until its remaining execution prefix has been physically
revalidated from decision 42's current pose.

The baseline selector does not yet represent that proof and therefore accepts the stale retained
candidate. The first test must demonstrate this incorrect acceptance before implementation.

Observed baseline result:

- focused suite: 33 passed, 1 failed;
- failing test: `CanonicalNormalAuthorityRejectsRetainedWithoutCurrentExecutionProof`;
- actual source: `RetainedCertified`;
- expected source: `EmergencyStop`.

This isolates the defect to the selector contract. No production controller or parameter was
changed to obtain the failure.

## Implemented invariant

`CanonicalNormalCandidate` now carries all three independent facts:

```text
problem.decision_id
execution_plan_id
execution_certificate_decision_id
```

A fresh candidate must have both its problem decision and execution certificate at the current
decision. A retained candidate keeps its older problem identity, but its real remaining control
plan must have a nonzero identity and a current-decision execution certificate. A solution object
plus a claimed stage count is insufficient.

Explicit reject reasons distinguish:

- missing executable plan identity;
- missing executable stages;
- malformed remaining horizon;
- stale current-pose execution certificate.

## Verification

- Focused `test_mpcc_execution_contract`: 36/36 passed.
- `make autoware-build`: 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 33/33 CTest targets passed.
- `colcon test-result --verbose`: 1,579 tests, zero errors, failures or skips.
- The existing stale `build/joycon_contract_guard/package.xml` result-parser warning remains; the
  command exited successfully and no package test failed.
- `git diff --check`: passed.

The selector remains runtime-disconnected. Therefore this Slice changes neither published commands
nor simulation behavior and does not authorize Track/Cruise promotion.
