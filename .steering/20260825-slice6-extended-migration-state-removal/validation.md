# Validation

## Evidence boundary

- Branch: `develop_july`
- Baseline: `8183d6a docs(mpcc): record normal fallback deletion trial`
- Dynamic reference: `output/20260825-112734`
- User-owned `aichallenge/result-summary.json` is excluded.

## Pre-fix audit

Exact-use search found no controller call to:

- `ExtendedSolverCircuitBreaker::record_failure/record_success`;
- `ExtendedSolverReentryGate::record_failure/record_success`;
- `ExtendedModeHandoff::resolve_velocity`;
- `record_extended_mpcc_telemetry`;
- `record_rti_sqp_telemetry`.

The controller still read `ExtendedSolverCircuitBreaker::active()` in two DP
authority paths and reset all three migration objects. Because no reachable
producer can activate them, those reads are constant-false migration residue,
not a live safety certificate.

## Failure-first proof

- Added `test_retired_extended_formulation_switch_state_is_physically_deleted`.
- Before implementation it failed on the compiled circuit breaker, reentry
  gate, mode handoff, telemetry recorders and four migration-only YAML keys:
  `1 failed, 35 passed`.
- After physical deletion the same source-contract suite passes:
  `36 passed`.
- A final aggregate-contract audit then exposed the constant-false
  `FrenetDpExecutionAuthorityRequest::solver_degraded` input and its
  `SolverDegraded` rejection reason. Extending the same failure-first test
  reproduced the residue (`1 failed, 35 passed`); the field, reason, caller
  arguments and dedicated hard-fault test were physically deleted. The suite
  again passes `36 passed`.

## Static validation

- Exact-symbol search finds the retired types, controller members, functions
  and YAML keys only in the negative source-contract assertions; production
  source and configuration have no hit.
- The first production build exposed three debug-format references to the
  deleted `extended_mpcc_solver_degraded` local. They were display-only
  residue. The obsolete log column was deleted instead of restoring the
  migration state.
- `make autoware-build`: 25 packages passed.
- Package rebuilt with `BUILD_TESTING=ON`.
- `colcon test --packages-select multi_purpose_mpc_ros`: passed.
- `colcon test-result --verbose --test-result-base build/multi_purpose_mpc_ros`:
  1,868 tests, zero errors, failures or skips.

## Dynamic validation

This Slice removes inert state and does not change a live normal owner. A new
trial is required only if static/source-contract review finds that a deleted
symbol was still reachable through production.

The accepted committed-source structural run `output/20260825-112734`
already demonstrates that normal publication uses only six-state
Track/Cruise and five-state Follow identities, with no legacy/three-state
normal solve or cross-formulation fallback. No new dynamic trial is required
for this behavior-neutral deletion.
