# Validation

## Failure-first evidence

- Before strict-side repair,
  `RowToleranceNormalizationUsesStrictAsymmetricSide` failed with maximum row
  scale `250` instead of the contract value `1.0`.
- Before variable-coordinate implementation, the new
  `VariableCoordinateScaling` API did not compile.

## Static validation

- `make autoware-build`: 25 packages passed.
- `test_persistent_osqp`: 22/22 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 40/40 programs passed.
- `colcon test-result --verbose`: 1709 tests, 0 errors, 0 failures, 0 skipped.
  The command also reports the pre-existing missing
  `build/joycon_contract_guard/package.xml` artifact while still producing the
  successful aggregate.

## Dynamic validation

Accepted run: `output/20260824-031300`.

After 40 seconds of dev2 closed-loop execution:

| Domain | Track/Cruise | Certified | Constraint failure | Follow decisions | Follow selected | Follow constraint failure |
|---|---:|---:|---:|---:|---:|---:|
| d1 | 1 | 1 | 0 | 383 | 220 | 0 |
| d2 | 1 | 1 | 0 | 0 | 0 | 0 |

There were zero `stage=constraint_check` records and zero legacy extended
fallback records in both domains. This directly contrasts with
`output/20260824-022828` (34 D2 execution-primal rejects) and the intermediate
variable-scaled runs, which retained stage-0 constraint failures.

This is a numerical-contract acceptance run, not a six-lap performance claim.
