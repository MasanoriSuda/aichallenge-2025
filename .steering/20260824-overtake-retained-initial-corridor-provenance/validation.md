# Validation

## Static

- `git diff --check`: pass.
- MPCC source contract: 16/16 pass.
- `make autoware-build`: 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: pass.
- `colcon test-result --verbose`: 1767 tests, 0 errors, 0 failures, 0 skipped.

The unrelated stale `build/joycon_contract_guard/package.xml` warning remains
outside this Slice.

## Dynamic

- Run: `output/20260824-135615`
- ShiftOut canonical production authority was exercised.
- Accepted and rejected time-zero proofs emitted the same typed operands used
  by admission.
- The first rejection was classified without changing any control, admission,
  solver, wall, timing, or phase behavior.

## Result

The observation Slice is complete.  The first authority break is attributable
to measured lateral tracking error exceeding a nominal retained corridor with
insufficient execution reserve, not to a malformed corridor or progress-frame
identity error.
