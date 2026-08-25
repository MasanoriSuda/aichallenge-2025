# Validation

## Static verification

- `make autoware-build`: passed, 25 packages.
- `test_single_authority_source_contract.py`: 51 passed after the DTO-copy
  guard and final telemetry contract.
- `colcon test --packages-select multi_purpose_mpc_ros`: all 49 CTest targets
  passed after the final source change.
- `colcon test-result --verbose`: 1866 tests, zero errors and zero failures.
  The command also reports the pre-existing stale
  `build/joycon_contract_guard/package.xml` artifact warning.
- `git diff --check`: passed before commit.

## Dynamic verification

Bounded `make dev2` evidence:

- `output/20260825-191126`: immutable six-state artifacts were produced, but
  the live selection stayed default-invalid.
- `output/20260825-191856`: side eligibility confirmed the mismatch was between
  a complete worker artifact and an absent live selection.
- `output/20260825-192536`: after repairing the worker-to-live DTO copy, valid
  six-state selections appeared in the live telemetry.

The final run contained eight throttled pre-entry comparison records: three
complete/selected, five solver-infeasible, one valid five/six agreement, and two
cases where six-state selected a complete side while five-state selected none.
Every record remained `authority=shadow,selected=0`; production Mission and
normal command authority were unchanged.

## Remaining promotion gates

- No dynamic `Pass` prospective artifact was observed.
- The selected six-state CertifiedPlan still needs current-world revalidation
  at the live Gate-A adoption point.
- Five-state Gate A must be physically deleted in the same Slice that promotes
  the six-state proof; it must not remain as a permanent fallback.
