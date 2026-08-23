# Validation

## Static

- `make autoware-build`: 25 packages succeeded.
- `test_race_mpcc_foundation`: 31/31 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 1,733 tests,
  zero failures/errors/skips. The result scan also reported the known unrelated
  stale `joycon_contract_guard/package.xml` lookup.
- The new tests prove stage-level provenance for progress regression and
  negative velocity.
- `git diff --check`: clean.

## Dynamic

- `output/20260824-070520` is excluded because the build container was still
  compiling and the run loaded the prior install.
- `output/20260824-071238` used the rebuilt install.
- Exact-artifact incompleteness did not recur; the first ShiftOut exact proof
  was accepted and published by the existing converted production path.
- Canonical Overtake shadow produced repeated complete current-world plans,
  while the final trace remained `legacy-normal-bypass`.

## Gate

Accept the typed diagnostic only. Do not alter exact-artifact completeness
without a field-level recurrence. Proceed to the separately scoped canonical
Overtake production authority Slice.
