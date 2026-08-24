# Validation

## Static and package validation

- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q
  aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/test/test_single_authority_source_contract.py`
  passed: 13 tests.
- `make autoware-build` passed: 25 packages.
- `colcon test --packages-select multi_purpose_mpc_ros` passed: 40/40
  package tests.
- `colcon test-result --verbose` reported 1,758 tests, zero errors and zero
  failures. It also printed a pre-existing stale-build metadata warning for
  `build/joycon_contract_guard/package.xml`; no test result failed.
- `git diff --check` passed.

## Dynamic Gates

- First run: `output/20260824-122401/d1/autoware.log`.
- Exact final rebuilt source: `output/20260824-123452/d1/autoware.log`.
- Final source entry: line 629, decision 2384 follows at line 637.

The first accepted Overtake entry carried generation 1, side -1, a complete
20-stage five-state artifact and the unchanged 0.40 m production wall
contract. Its first published ShiftOut command was:

```text
solver=canonical-shiftout-retained
output=canonical-shiftout-retained-published
authority=certified-normal-solution
acceleration=+1.37 m/s2
```

In the first run, ShiftOut produced three canonical retained normal
publications and no legacy normal handoff. In the rebuilt-source run, the
first ShiftOut again produced canonical retained normal authority. Across the
accepted boundary in both runs there were:

- 0 `legacy-normal-bypass` publications;
- 0 `dynamic-escape-exit-wall-hold` publications;
- 0 `current origin rejected: discontinuous` outcomes while Overtake owned
  ShiftOut.

The retirement trace count was zero because no old DynamicEscape executable
artifact was present at the accepted boundary in this run. Deterministic
source and policy tests cover unconditional retirement ordering, including
the case where a wall gate is already active.

## Gate result

Accepted for the scoped root cause and reproduced with the exact final
binary. The initial downstream legacy wall owner and its -3.0 m/s2 command
mutation did not recur. Recovery later used legacy Rejoin authority, but
Rejoin is explicitly outside this Slice.

The runs exposed the next independent defect after entry: current-world
Overtake proof can reject `initial-corridor-violation` or
`optimized horizon failed physical revalidation`, then enter
DynamicWait/Recovery. In the rebuilt-source run, 14 later
`current origin rejected: discontinuous` strings were all emitted only after
Overtake had left ShiftOut, by Follow/Track/Cruise retained-plan handling
during the resulting stop/intent transition. This is not a reason to restore
the deleted normal owner or loosen the progress/wall contracts. The next
Slice must trace the first Overtake physical-revalidation failure and the
cross-intent retained-plan lifecycle separately.

## Final-rebuild note

The implementation audit found and fixed a short-circuit ordering bug in the
retirement call after the first package run: executable artifacts are now
retired before the boolean state summary is formed. The source-contract test
pins this order. Final build/package tests and the second dynamic Gate used
that exact source revision.
