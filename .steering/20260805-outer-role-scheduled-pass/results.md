# Results

## Static verification

- `make autoware-build`: passed; 25 packages completed.
- Focused `multi_purpose_mpc_ros` test run: 851 tests, 0 errors, 0 failures.
- `git diff --check`: passed.
- `aichallenge/result-summary.json` remains an unrelated user-owned change.

`colcon test-result --verbose` reported a pre-existing missing
`build/joycon_contract_guard/package.xml` while scanning old build metadata, but
the command completed successfully and the selected package had no test
failures.

## Dynamic check

Run `make dev2` without changing parameters first. In the new Domain 1 log,
check this sequence:

1. `PassPlan frozen` reports `outer_transition=1` with a finite transition
   window when the initial outside role reverses before rear-clear.
2. `scheduled outer transition accepted` appears before the window deadline.
3. The same mission reaches `Pass -> Return -> Idle` without
   `inner_pass=1`, SafeSeparation, wall Recovery, or solver failure.

If the transition is rejected, the INFO line now includes the exact atomic
preflight reason. Do not relax wall or acceleration limits until that reason is
confirmed.

## Acceptance target

- At least one scheduled role handoff is accepted in the previously failing
  hairpin sequence.
- No transition window expires without either a successful handoff or a clear
  rejection reason.
- Clean overtake completion count increases from the prior `0 / 7` baseline.
