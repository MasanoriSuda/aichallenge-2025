# Results

## Static verification

- `make autoware-build`: passed, 25 packages.
- focused retained-revalidation tests: 57/57 passed.
- focused physical-adapter tests: 22/22 passed.
- single-authority source-contract tests: 89/89 passed.
- `colcon test --packages-select multi_purpose_mpc_ros`: 59/59 test
  targets passed.
- `colcon test-result --verbose`: 2254 tests, 0 errors, 0 failures,
  0 skipped.
- The unrelated pre-existing `joycon_contract_guard/package.xml` diagnostic
  remains outside this Slice.

## Dynamic verification

- Command: `make dev2`
- Evidence: `output/20260830-183143/d1/autoware.log`
- The run exercised three `Idle -> ShiftOut` entries, two
  `ShiftOut -> Pass` transitions, one complete
  `Pass -> Return -> Idle` chain, and later Recovery cases.

The new production edge was exercised repeatedly.  Representative evidence:

```text
decision=2670, intent=pass, source=1986,
bundle=available/2037, joined=1, join_reason=accepted,
authority=canonical-normal
```

The same reified artifact was subsequently published and retained as sequence
2037, and later output reported
`canonical-rate-resolved-pass-published-bundle-reproved`.  Additional accepted
current-world successors were observed for ShiftOut, including decisions
3100, 3111 and 3148.  Across the full stopped run, 45 accepted bundles crossed
the canonical normal boundary.

Rejected reifications remained fail-closed.  The observed rejection was
`invalid-actuation-sequence`; its examples also failed the current-world join
through `continuation-rejected`, `steering-unreachable`, or wall evidence and
therefore retained `authority=external-emergency`.  No direct Stop command or
secondary normal publisher was introduced.

## Interpretation

The production defect in this Slice is resolved: when ordinary retained
authority is unavailable but a same-intent current-world Stop is reifiable and
revalidates, the exact artifact now becomes canonical normal authority and is
recorded only after publication.  A following cycle may still reject a newly
observed Stop because its actuation sequence, steering reachability, or wall
proof is no longer valid; those are explicit physical/current-world rejection
paths rather than loss of the reified artifact identity.

The run still contains pre-existing race-quality failures, notably a Pass
progress stall and a later static-wall physical-footprint Recovery.  They are
not hidden by this Slice and remain separate root-cause work.  No parameter,
clearance, solver tolerance, lease, grace, timeout, Mission resume rule, or
fallback was changed.
