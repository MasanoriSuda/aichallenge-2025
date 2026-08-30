# Results

## Static verification

- `make autoware-build`: passed, 25 packages.
- Focused tests:
  - `test_race_mpcc_foundation`: 34/34 passed;
  - `test_single_authority_source_contract`: 84/84 passed.
- Full `multi_purpose_mpc_ros` CTest: 58/58 targets passed,
  2194 tests, zero errors and zero failures.

## Implemented causal repair

- A valid geometric Return reference now requests a prospective Return solve
  without mutating the live Pass phase.
- The existing bounded latest-only causal worker solves the private Return
  snapshot through the canonical seven-state, wall, dynamic, terminal and
  contingency proof pipeline.
- Consumption requires current-world target provenance and exact
  target/Mission-generation/side/Return identity.
- `Pass -> Return` remains deferred while that proposal is missing or
  rejected; the current certified Pass authority remains live.
- Canonical atomic admission consumes the same proposal in the transition
  cycle. No new command publisher or fallback was added.

## Dynamic acceptance

Pending a commit-provenance run. Acceptance requires a visible
`kind=return` causal result, `Pass -> Return`, `gate_a_joined=1`, and no
intervening canonical Stop.

