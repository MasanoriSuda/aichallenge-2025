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

### First rebuilt run: `output/20260830-132516`

The run reached a physically valid Return preflight in episode 4, but exposed
one remaining same-cycle lifecycle defect:

- `Return authority deferred ... action=retain-certified-pass` was emitted;
- the prospective Return proposal was not complete yet;
- the same `TargetClearAhead` event then invalidated Mission generation 1;
- legacy DynamicMissionWait changed `Pass -> FollowPrepare` in that cycle.

The previous fallback comment explicitly permitted this fall-through.  This
was not physical infeasibility or parameter tuning: a second phase writer
defeated the atomic handoff and destroyed the identity needed by the in-flight
Return proposal.

The causal repair now distinguishes physical Return rejection from a
physically valid Return whose certified authority is pending.  The latter
keeps Pass as the sole tactical phase owner and cannot enter DynamicMissionWait
from that SafeSeparation decision.

### Second rebuilt run: `output/20260830-133656`

The repaired binary completed a two-vehicle dynamic run without reproducing
the same-cycle invalidation.  The available traffic episodes did not exercise
the Return condition:

- six ShiftOut episodes were observed;
- one reached Pass;
- that Pass was superseded by external Recovery before Return eligibility;
- the other episodes were dominated by actual wall-margin or live-corridor
  failures.

Consequently the exact positive acceptance sequence (`kind=return`,
`Pass -> Return`, `gate_a_joined=1`, no intervening canonical Stop) remains
**dynamically unobserved**, not failed.  Static verification after the causal
repair passed all 58 package test targets and all 84 single-authority source
contracts.  The next Return-eligible run must still provide the positive
acceptance sequence before this handoff is called fully dynamically accepted.
