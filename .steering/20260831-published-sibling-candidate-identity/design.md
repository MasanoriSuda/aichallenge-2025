# Design: published sibling candidate identity

## Failure classification

This is a lifecycle/authority defect, not physical infeasibility.

The exact side `+1` sibling Bundle was current-world certified and crossed the
publisher.  A later direct retained evaluation selected a candidate captured
before that publication with side `-1`.  Current-world geometric proof accepted
the trajectory, but that proof does not grant tactical identity.  The candidate
therefore crossed the publisher without a cross-side adoption token and
overwrote the published source ledger.

## First failed invariant

After tactical side mutation, the generic candidate/published/executed
retained paths do not require an Overtake plan's immutable
`{intent,target,generation,side}` to match the current encounter.  Only the
dedicated sibling resolver checks that identity.

## Repair

Introduce one tactical identity predicate at retained-plan selection:

- Track/Cruise/Follow and non-sided intents are unaffected;
- ShiftOut/Pass plans must match the current target, Mission generation and
  side before entering a generic retained path;
- an opposite-side plan is still inspected by the existing sibling resolver;
  it may cross authority only with the existing publisher-bound adoption token;
- once an adoption changes side, in-flight pre-adoption candidates become
  observation-only rather than silently reclaiming authority.

This removes a duplicate cross-side authority path.  It does not retain the new
side by time, relax proof, or add a fallback.

## Validation

- source-contract and targeted C++ tests;
- full `make autoware-build`;
- `make dev2`, checking the exact adoption chronology and phase completion.
