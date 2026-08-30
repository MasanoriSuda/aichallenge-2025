# Requirements

## Objective

Repair the producer/Store contract which rejects a certified active-Overtake
left/right pair even though both branches belong to the same immutable world
epoch.

## Evidence

In `output/20260830-144538`, Pass sequence 1529 was solved and both sides were
physically and dynamically certified, but the production Store reported
`store=invalid-plan`. The Store retained sequence 1379 until its progress join
exceeded tolerance, normal authority selected emergency Stop, and stuck
recovery followed.

The Store's pair validator currently accepts only Cruise/Follow dynamic
avoidance siblings. The active Overtake producer calls the same `replace_pair`
API with ShiftOut/Pass siblings, which makes every two-sided success invalid.

## Constraints

- Do not change Mission lifecycle, authority selection, solver settings,
  timing, tolerances or clearances.
- Preserve strict same-sequence, same-snapshot and full-context identity.
- Permit only the two explicit sibling families already produced by the
  architecture: Cruise/Follow dynamic avoidance and ShiftOut/Pass Overtake.
- An Overtake sibling differs only in execution side and dynamic-obstacle
  side; all other sealed problem context fields must match.
- Keep candidate, published-bundle and executed sibling ownership atomic.

## Definition of Done

- Store accepts a same-epoch certified ShiftOut or Pass sibling pair.
- Store rejects mixed intent, wrong epoch and non-opposite side pairs.
- Publication/execution promotion preserves the exact Overtake sibling.
- Existing normal sibling tests continue to pass.
- Build and focused contract tests pass.
- A dynamic Pass run shows newer two-sided certified epochs reaching Store
  without `store=invalid-plan`.
