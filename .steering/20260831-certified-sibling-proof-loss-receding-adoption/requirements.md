# Requirements

## Objective

Fix the proven Mission lifecycle defect where an active Overtake keeps an
uncertified selected homotopy even though the same immutable world epoch
contains an exact, production-certified sibling ManeuverBundle.

## Frozen evidence

Run: `output/20260831-164618/d1/autoware.log`

- sequence 901: selected side `-1` lost exact wall authority and the exact
  sibling side `+1` was published and adopted successfully.
- sequence 1104: selected side `+1` had no current-world authority while the
  exact sibling side `-1` had production authority.
- the second adoption was rejected as `overtake_adoption:no-return`.
- the previously published side `+1` artifact remained active until terminal
  successor viability failed and certified Stop took authority.

This classifies the failure as a persistent Mission lifecycle defect: candidate
generation and physical feasibility both succeeded for the sibling branch.

## Constraints

- Do not change solver tolerances, clearances, timing, lease, timeout, grace, or
  vehicle parameters.
- Do not add another fallback path.
- Do not allow arbitrary cross-side switching.
- Hard faults and exact tactical/world identity checks remain blocking.
- A currently certified selected homotopy remains preferred.
- Tactical state mutates only after the exact sibling command crosses the
  canonical publisher boundary.

## Acceptance

- An exact same-epoch, stateless, production-certified sibling may replace an
  uncertified selected homotopy during active ShiftOut or Pass even when the
  frozen Mission has established its homotopy, crossed no-return, or consumed
  a legacy replacement budget.
- Missing authority, retained/non-stateless artifacts, identity mismatch,
  sibling-side mismatch, and hard faults remain rejected.
- Unit tests, contract tests, package build, and a dynamic run pass.
- Dynamic evidence must show either repeated certified sibling adoption in one
  encounter or an explicit physical reason why no sibling authority exists.

