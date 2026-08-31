# Results: independent Overtake branch publication

## Observed failure chain

In `output/20260831-085329/d2`, selected side `-1` remained executable from an
older certified artifact while a newer dual solve was running.  Frozen decision
1506 exact-certified side `+1` offline and rejected side `-1`, but live branch
evidence stayed at decision 1493 until normal authority disappeared.  The
trace still had `no_return=0`; the vehicle then entered Emergency Stop and an
actual wall-margin violation.

## Root cause

The two side solves were concurrent, but `branch_bank->replace()` ran only
after the outer evaluator waited for both.  Thus one slow branch withheld its
already completed sibling.  `offline succeeds / live fails` classifies this as
a scheduling/lifecycle defect.

## Change

- Added exact-source `merge_branch()` to the observation-only active Overtake
  branch bank.
- A newer source invalidates both older sides, then each exact-certified side
  becomes visible as soon as its own solve/proof finishes.
- Same sequence with a different immutable identity is rejected; stale results
  cannot overwrite a newer epoch.
- Selected-plan Store, publication, no-return and command ownership are
  unchanged.

## Validation

- Focused branch-bank tests: 8/8 passed.
- Source-contract tests: 94/94 passed.
- `make autoware-build`: 25 packages succeeded.
- Full CTest with ROS and workspace setup sourced: 59/59 passed.
- Bounded run: `output/20260831-091516`.

The bounded run proves early visibility directly.  In D1 telemetry at
1788135364.50, the normal mailbox's latest completed pipeline was sequence 954,
while the branch bank already exposed sequence 958 with its positive side
certified and the outer executor still reported `running=1`.  Therefore a
completed side is no longer withheld by the sibling join.

No sibling adoption occurred in this bounded sample because its exact trigger
combination did not recur.  A later Pass authority loss and wall contact is a
separate frozen failure family; it must be audited independently rather than
changing this Slice's no-return or clearance contracts.
