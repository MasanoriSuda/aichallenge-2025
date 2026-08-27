# Requirements

## Objective

Repair the frozen-baseline ShiftOut lifecycle defect exposed by
`output/20260828-041315`: a certified Mission remained in ShiftOut after both
its planned shift distance and total frozen path distance had been consumed,
because phase completion required the original fixed lateral goal even after
the selected-side physical separation had already been acquired.

## Constraints

- Keep production authority unchanged: the causal six-state/tactical SQP
  remains the only normal command producer.
- Do not add a lease, timeout, fallback, solver tolerance, wall-clearance, or
  target-clearance parameter.
- Do not weaken Pass admission.  Fresh dynamic-horizon and physical-horizon
  gates remain mandatory after the ShiftOut boundary is reached.
- A target that is only longitudinally separated must not satisfy the new
  lateral handoff.
- The observed relative lateral ordering must agree with the frozen homotopy.
- Preserve all user/generated result files outside this steering Slice.

## Definition of done

- A failure-first pure test proves that distance completion plus acquired
  selected-side physical lateral separation can end ShiftOut even when the
  obsolete fixed goal is not reached.
- Wrong-side separation, target discontinuity, insufficient physical lateral
  clearance, and incomplete distance remain rejected.
- Runtime logs distinguish planned-goal completion from physical-separation
  handoff.
- Package build and the complete package test suite pass.
- A bounded `make dev2` Gate checks whether ShiftOut reaches Pass or a typed
  DynamicWait/reselection without overrunning into the wall.
