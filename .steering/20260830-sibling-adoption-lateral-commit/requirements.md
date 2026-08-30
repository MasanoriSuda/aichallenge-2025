# Requirements

## Evidence

Run `output/20260830-185915`, episode 1:

- `Idle -> ShiftOut` selected side `-1`.
- `ShiftOut -> Pass` completed on side `-1`.
- At Pass elapsed `0.65 s`, the locked target was laterally clear and the
  current vehicle was already on the selected side (`relative_lateral=+2.15 m`).
- A transient loss of selected-branch authority caused
  `Published stateless sibling Bundle adopted`, side `-1 -> +1`.
- The vehicle then crossed the full track, repeatedly lost exact lateral
  feasibility and ended in `committed pass longitudinal progress stalled`.
- The same cross-side adoption pattern recurred in episode 4, which ended on
  the Mission total budget.

## Root cause

The sibling-adoption contract defines pre-no-return only from target
longitudinal distance and existing tactical latches.  It does not recognize
that the selected homotopy has already been physically established by current
lateral separation.  A numerical failure of the selected branch can therefore
change tactics after ShiftOut has succeeded.

## Required behavior

- Cross-side sibling adoption may rescue an unestablished homotopy.
- Once current measured lateral separation is established on the selected
  side, sibling failure recovery may not replace that side.
- The rule must be revalidated at the single publisher boundary.
- Rejection must be explicitly classified in telemetry.
- Existing same-epoch, target, generation, hard-fault, no-return and budget
  checks remain intact.

## Constraints

- Do not change Mission time budgets, leases, grace periods or timeouts.
- Do not change solver tolerances, wall margins or vehicle clearances.
- Do not add another fallback or direct command path.
- Do not weaken exact physical or dynamic proof.

## Definition of done

- Pure contract tests prove rejection after selected-side lateral commitment.
- Publisher token revalidation rejects a token if commitment becomes true.
- Existing pre-commit sibling adoption remains covered.
- Full package tests pass.
- A `make dev2` run shows no cross-side sibling adoption after selected-side
  lateral commitment; any remaining failure is classified independently.
