# Requirements

## Objective

Remove the lifecycle defect which keeps executing an older retained selected
homotopy after the latest immutable world epoch proves that homotopy invalid
and certifies its same-epoch sibling.

## Observed failure

Run `output/20260831-110041/d1` retained ShiftOut side `-1` from source
sequence 1366.  At decision 2028 / sequence 1369, the live dual evaluation
reported:

- selected side `-1`: exact physical wall proof rejected;
- sibling side `+1`: motion, wall, dynamic-obstacle and terminal Stop proofs
  accepted;
- the old retained side `-1` artifact: still current-world executable for the
  next command.

Because sibling adoption was evaluated only when *all* selected production
authority was absent, the valid sibling was ignored.  The old artifact carried
the vehicle forward until decision 2092, where A/B/C/D all became physically
infeasible and Recovery followed.

## Constraints

- Do not add a Mission resume rule, lease, grace period, timeout or fallback.
- Do not change solver tolerances, wall/vehicle clearances, weights or horizon.
- Preserve exact same-epoch identity and current-world revalidation.
- Preserve target continuity, pre-no-return, replacement-budget and
  publication-boundary checks.
- A retained older artifact is not evidence that its homotopy remains the
  latest current-world choice.
- Do not switch if the latest selected homotopy is itself certified.

## Definition of Done

- Sibling resolution distinguishes retained command availability from latest
  current-world selected-homotopy certification.
- When the latest selected branch fails and its exact same-epoch sibling is
  certified, the sibling may replace an older retained selected artifact.
- Tactical state changes only after the sibling command crosses the canonical
  publisher boundary.
- Build and complete package tests pass.
- A dynamic run records sibling adoption before the previous no-escape
  boundary, or freezes the next same-snapshot failure for classification.

