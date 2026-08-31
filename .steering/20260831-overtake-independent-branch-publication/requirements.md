# Requirements: independent Overtake branch publication

## Objective

Prevent one slow or rejected Overtake branch from withholding an already
exact-certified sibling branch from current-world retained execution.

## Frozen evidence

- Dynamic run: `output/20260831-085329/d2`.
- At decision 1506 the offline same-snapshot comparison rejected the selected
  negative side and exact-certified the positive side.
- Live branch evidence remained at decision 1493 while the newer dual solve was
  still running; normal authority was lost and the vehicle reached an actual
  wall-margin violation.
- The runtime trace still reported `no_return=0`, so this was not a legitimate
  homotopy commitment rejection.

## Constraints

- Do not change production authority, solver tolerances, wall/vehicle
  clearance, timeouts, leases, grace periods or fallback rules.
- A branch may be exposed only after exact execution, wall and dynamic proof.
- Opposite sides from different immutable world epochs must never be paired.
- A newer epoch must invalidate all branches from an older epoch.
- Keep selected-plan publication and command ownership unchanged.

## Definition of Done

- Either side can become visible in the Overtake branch bank without waiting
  for the sibling solve.
- Same-epoch branch completion merges safely; newer epochs atomically discard
  older branches; stale completions cannot overwrite newer evidence.
- Unit tests, full package tests and build pass.
- Bounded dynamic validation records branch availability/adoption before a
  selected-side authority loss, or produces a frozen counterexample.
