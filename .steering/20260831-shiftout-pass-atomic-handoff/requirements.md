# Requirements: atomic ShiftOut-to-Pass handoff

## Objective

Eliminate the split-brain interval where the tactical phase is `Pass` while
the only published certified normal artifact still has `ShiftOut` identity.

## Constraints

- Do not change solver tolerances, wall clearance, speed policy or timing
  limits.
- Do not add a lease, grace period, timeout, fallback or Mission resume rule.
- Do not adopt an opposite-side sibling after the no-return boundary merely
  because it is certified.
- Preserve one canonical normal publisher and immutable artifact identity.
- A phase mutation may occur only when the successor intent has a current-world
  physical and dynamic certificate for the same target, generation and side.

## Acceptance

- Frozen evidence explains the first violated invariant before implementation.
- `ShiftOut -> Pass` has the same pre-mutation Gate-A contract as
  `Pass -> Return`.
- No production command may report `phase=Pass` while publishing a retained
  `intent=shiftout` artifact.
- Focused and package-wide tests pass.
- A bounded dynamic run shows either an atomic Pass handoff or an explicit
  `ShiftOut` hold while Pass authority is unavailable.
