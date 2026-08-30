# Requirements

## Objective

When the selected active `ShiftOut`/`Pass` homotopy loses current-world
authority but its same-epoch sibling is exactly certified, publish the sibling
only before tactical no-return and atomically move tactical homotopy ownership
after the serialized command is accepted.

## Constraints

- Do not add a lease, grace period, timeout, fallback, solver tolerance, wall
  margin, or vehicle-clearance change.
- The sibling must come from the exact same immutable source epoch as the
  rejected selected branch and must pass current-world retained revalidation.
- Adoption is forbidden after no-return, during a hard fault, after the
  cross-side replacement budget is exhausted, or for a different
  target/generation/intent.
- Solver selection is not a tactical state mutation. Tactical state changes
  only after the exact command derived from the sibling crosses the publisher.
- Do not convert the stateless ManeuverBundle back into a legacy Mission
  candidate. Retire the old frozen path while preserving encounter identity,
  generation, phase, budgets, and the newly selected homotopy.

## Definition of done

- A pure, unit-tested adoption contract rejects identity, epoch, intent,
  no-return, hard-fault, and proof mismatches.
- The active sibling bank is consumed only when selected authority is absent.
- Publication carries an immutable adoption token and commits it only after
  serialized-actuation identity succeeds.
- The old frozen path cannot regain lateral authority after adoption.
- Build and package tests pass; a dynamic run confirms either adoption or an
  explicit rejection reason.
