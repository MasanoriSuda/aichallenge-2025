# Requirements: Track/Cruise row-tolerance solver contract

## Objective

Test, rather than assume, whether assigning the canonical physical-unit
row-tolerance policy to Track/Cruise fixes the solved-primal/execution-boundary
mismatch. Preserve the result as a falsified hypothesis if it increases normal
authority loss.

## Evidence

Closed-loop run `output/20260824-005436` failed before a usable Overtake
interval. Domain 2 produced 153 `execution-primal-reject` outcomes after OSQP
reported `solved`: acceleration 133, predicted velocity 16, and virtual
progress speed 4. The earliest rejected row exceeded its own recorded
tolerance.

## In scope

- Assign the existing `RowToleranceNormalized` policy to the dedicated
  Track/Cruise five-state solver context.
- Remove the implicit default policy from `ExtendedBranchSolverContext` so
  every formulation owner declares its residual contract explicitly.
- Preserve exact execution-primal certification and emergency fail-closed
  behavior while the hypothesis is tested.
- Add/update static proof and run targeted, full package, build, replay, and
  closed-loop checks in that order.

## Out of scope

- OSQP tolerance, iteration, weight, horizon, wall, vehicle-clearance, or
  velocity tuning.
- Changing the shared legacy three-state solver admission behavior.
- Adding fallback, timeout, lease, feature flag, or alternate command owner.
- Overtake production promotion.

## Acceptance

- Accept only if Track/Cruise obtains stable fresh/retained authority without
  increasing emergency outcomes.
- Reject and remove the experiment if row-normalized admission turns the
  hidden boundary error into repeated solve failure.
