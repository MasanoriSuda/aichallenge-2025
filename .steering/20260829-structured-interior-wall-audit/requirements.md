# Requirements: structured interior-wall audit

## Objective

Determine whether the exact nonlinear interior-wall proof can be represented
within the existing four-rows-per-transition budget, without another
pointwise cut loop and without changing production authority.

## Evidence boundary

- Production controller, authority, solver settings, clearances and timing are
  frozen.
- The comparison uses the same immutable ShiftOut, Follow and Cruise
  snapshots as the dense-row and proof-guided audits.
- The new arm is offline and observation-only.

## Hypothesis

The current swept rows constrain affine interpolation between optimized
transition endpoints. Replacing those rows, at the same bounded sampling
budget, with partial nonlinear transition tangents will reduce the
QP-to-proof representation gap without the dense oracle's row count.

## Definition of done

- The structured arm replaces, rather than appends to, the existing swept-wall
  row family.
- At most four nonlinear interior rows are used per transition.
- Existing endpoint, progress-aligned wall, dynamic-obstacle, actuator and
  terminal constraints remain unchanged.
- Frozen replay reports solve, exact-proof and timing evidence separately.
- No production Store, worker, publisher or fallback can consume the arm.
