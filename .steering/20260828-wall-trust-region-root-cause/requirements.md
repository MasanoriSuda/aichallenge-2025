# Requirements

## Objective

Classify the frozen ShiftOut wall-refinement failure at decision 2473 without
changing production authority, solver settings, clearance, Mission lifetime,
or fallback behavior.

## Questions

1. Is the final affine QP physically/mathematically feasible?
2. Does rebuilding the Maneuver from the same current world produce a
   certifiable alternative?
3. Which constraint owner first makes the affine problem empty?
4. Is the failure caused by Mission persistence, candidate topology, the
   single-SQP wall approximation, or physical infeasibility?

## Frozen evidence

`output/20260828-174825/d1/mpcc_architecture_snapshots/000000002473-shiftout-wall-refinement-solve-rejected/snapshot.yaml`

## Constraints

- Do not change production authority.
- Do not add a resume rule, lease, grace period, timeout, fallback, or retry.
- Do not change OSQP tolerance/iterations/scaling or physical clearance.
- Do not classify solver failure alone as physical infeasibility.
- Retain the user's generated output files outside this Slice.

## Definition of done

- A/B/C/G are replayed from the immutable current world.
- Exact affine feasibility is checked with an independent LP solver.
- Trust-region constraint ownership is isolated quantitatively.
- The result names the next architecture boundary; no speculative production
  patch is added.
