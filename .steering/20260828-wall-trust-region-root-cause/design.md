# Design

## Audit flow

```text
frozen current world
  -> A persistent Mission
  -> B stateless left/right
  -> C rough lateral/disjunction lattice
  -> G bounded production candidate population
  -> unchanged seven-state SQP + exact wall/dynamic proof

recorded wall-refined QP
  -> independent HiGHS feasibility LP
  -> row-group removal tests
  -> selective minimum-slack tests
  -> recorded pre-refinement primal violation trace
```

The candidate comparison distinguishes Mission/candidate lifecycle from the
common solver/refinement pipeline. The LP analysis then distinguishes OSQP
convergence from an actually empty affine feasible set.

## Constraint grouping

The exact row layout is decoded from the serialized seven-state problem:

- dynamics equalities;
- state and input boxes;
- steering-prefix rows;
- progress-aligned wall rows;
- swept-wall rows;
- dynamic-obstacle rows.

The physical wall refinement does more than add explicit wall rows. It also
replaces lateral, lag, heading, and progress state bounds with narrow buckets
around the provisional solution. Therefore those state families must be
audited separately; removing only rows named `wall` is insufficient.

## Decision rule

- A fails and B succeeds: persistent Mission lifecycle defect.
- A/B fail and C/G succeeds: bounded candidate-generation defect.
- All current-world candidates share an affine-infeasible wall refinement,
  while removing a refinement-owned state bucket restores feasibility:
  single-SQP/refinement construction defect.
- A nonlinear or alternate-tangent solve is still required before declaring
  the physical maneuver feasible. Failure of this affine audit is not proof
  of physical infeasibility.

## Architecture boundary

The next experiment may change only offline candidate/tangent construction.
It may not change production authority, wall clearance, solver tolerance,
Mission lease, timeout, or fallback behavior. A restoration result is only a
seed: production eligibility still requires rebuilding the unchanged full QP
and passing the exact physical wall, dynamic-obstacle, and successor proofs.
