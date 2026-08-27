# Requirements: Pass partial-escape reachability audit

## Objective

Classify the first frozen Pass failure from `output/20260828-011708` before
changing production authority or controller parameters.  Determine whether
the hard failure is owned by persistent Mission lifetime, the partial lateral
escape constraint producer, the affine SQP model, or physical infeasibility.

## Frozen evidence

- production baseline: `d154f912`;
- dynamic run: `output/20260828-011708`, Domain 1, episode 2;
- Pass transition: `1787847481.366`, waypoint 132;
- terminal failure: `1787847486.440`, waypoint 157;
- exact snapshots: sequence 1594 dynamic-obstacle refinement and sequence
  1596 wall refinement.

## Constraints

- Do not change production authority during A--D comparison.
- Do not tune a margin, weight, solver tolerance, iteration count, horizon,
  timeout, lease, grace period or fallback.
- Use the exact recorded state/input boxes, wall rows, target rows and vehicle
  dynamics for offline feasibility checks.
- A local nonlinear failure is `Unknown`, never a physical infeasibility
  certificate.

## Definition of done

- Warm/cold A replay is recorded.
- Independent linear feasibility identifies the row family that makes the
  frozen QP infeasible.
- A bounded nonlinear control-feasibility search evaluates D without changing
  production.
- The earliest invalid producer and the next structural Slice are documented.
