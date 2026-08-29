# Requirements

## Frozen evidence

- Production run: `output/20260830-023852/d1/autoware.log`.
- Six certified seven-state ShiftOut entries occurred.
- No episode reached Pass.
- Five episodes left ShiftOut through `DynamicMissionWait` because the
  Pass-entry physical gate had no valid current-side prefix or remained
  unresolved.
- One episode aborted after the locked target became stale/lost.
- The removed legacy receding optimiser did not run; this failure family is
  therefore downstream of tactical reference construction and canonical
  ShiftOut admission.

## Objective

Classify the repeated ShiftOut-to-Pass failure before another production
change. Evaluate the same immutable failure world with:

1. persistent Mission plus current seven-state SQP;
2. stateless receding ManeuverBundle plus the same SQP;
3. rough lattice/spline candidate plus seven-state refinement;
4. bounded offline multi-SQP/nonlinear feasibility when A/B/C fail.

## Constraints

- Production authority and runtime configuration remain unchanged.
- No new resume, lease, timeout, grace, retry or fallback rule.
- No solver tolerance, weight, speed or clearance change.
- A local solve failure is `Unknown`, not physical infeasibility.
- All compared arms must use one sealed current-world snapshot and the same
  wall grid, obstacle predictions, physical footprint and actuation state.
- The dynamic run must preserve its replay snapshot outside the ephemeral
  container before shutdown.

## Definition of Done

- At least one representative Pass-entry failure is replay-ready and sealed.
- A/B/C are evaluated on that exact snapshot; D is evaluated if required.
- The result is classified as lifecycle, candidate generation, single-SQP,
  model/certificate mismatch, scheduling/lifecycle, physical infeasibility,
  or `Unknown` with the missing evidence named.
- The experiment registry is updated before any production implementation.
