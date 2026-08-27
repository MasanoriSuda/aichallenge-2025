# Design: Architecture escape-hatch platform

## Evidence boundary

- Branch: `develop_july`
- Baseline: `b6da7ebb296292bf57929ed9064a9fb789b95df0`
- Dynamic evidence: `output/20260827-175828` proves production authority only
  through ShiftOut. `output/20260827-214537` contains a Stop/wall sequence,
  while `output/20260827-221458` is only a no-regression run for that Stop
  repair.
- None of these logs serializes every solver matrix, wall grid, prediction,
  exact physical trajectory and warm-start input needed for deterministic A--D
  replay. They must therefore be registered as incomplete evidence.

## Platform layers

### Policy gate

The hard invariant is one certified normal command owner. The following are
implementation hypotheses and may be replaced after an escape-hatch trigger:

- persistent Mission lifecycle;
- candidate generator and homotopy representation;
- seven-state convexification schedule;
- solver backend.

### Failure snapshot manifest

The JSON manifest seals provenance and references immutable payloads. Large
payloads remain external artifacts and are identified by SHA-256. Required
groups are:

- evidence provenance;
- seven-state/control origin;
- reference and wall geometry;
- all relevant peer observations/predictions;
- tactical identity;
- exact problem, warm start and certificate payloads.

`replay_ready=true` is accepted only when all groups and payload hashes are
present.

### Experiment registry

Every comparison records baseline, snapshot IDs, changed dimension, method
outcomes, acceptance state, deleted production paths and revisit condition.
Rejected and inconclusive experiments are first-class results.

### A--D classifier

| Evidence | Classification |
|---|---|
| A fails, B succeeds | Mission lifecycle defect |
| A/B fail, C succeeds | candidate generation defect |
| A/B/C fail, D succeeds | live single-SQP/scheduling limitation |
| solve succeeds, proof fails | model/certificate mismatch |
| all fail + physical certificate | physical infeasibility |
| all fail without certificate | unknown |

The classifier is deliberately incapable of changing production authority.

## Follow-on comparison

The first complete Pass/Return failure snapshot will be evaluated by:

- A: persistent Mission pipeline + current seven-state SQP;
- B: stateless receding ManeuverBundle + the same SQP;
- C: rough polynomial/lattice proposal + the same seven-state refinement;
- D: bounded offline multi-start/multi-SQP feasibility solve.

Only the classification result opens the next implementation Slice.
