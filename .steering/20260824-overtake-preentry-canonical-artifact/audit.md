# Audit

## Expected versus observed

Expected:

```text
dual five-state branch solved and physically certified
  -> selected immutable canonical plan
  -> atomic Mission/authority commit
  -> first ShiftOut command uses current-world-certified canonical evidence
```

Observed in `output/20260824-092036`:

```text
Idle -> ShiftOut
  -> canonical worker pending / no plan
  -> Emergency braking
  -> intermittent retained selection
  -> current progress discontinuity and solver maximum-iteration outcomes
  -> zero speed
  -> Stuck/AWSIM Recovery
```

Two intermediate Gates refined the proof contract:

- `output/20260824-095518`: the selected artifact reached entry, but target exclusion was not
  certified against a pre-entry target observation.
- `output/20260824-101828`: fail-closed target prediction showed that Idle-side workers incorrectly
  depended on the not-yet-created locked-target lifecycle.
- `output/20260824-105259`: target time/progress alignment was repaired, but retained proof checked
  the selected plan against a later regenerated Mission corridor and rejected stage 2.

The complete upstream defect was therefore a lossy artifact boundary: target snapshot and solved
lateral corridor, as well as state/control trajectory, must cross the selection boundary together.

## First violated invariant

Selected pre-entry five-state execution evidence does not imply that an immutable canonical plan is
available with the exact target prediction and lateral corridor when the selected phase takes
production authority.

## Producer

`evaluate_extended_mpcc_branch()` solves and wall-certifies the exact trajectory.
`evaluate_and_select_extended_mpcc_branches()` preserves only branch metrics and the Mission-side
certificate. Before this Slice the canonical execution artifact was discarded before the async
result reached the live FSM. The first implementation also left the target snapshot and solved
lateral bounds outside that artifact, so current-world proof silently substituted post-transition
state.

## Competing hypotheses

| Hypothesis | Support | Falsifier | Confidence |
|---|---|---|---|
| Pre-entry artifact is discarded | Source has an exact solve/proof but production plan store starts only after phase entry | A matching plan exists in the store before `Idle -> ShiftOut` | High |
| Solver settings are the first cause | Later worker results report maximum iterations | Maximum iterations precede the first pending Emergency and no solved pre-entry branch exists | Low; timing contradicts it |
| Wall/corridor config is too strict | Some current-world proofs reject corridor/wall | First phase cycle has no plan to prove at all | Contributor only |
| Worker cadence is too slow | First plan arrives after entry | Pre-entry solved plan is unavailable even with zero producer latency | Contributor only |
| Current wall margin needs tuning | Current-world proof rejects some stages | The same selected plan passes when checked with its own sealed bounds | Rejected as root |

## Pre-fix tests

- Unit contract: pre-entry adoption rejects missing/mismatched/expired artifacts and accepts one
  exact matching plan.
- Source contract: a fresh FSM entry must adopt a pre-entry plan before phase transition; the dual
  branch must construct that plan from its existing solve rather than invoke another solve.
- Dynamic replay/gate: first new-entry ShiftOut may not report `async-pending` solely because the
  Overtake canonical worker has not completed.

## Rollback

Rollback commit: `259804db9f55b0b90599c84a20bd433a4713d499`.
