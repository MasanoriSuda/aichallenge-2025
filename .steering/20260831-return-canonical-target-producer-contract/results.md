# Results: Return canonical target producer contract

## Observed phenomenon

In `output/20260831-093415/d1`, Pass requested Return for more than three
seconds. Every Return Gate-A evaluation failed before solving with
`canonical current-epoch target tube unavailable`. Atomic handoff retained the
last certified Pass, which later lost terminal/wall feasibility at decision
1839 and fell to Emergency Stop.

## Root cause

The stateless Return candidate requires the canonical target horizon to prove
whether its merge remains ahead of or behind the target. The only canonical
producer, `resolve_dynamic_obstacle_contract()`, explicitly supported
Cruise/Follow/ShiftOut/Pass but excluded Return. Therefore no Return snapshot
could ever satisfy its consumer contract, regardless of current target data,
solver result or physical room.

Same-snapshot A/B/C/D and independent nonlinear checks also showed that
decision 1839 was already physically infeasible. The Return producer mismatch
several seconds earlier was the first repairable invariant.

## Implemented change

- Return is now a supported current-world dynamic-obstacle interaction intent.
- A complete `CurrentTargetTube` is mandatory for Return.
- The passing `StageCorridor` remains exclusive to ShiftOut/Pass.
- The existing stateless Return topology, ReplayWorld proof and authority
  handoff are unchanged.

No Mission rule, timeout, lease, fallback, solver setting, wall margin,
clearance or production authority was changed.

## Verification

- `make autoware-build`: 25 packages succeeded.
- complete package suite: 59/59 CTest targets, 2309 tests, zero failures.
- bounded `make dev3`: `output/20260831-100351`.
- old `canonical current-epoch target tube unavailable`: zero in D1/D2/D3.
- D1: one `Pass -> Return -> Idle` completion, no wall-margin abort or target
  stale/lost event.
- D2: Return candidate reported `return-rejoin`, `solver=solved`,
  `physical=accepted`, `dynamic=valid/clear`, `complete=1`, followed by one
  `Pass -> Return -> Idle` completion.

## Remaining independent failures

D2 later had a separate ShiftOut target-stale/lost abort and another episode
with actual footprint wall-margin violation. They occur after the repaired
Return episode and are not evidence for restoring the old producer contract or
changing parameters in this Slice.
