# Design: current-world Gate A lifecycle

## Root cause

The live behavior calculation and the causal execution producer currently use
different tactical ownership rules:

1. the live control cycle can build a complete `overtake_selected_mission` and
   can solve/publish a current-world opponent-aware seven-state trajectory;
2. `build_rate_resolved_preentry_execution_draft()` ignores that live Mission
   while `OvertakeLine` is Idle;
3. it instead requires `rate_resolved_preentry_selected_mission_hint` and a
   sequence from the slower asynchronous dual-branch worker;
4. therefore executable evidence from the current control epoch cannot create
   Gate A, even though a delayed sibling producer is evaluating the same
   encounter;
5. front distance continues to contract and the scalar supervisor changes
   authority to Emergency Stop before `ShiftOut` can own execution.

The visible braking is downstream. The upstream defect is the mandatory
cross-worker lifecycle join at initial admission.

## Repair

Make the live, current-world selected Mission the canonical tactical geometry
for **inactive pre-entry only**. The geometry is not trusted as a certificate:
the existing causal Gate A worker still:

1. snapshots the current owned model/reference/gap planner;
2. binds the serialized command predecessor;
3. builds the prospective ShiftOut or Pass seven-state problem;
4. evaluates the current-world candidate population;
5. proves exact wall, dynamic-obstacle and terminal Stop safety;
6. publishes immutable Gate A evidence only when the current target, side,
   generation and observation provenance still join.

The asynchronous dual solve remains useful as advisory branch evidence and
for active replanning, but it is no longer a prerequisite for the first Gate A
submission. This follows the upper-log architecture: the main
opponent-aware solve keeps normal motion alive while optional branch
evaluations may arrive or fail independently.

## Non-goals

- No SafetyBrake suppression.
- No geometry or margin relaxation.
- No persistent reuse of path samples or certificates.
- No parameter tuning.
- No change to active no-return/cross-side replacement policy.
