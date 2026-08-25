# Requirements

## Purpose

Remove the residual node-level normal wall-handoff owners after every normal
intent was promoted to canonical MPCC. Preserve physical proof inside the
canonical producer and independent Emergency/Recovery supervisors.

## Earliest violated invariant

A certified canonical normal command owns both longitudinal and lateral axes
for one decision. The node publisher must not reinterpret a different path or
replace either axis with a wall-handoff command after that selection.

## Reachability audit

- ActiveOvertake admission requires no canonical command, no canonical
  Emergency and no solver fallback. Current normal dispatch always returns one
  of those outcomes, so this gate has no producer.
- DynamicEscape maps to canonical `ShiftOut`. A canonical command disables the
  legacy permission; Emergency and fallback disable its monitor. Its wall and
  exit gates therefore have no reachable normal execution.
- Solver recovery admission remains reachable after bounded solver
  continuation, but reinterprets and can replace an already certified fresh
  canonical command. It is a duplicate normal owner, not an independent
  Emergency supervisor.
- `output/20260825-112734` contains zero node-level wall-handoff or
  DynamicEscape-exit traces across both vehicles.

## Scope

- Delete `LegacyWallHandoffAuthority` and all node-level wall admission/exit
  gates.
- Delete their final-publisher hold/replan/log branches and final-source enum
  values.
- Preserve canonical current-world wall certificates, executed-solution wall
  safety, bounded solver-failure continuation, Emergency and Stuck Recovery.
- Add failure-first source contracts preventing the deleted normal owners from
  returning.

## Non-scope

- No wall margin, tolerance, solver, horizon or behavior tuning.
- No change to ROS interfaces or Recovery policy.
- No new flag, lease, timeout, fallback or replacement gate.
- Do not modify or stage `aichallenge/result-summary.json`.

## Definition of Done

- Normal publication has one canonical owner or explicit supervisor override.
- No node-level normal wall gate can replace a certified MPCC command.
- Focused contracts, full package tests and `make autoware-build` pass.
- Because solver recovery handoff is reachable, a dynamic Acceptance run shows
  no authority regression before this Slice is closed.
