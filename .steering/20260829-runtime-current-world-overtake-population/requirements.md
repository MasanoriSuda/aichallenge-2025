# Requirements: runtime current-world Overtake population

## Objective

Make the asynchronous canonical normal worker rebuild Overtake candidate
geometry from the current immutable world on every causal submission instead
of directly solving retained Mission path samples.

## Root cause

A frozen ShiftOut post-refinement failure has a certified same-side candidate
in the existing bounded production population, while the captured persistent
Mission geometry fails. The richer population is currently used at pre-entry
but is not the active geometry owner after Mission admission.

## Invariants

- one seven-state MPCC normal authority remains;
- Mission keeps target identity, homotopy side and commit/no-return state;
- only the selected Mission side is evaluated;
- the current-world `ReplayWorld`, wall model and exact proofs remain required;
- last actually published certified authority remains the execution bridge;
- no fallback, timeout, lease, solver tolerance, clearance or parameter change;
- the persistent direct-solve path becomes unreachable for ShiftOut, Pass and
  Return in the same Slice.

## Dynamic gate

- observe current-world candidate source in the normal worker log;
- require at least one ShiftOut execution with no persistent-geometry solve;
- reject promotion evidence if a candidate is solved but either exact physical
  proof fails;
- compare failure snapshots again before adding a fourth temporal candidate.
