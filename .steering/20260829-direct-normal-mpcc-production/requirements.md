# Requirements

## Objective

Replace the defective asynchronous normal-control producer with a directly
solved canonical seven-state MPCC.  The asynchronous worker remains only for
tactical alternatives.

## Root cause

`output/20260829-003210` proved that the async normal path had no authority
while the identical same-cycle solve, physical proof and current-world proof
all succeeded.  Delayed candidate adoption is therefore the authority-loss
source.

## Constraints

- One production owner for Track, Cruise, Follow, ShiftOut, Pass, Return and
  Rejoin: direct seven-state MPCC.
- The exact previous serialized command is the solve predecessor.
- Solver success is insufficient: exact physical and current-world proof stay
  mandatory.
- The last actually published certified artifact may remain as standard MPC
  continuity evidence; an unpublished async candidate may not.
- Do not add a timeout, lease, grace, fallback, solver tolerance, clearance or
  parameter change.
- Remove the temporary synchronous A/B and the async normal submission path in
  this slice.  Do not keep two permanent normal producers.

## Exit criteria

- Source contract proves there is no async normal submission/adoption owner.
- Full build and all package tests pass.
- Track/Cruise starts and keeps canonical authority in `make dev2`.
- ShiftOut, Pass and Return obtain production authority dynamically.
- Runtime p95/p99 and maximum direct-solve cost are recorded before any cadence
  decision.
