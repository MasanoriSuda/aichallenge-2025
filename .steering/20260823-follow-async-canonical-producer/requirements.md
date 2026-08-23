# Follow asynchronous canonical producer requirements

## Purpose

Remove the synchronous diagnostic Follow QP from the 40 Hz control callback before any Follow
production-authority promotion. The producer may solve from an immutable observation snapshot in a
latest-only worker; the live callback may execute only a canonical plan re-certified against the
current target, wall and intent.

## Root-cause boundary

`output/20260823-181103` proves two separate facts:

- current-world retained Follow revalidation works and can reconstruct a complete command;
- `evaluate_follow_shadow()` still solves synchronously before the existing production branch, so a
  hard Follow case pays for both the shadow five-state solve and the legacy/recovery solve in one
  callback.

At decision 4426 the Follow solve reached 4000 iterations and took up to 27.292 ms. The enclosing
40 Hz callback later reported 50.746 ms maximum and 11 overruns. Moving solver tolerances or hiding
the event behind a longer lease would not remove the double-solve authority structure.

## Required invariants

- Solver work never blocks the 40 Hz command callback.
- A worker result is an immutable canonical plan candidate, not current command authority.
- The live callback never executes an async result directly; it first resolves the exact cursor and
  re-certifies the plan against current intent, target tube, physical gap and wall.
- Latest-only publication rejects old context epochs and sequence rollback.
- A rejected/stale worker result cannot replace the last accepted canonical plan.
- The Follow solver context and warm-start lifecycle have one owner and are not copied into
  independently diverging snapshots.
- No legacy fallback, age lease, extra feature flag, parameter tuning or production authority is
  introduced.
- Existing Track/Cruise production and emergency/recovery supervisors are unchanged.

## Exit gate

- Deterministic tests cover latest-only ordering and stale result rejection.
- The synchronous Follow solve is absent from the live callback.
- Dynamic Follow evidence shows zero callback overrun attributable to Follow worker computation.
- A worker-produced plan is current-world re-certified in shadow after at least one newer live
  observation.
- Production Follow authority remains a later explicit Slice.
