# Promotion sequence

## Gate A: fresh dynamic evidence

Run clean `make dev` with the current HEAD and confirm for Track/Cruise windows:

```text
physically certified == canonical extracted == canonical stored
                     == cursor available == candidate accepted
                     == fresh authority ready == actuation extracted
actuation_diff == 0
authority=shadow
selected=0
```

Also record solve/certificate/callback p95, p99 and maximum. Any mismatch is an upstream defect and
blocks promotion.

### 2026-08-22 evidence update

`output/20260822-232351` completed five observed waypoint wraps after adding the explicit
numerical-to-semantic execution-primal boundary. All 9,678 physically certified cycles reached
canonical extraction, storage, cursor, candidate, fresh selector and actuation with zero actuation
difference. The former 324 `invalid-control-stage` rejects became zero.

Gate A is not yet closed because three solver results violated their own virtual-progress box-row
tolerance but passed the persistent solver's mixed-unit global absolute test. They were rejected
before physical/canonical certification. Fix the solver admission contract and repeat Gate A;
do not normalize those out-of-tolerance values or begin retained authority work first.

## Gate B: retained revalidation design

When a fresh solve is intentionally unavailable, a retained plan may be considered only after:

1. exact time cursor resolution;
2. current normal intent is Track or Cruise;
3. remaining predicted absolute progress is aligned with a current course-frame geometry;
4. every remaining state is checked against current wall and dynamic-obstacle bounds sampled at
   that progress;
5. the swept path begins at the current measured/predicted control pose;
6. the proof records the current decision and observation generation;
7. the exact retained stage is extracted without clamping/repeating.

If any input is unavailable, the selector must return Emergency Stop, not legacy MPC.

The current-intent portion is implemented by
`.steering/20260822-canonical-current-intent-contract`: the candidate problem intent must also equal
the current supervisor intent. Progress-aligned wall/obstacle revalidation remains pending.

The source audit and implementation contract for that remaining work are frozen in
`.steering/20260822-track-cruise-retained-revalidation-design`.  Static wall/course geometry is
aligned by unwrapped absolute progress, dynamic obstacle occupancy is aligned by current-relative
time plus progress, and the current measured-to-predicted control prefix is a separate physical
proof.  Production and shadow retained authority remain disconnected until Gate A passes.

## Gate C: explicit authority promotion

Only after Gates A and B:

- feed fresh/retained candidates to the selector used by the final publisher;
- publish canonical actuation for Track/Cruise;
- retain Emergency and Recovery as external supervisors;
- remove the Track/Cruise cycle-local legacy fallback and formulation handoff;
- run a short rollback-capable trial before six-lap acceptance.

This is the first Slice that changes normal command ownership and therefore is not performed
implicitly while the user is away.
