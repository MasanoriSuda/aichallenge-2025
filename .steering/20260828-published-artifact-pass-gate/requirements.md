# Requirements

## Objective

Make the ShiftOut-to-Pass physical gate and its DynamicWait continuation
consume the same last actually published, certified seven-state artifact as
canonical production.  A live DP/reference candidate must not invalidate an
executed artifact merely because the candidate lifecycle is stale or has been
released.

## Frozen evidence

`output/20260828-215316` entered a certified ShiftOut and published artifact
sequence 3238.  While canonical production continued to execute that artifact,
the lateral supervisor released its DP/temporary solved bridge and evaluated a
different fallback reference:

- Pass entry held with `execution horizon requires wall clamp`;
- the hold expired and invalidated the Mission;
- DynamicWait reported `prefix=1` but `escape_authority=0`;
- the state moved to Recovery with
  `dynamic Mission wait has no wall-feasible lateral authority`;
- canonical production was still publishing retained sequence 3238 during the
  beginning of this sequence.

The certified store already records the exact executed artifact and its first
publication control origin atomically.  The supervisor instead projects the
artifact using its original solve time and Mission travel distance, so the
projection expires independently of the artifact's real publication cursor.

## Constraints

- Do not change wall/opponent clearance, solver tolerance, horizon, lease,
  grace period, timeout, fallback or speed policy.
- Do not change canonical production authority.
- Do not treat an unpublished candidate as executed evidence.
- Do not renew the immutable solve/snapshot timestamp during revalidation.
- Require exact target, generation, side and ShiftOut intent identity.
- The published artifact may supply only a lateral reference; the existing
  current-world execution feasibility check remains fail-closed.
- Do not commit generated run artifacts or user result files.

## Exit criteria

- Published-artifact alignment uses the store's executed snapshot and first
  publication clock, not candidate age or Mission phase distance.
- Pass entry and ShiftOut-origin DynamicWait prefer that reference before
  candidate DP/temporary solved bridges.
- Cursor exhaustion, identity mismatch or current-world wall infeasibility
  still reject the reference.
- Build and package tests pass.
- A bounded `make dev2` run demonstrates whether the frozen
  ShiftOut-to-DynamicWait failure is removed without hiding a physical reject.
