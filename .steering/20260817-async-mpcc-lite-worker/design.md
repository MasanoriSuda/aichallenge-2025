# Design

## Boundary

The worker owns only the tactical snapshot evaluation. The live controller
continues to own behavior arbitration, Mission/FSM state, hard validation and
the final tracking MPC.

The submitted snapshot is a detached copy of:

- ego/model state and current bounds;
- V2X planner tracks;
- target/Mission generation, phase and selected side;
- tactical warm-start/history needed by the Frenet DP rollout;
- deep snapshots of the mutable reference path and static wall grid.

The detached planner runs the existing MPCC-lite assessment path in worker
mode, so the algorithm and scoring are not duplicated.

The deep path snapshot is required because runtime speed limits, path
constraints and Autoware trajectory replacement mutate or replace the live
`ReferencePath`. It also removes a shutdown/use-after-free risk while a rollout
is in flight. Normal worker-side behavior logs are suppressed; the live
controller emits one compact asynchronous status line.

## Latest-only worker

A reusable worker stores one pending callable. `submit_latest()` replaces a
pending callable without blocking on the running one. Completed results are
published through a small mutex-protected mailbox with a monotonically
increasing sequence.

The control callback only performs:

1. non-blocking result poll;
2. exact context/freshness check;
3. copy of tactical candidate fields;
4. non-blocking latest snapshot submission when the 10 Hz interval expires.

## Result admission

A result is consumable only when all of the following still match:

- target ID;
- Mission generation;
- `Idle`/`ShiftOut`/`Pass`/`FollowPrepare`/`Return` phase;
- locked side sign;
- configured maximum result age;
- no current emergency front risk or solver recovery.

The worker result is advisory. Existing current-state Mission admission,
physical-wall checks, target-footprint checks, no-return policy and atomic
promotion remain authoritative.

## Failure behavior

- Worker busy: replace only its pending job and keep current Mission.
- Worker exception: publish a failed result and keep current Mission.
- Stale/mismatched result: discard without changing the live Mission.
- Shutdown: request stop, clear pending work and join.

An `Idle -> Idle` "not overtaking" cleanup is treated as a no-op for the
worker context epoch. Real session resets, active Mission resets, phase/side
changes and target changes still reject old results through the epoch and exact
admission keys. This allows a first entry result to survive from one 40 Hz
cycle to the next without weakening active-Mission invalidation.
