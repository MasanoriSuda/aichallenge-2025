# Design: stateless ShiftOut corridor authority

## Failure chain

1. An exact opposite-side Bundle crosses the canonical publisher and atomically
   changes the active side.
2. Its current-world trajectory is aligned and the six-state/DP bridge becomes
   active on the new side.
3. The legacy gap planner independently tries to regenerate a full Mission and
   rejects it with `ShiftOut/Pass path requires wall clamp`.
4. `raw_execution_corridor_blocked` treats that candidate-generation failure
   as failure of the already published execution trajectory.
5. target continuity converts the false block into Recovery.

The observed cache `side-mismatch` is transient and is repaired by the next
certified source.  Recovery happens later, while the new-side DP execution
authority is already active.  Therefore the first failed invariant is the
duplicate corridor owner, not cache expiry.

## Root cause

`should_block_live_execution_corridor()` knows only whether the phase is Pass
and lateral clearance is latched.  It cannot distinguish an entry candidate
search from the exact stateless trajectory that already crossed the publisher.
The candidate generator can therefore revoke a different, stronger authority.

## Repair

- define one publisher-bound stateless execution-source predicate from the
  live phase and immutable encounter identity;
- pass that fact to the live-corridor arbitration helper;
- make the gap planner diagnostic-only while that exact source owns execution;
- retain all downstream physical and emergency guards unchanged;
- reuse the predicate in Behavior ownership to avoid two definitions.

This removes an authority conflict.  It does not add a temporal hold or treat
Mission existence as proof.
