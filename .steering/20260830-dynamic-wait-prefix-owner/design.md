# Design: dynamic-wait prefix owner removal

## Alternatives

1. Increase the prefix wall clearance tolerance: rejected; the prefix is not
   the production authority and the exact canonical artifact is already
   certified.
2. Add a prefix retry or grace period: rejected; this preserves split
   ownership and hides the stale lifecycle.
3. Immediately reset to Idle/fresh search: rejected; it still lets legacy
   prefix availability mutate tactical identity.
4. Preserve `FollowPrepare` when the optional prefix is unavailable and defer
   command authority to canonical admission: selected.

## Ownership rule

`resolve_dynamic_mission_wait()` owns the tactical transition. Its `Hold`
result means no phase transition. `publish_dynamic_wait_forward_prefix()` is
only an optional reference producer. Its failure may be logged, but may not
override Hold with Recovery.

The later canonical admission already has the required outcomes:

- a currently proved ShiftOut/Pass artifact is retained and published;
- a fresh current-world artifact supersedes it;
- no proved artifact produces Emergency Stop.

No replacement command source is introduced.

## Hard faults

Hard wall contact, missing wall observation, emergency front risk, solver
Recovery and forbidden waypoint are evaluated before Hold and resolve to the
existing Recovery action. This Slice does not weaken those edges.

## Falsifiers

- a failed prefix is serialized as a normal command;
- a hard fault remains in FollowPrepare rather than Recovery;
- a no-prefix wait cannot accept a fresh current-world replacement;
- the phase remains FollowPrepare indefinitely after the existing terminal
  wait budget expires;
- the change increases Emergency tails or callback overruns.
