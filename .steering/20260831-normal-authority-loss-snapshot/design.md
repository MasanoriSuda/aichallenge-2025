# Design: normal authority-loss snapshot

## Causal boundary

The evidence boundary is after all current-world retained, Stop-suffix,
Gate-A and previous-intent joins have been attempted, but before command
selection turns missing normal authority into external Emergency Stop.

Capturing earlier would record an intermediate `intent-mismatch` even when
the preceding certified artifact successfully bridges the transition.
Capturing later would observe only the Stop symptom.

## Implementation

Extract one helper that constructs a replay-ready interaction snapshot from:

- the current `MpcProblem` and normal submission draft;
- the last actually serialized predecessor;
- the current control path and physical wall world;
- the current V2X obstacle observation.

Both terminal-contingency capture and the new final-authority-loss capture use
that helper.  The new recorder path writes an observation-only
`normal-authority-unavailable` physical-proof snapshot with the final retained
reason in its diagnostic detail.

The existing background observation worker remains the only I/O owner.  The
40 Hz callback performs no solve and acquires no new authority.

## Acceptance

- Existing terminal snapshot capture remains behaviorally identical.
- Unit/static tests prove the new call is guarded by final missing authority
  and does not feed Store/mailbox/publisher paths.
- Package build and complete tests pass.
- Bounded dynamic run records the first Follow loss with complete replay
  world and without changing the published decision sequence.
