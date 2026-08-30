# Design: atomic ShiftOut-to-Pass handoff

## Frozen observation

In `output/20260831-074836/d1/autoware.log` episode 1 entered `ShiftOut` with
certified artifact sequence 733. Both sides were certified at the entry epoch;
the selected positive side was physically valid and the negative tactical
candidate was rejected by the entry planner because it required a wall clamp.

At `1788130156.754974684`, the tactical state changed from `ShiftOut` to
`Pass`. From decision 1492 through decision 1579, however, canonical atomic
admission repeatedly reported:

```text
previous=shiftout proposed=pass effective=shiftout
resolution=previous-retained gate_a_attempted=0
previous_candidate=accepted/733
```

At decision 1580 the old ShiftOut artifact lost its terminal Stop successor,
and normal authority fell to Emergency Stop. During the same interval Return
drafts were also being produced, so tactical phase, draft intent and published
artifact identity represented three different lifecycle states.

## Root cause

`Pass -> Return` already requires a current-world certified Return Gate A
before mutating the phase. `Idle -> ShiftOut` similarly carries Mission Gate A
evidence. `ShiftOut -> Pass` is the only normal intent transition without this
contract: fresh tactical prediction and a physical horizon are treated as
sufficient even though neither is a certified seven-state Pass artifact.

The general background worker eventually produced Pass candidates, but it is
not an atomic transition producer. In this run its first visible Pass result
arrived about 1.7 seconds after the phase mutation. Retaining the old ShiftOut
artifact during that delay is safe only until its finite certified suffix and
terminal contingency expire.

## Repair

Add a Pass Gate-A producer and proposal parallel to the existing Return Gate A:

1. While the live phase is `ShiftOut`, rebuild a prospective `Pass` problem
   from the worker-owned current-world snapshot.
2. Solve and certify its exact trajectory against current walls and peers.
3. Bind immutable target, observation generation, Mission generation and side.
4. Keep `ShiftOut` phase and authority until this proposal is current-world
   joinable.
5. Mutate to `Pass` only when the proposal is complete; canonical atomic
   admission then consumes the same certified artifact in that callback.

This does not retain old path samples as authority and does not add time-based
permission. The old ShiftOut command remains the sole owner until the exact
successor exists.

## Sibling branch finding

Later Pass epochs sometimes certified the opposite side while rejecting the
committed side. This is not the entry root cause: the opposite side was not a
valid tactical entry at Mission start, and blindly crossing the whole track
after no-return would be unsafe. That evidence remains a separate candidate
generation/lifecycle audit after the intent handoff is made atomic.
