# Design

## Authority boundary

This Slice adds a producer, not an authority. Production keeps its current
five-state-to-legacy execution path until the shadow gate demonstrates that the
new lifecycle covers normal Overtake cycles. A worker result can only reach the
shadow selector after live current-world proof.

## Context lifecycle

The async context key is:

```text
exact ControlIntent + intent generation + target ID
```

Observation generation and decision ID belong to each immutable job/result,
but do not reset the mailbox epoch. A newer observation is expected while a
worker is running and is handled by live revalidation. ShiftOut-to-Pass and
Pass-to-Return always change the exact intent and therefore reset the epoch,
even if target and Mission generation remain unchanged.

## Producer flow

```text
live MpcProblem + sealed MpccProblemContext
  -> immutable model/reference/wall snapshot
  -> latest-only worker
  -> dedicated canonical Overtake five-state solver context
  -> normalization + exact physical certificate + immutable plan
  -> typed mailbox
  -> live exact identity check
  -> live target/corridor/wall/current-pose proof
  -> shadow canonical selection/store
```

Only a complete plan crosses the thread boundary. Tactical state and authority
are never re-derived in the worker.

## Temporary migration state

Follow and Overtake use the same transport and lifecycle rules but separate
solver/plan instances during shadow comparison. Overtake's temporary shadow
instance is removed or merged when production authority is promoted; it must
not survive Slice 6 as a second normal controller.
