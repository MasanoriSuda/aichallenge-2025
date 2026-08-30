# Design

## Root cause

The canonical solver is a slower producer behind a 40 Hz publisher. The old
queue policy continuously replaced its sole pending immutable world. A narrow
but certifiable current-world state could therefore disappear without ever
being evaluated. Once the retained artifact lost terminal-successor proof, an
external Stop was published; later queued worlds were conditioned on that Stop
and no longer represented the feasible pre-failure state.

This is why replaying decision 7602 offline succeeds while live authority was
lost. The problem was visible at Stop publication, but was created upstream by
pending-work replacement.

## Scheduling contract

`LatestOnlyWorker` keeps its existing replacement API for tactical consumers
and gains a second explicit submission operation:

```text
submit_if_pending_slot_available(job)
  idle                         -> accept as next job
  running, no pending          -> accept one pending job
  running, pending already set -> reject new job; preserve old pending job
```

The canonical seven-state normal producer uses this operation. It remains
non-blocking and bounded to one running plus one pending job. A rejected new
world never acquires authority; an accepted older completion still passes the
existing exact current-world retained proof before publication.

## Why this is not the rejected cadence experiment

The former 20 Hz experiment changed when submissions were allowed while still
using replaceable pending work. It could still erase a feasible world and it
introduced an arbitrary timing parameter. This Slice changes no cadence. It
applies queue backpressure based solely on bounded capacity, matching the upper
implementation's observed non-blocking `queue full -> reject new` behavior.

## Safety invariants

- No completed result publishes without immutable identity validation.
- No candidate gains command authority without current wall, peer and terminal
  successor proof.
- Queue age alone grants no authority.
- The last actually published certified artifact remains the only bridge while
  a queued snapshot is evaluated.

