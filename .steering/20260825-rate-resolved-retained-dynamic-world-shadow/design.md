# Design

## Causal chain

1. A retained six-state suffix can be statically certified and joined from
   the current ego state.
2. Admission asks whether the entire V2X set is empty rather than whether the
   suffix intersects an observed vehicle.
3. `make dev2` always observes a peer, so otherwise valid Track/Cruise suffixes
   cannot pass the retained gate.
4. Relaxing empty-world freshness would create an age-only fallback.

The correction replaces the proxy with the actual physical predicate while
preserving fail-closed observation semantics.

## Atomic observation

`V2XGapPlanner` returns one immutable observation containing generation,
receipt/source times and exact vehicles from the latest array.  A vehicle
which was not part of that generation cannot leak from the tracking map into
the proof.

## Dynamic proof

The pure retained evaluator reconstructs the expected ego pose at the cursor
and at each remaining state.  Each segment is spatially and temporally swept.
For sample time `t`, a peer centre is predicted as:

`p(t) = p_observed + v * (observation_age + t)`

The peer radius uses the existing V2X planning inflation policy.  Every sample
must have non-negative signed circle-to-oriented-ego-footprint clearance.

The measured-to-control prefix and zero-time connector are checked against the
peer position predicted to the current control time.  Static wall checks and
actuation reachability still run independently.

## Authority boundary

The output extends the retained `Proof` with dynamic-world provenance and
minimum clearance diagnostics.  It cannot build or publish a control command.
Controller integration only logs `authority=shadow, selected=0`.
