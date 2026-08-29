# Design: current-world feedback bundle as the Stop exit

## Root cause

The asynchronous solver certifies a trajectory from an older serialized
steering origin.  While it solves, external Stop continues path-tracking and
changes the wire steering.  Direct persistent-plan admission then compares
the new command against a different predecessor and rejects it as
`steering-unreachable`.

The existing latest-state connector already solves the exact one-dimensional
feedback problem over the unchanged steering/rate envelope.  It then replays
the remaining seven-state controls from the current state and applies the
same wall, obstacle, Follow and terminal Stop proofs.  Production nevertheless
restores `SteeringUnreachable` after a successful proof.  External Stop is
therefore retained, changes the predecessor again, and creates a fixed point.

## Candidate comparison

```text
A persistent plan
  old artifact steering + current wire predecessor
  -> steering-unreachable
  -> Stop retained

B current-world bundle
  same immutable controls/homotopy
  + exact reachable steering feedback
  + current state nonlinear rollout
  + unchanged wall/obstacle/terminal proof
  -> one certified current-world command
```

The live evidence classifies the frozen failure as `A fails, B succeeds`.
Positive/negative population C also produces exact-clear candidates, and the
offline oracle D previously established physical feasibility.  The first
violated invariant is therefore the persistent-plan publication join, not
candidate side coverage or physical clearance.

## Authority boundary

`retained_revalidation::Proof` gains an explicit source classification.  When
the feedback connector was required and every unchanged proof accepts, the
proof is classified `LatestStateFeedbackBundle` and may pass through the
existing production adapter.

The controller publishes that command through the existing canonical normal
publisher but does **not** call `Store::mark_executed()` for the source plan.
Doing so would falsely claim that the source artifact's unmodified first
command crossed the wire.  Directly reachable candidates and actually
executed plans retain their existing ledger behavior.

If any continuation, wall, dynamic, Follow or terminal proof rejects, no
bundle exists and existing Emergency Stop behavior remains unchanged.

## Deleted assumption

Delete the rule that a successfully proved latest-state connection is always
diagnostic-only.  Keep the small feedback solver itself free of Store and
publisher APIs; authority is granted only by the common retained proof and
production adapter.
