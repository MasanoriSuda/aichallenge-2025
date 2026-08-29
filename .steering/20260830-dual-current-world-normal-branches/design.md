# Design

## 1. Atomic normal branch bank

Add a small data-only `NormalBranchBank`. One entry owns:

- the original side-free source identity;
- negative and positive certified plans from that exact source epoch;
- monotonically increasing source sequence.

`replace()` validates the source identity, each plan's complete physical
certificate and the source fields which must remain equal across homotopies.
Only `dynamic_obstacle_side_sign` and the resulting sealed problem fingerprint
may differ. Both pointers are replaced under one mutex; an empty accepted entry
invalidates older branches.

The bank has no command, Store, Mission or publisher API.

## 2. Complete dual evaluation in the existing background worker

The normal LatestOnlyWorker already owns separate persistent solver contexts
for negative and positive homotopies. Evaluate both concurrently inside that
worker, following the established Overtake dual-solve pattern:

- one branch in a child future;
- the other in the worker thread;
- join before publishing the atomic branch set.

The 40 Hz callback is not blocked. Both branches use the same immutable source
and physical snapshot, but never share an OSQP context.

After both complete, publish the atomic branch set and select the preferred
certified branch. Only the selected plan enters the existing candidate Store.
This preserves normal behavior while removing the branch-destruction edge.

## 3. Current-world alternate selection

The retained production evaluator keeps its current order:

1. selected candidate;
2. last published Bundle source;
3. last executed plan.

If these do not produce authority, read one atomic bank snapshot and evaluate
each not-yet-tried plan with `evaluate_rate_resolved_track_cruise_plan()`.
This is not a relaxed fallback: the function rebuilds the current-world Bundle
and requires the same exact physical, dynamic, command and terminal Stop
certificates as every other normal source.

When accepted, mark the result as a stateless current-world Bundle so the exact
serialized publication records that plan as the published Bundle source. The
homotopy owner is updated only after production proof succeeds.

## 4. Diagnostics

Retained evaluation records:

- whether the atomic branch bank was inspected;
- bank source sequence;
- attempted side;
- selected side;
- proof reason per attempted side.

The final decision log can therefore distinguish:

- selected branch retained;
- alternate current-world branch accepted;
- both branches rejected;
- no same-epoch branch evidence.

## 5. Deleted edge

Remove the loop-level immediate return on the first certified normal-avoidance
candidate. No replacement timeout, lease or Emergency suppression is added.
