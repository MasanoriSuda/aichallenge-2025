# Design

## Root cause

The terminal Stop worker was triggered only when a selected ShiftOut/Pass
artifact crossed the publisher.  It copied that artifact's original solver
snapshot and constructed the Stop from stage zero plus one publication
interval.  With asynchronous branch solving, the artifact could already be
far into its horizon when first published.  Sequence 929 first crossed the
publisher with an artifact cursor near 0.75 s, while its Stop source still
represented the old stage-zero boundary.  Reusing that certified plan in the
same tactical scope did not repair the temporal mismatch; current-world join
correctly rejected it as `steering-unreachable`.

## Same-snapshot classification

At decision 1576:

- persistent fixed-path Stop: wall rejected;
- stateless and rough maneuver arms: no certified normal Bundle;
- free seven-state Stop from the current-world snapshot: accepted in about
  69 ms with exact wall/dynamic/rest proof;
- seven-state control-lattice Stop: accepted in about 27 ms.

The scene is not physically stop-infeasible.  The earliest defect is the Stop
producer's observation origin.

## Replacement flow

```text
current observation
  -> serialized predecessor crosses publisher
  -> immutable current-world seven-state snapshot
       |-> normal latest-only worker
       `-> Stop latest-only worker
             -> maximum-braking references from current state
             -> free seven-state steering solve
             -> exact wall/dynamic/rest certificate
  -> tactical-scope match
  -> current-world retained revalidation
  -> canonical selection only if joined
```

The Stop candidate begins at the snapshot's control prediction origin.  The
snapshot already contains the exact current-command delay prefix and the
serialized predecessor, so no historical normal artifact or synthetic second
publisher interval is required.

## Cleanup

`update_published_stop_lattice_observation()` remains responsible for the
live tactical scope and invalidation boundary only.  It no longer submits a
Stop solve from `selected_plan->solver_source_snapshot`.  Current-world Stop
submission is colocated with current-world snapshot creation.
