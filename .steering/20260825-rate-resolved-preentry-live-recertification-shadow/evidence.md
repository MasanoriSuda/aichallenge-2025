# Evidence

## Static validation

- `make autoware-build`: 25 packages passed.
- After prototype removal, package CTest: 49/49 passed, 1888 test assertions,
  zero failures.
- The retained source contract proves isolated and asynchronous tactical paths
  share the same deep-owned snapshot boundary.

## Dynamic observation

### First prototype: `output/20260825-204929`

Four attempts all returned `candidate=0` in approximately 0.002 ms. The live
callback intentionally delegates tactical candidate generation to the async
worker, so its imported live assessments cannot be reused as a current-state
candidate source. This falsified the first observation method.

### Current-state rebuild: `output/20260825-205831`

The isolated current-state rebuild generated both rejected and fully certified
results:

- source sequence 5195: selected side -1, candidate available, six-state
  solver/wall/target accepted, terminal progress 16.658 m, terminal velocity
  6.068 m/s, minimum lateral reserve 0.033 m;
- total reconstruction time 110.475 ms;
- adjacent rejected reconstructions cost 87.840--115.352 ms;
- control callback maximum became 116.813 ms against the 25 ms period and
  recorded an overrun.

This supports the causal hypothesis: recomputing the selected homotopy from
the current committed state can produce a valid six-state artifact, whereas
retaining the old async trajectory cannot. It also rejects the synchronous
implementation: full tactical rebuild in the control callback is more than
four periods long.

## Resulting architecture decision

- The synchronous observation and its trigger were deleted; no overrun-causing
  code remains in the callback.
- The common deep-copy ownership boundary is retained as
  `make_owned_tactical_snapshot()` and shared by isolated branch evaluation and
  the tactical async worker. This removes two manually duplicated snapshot
  paths without changing authority.
- The next production boundary must be an asynchronous, causal selected-side
  execution producer. Tactical async output may choose homotopy, but a new
  six-state execution artifact must be bound to the current committed
  predecessor and revalidated before admission.
- Thresholds, fallback, leases and production authority were not changed.
