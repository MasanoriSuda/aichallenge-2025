# Results: Return worker isolation

## Static verification

- `make autoware-build`: 25 packages built successfully.
- package CTest: 59/59 passed.
- the architecture contract verifies that `ReturnGateA` selects its own
  latest-only worker and private solver context before submission.

## Dynamic verification

Three bounded `make dev2` runs were used without changing parameters.

- `output/20260831-150942/d1`: three episodes ended before Pass
  (`actual footprint wall margin violated`, stale target, and invalid Pass
  prefix).  These are outside this scheduling Slice.
- `output/20260831-151329/d1`: two episodes ended before Pass due to corridor
  and wall failures.  These are outside this scheduling Slice.
- `output/20260831-151543/d1`: episode 1 completed
  `Idle -> ShiftOut -> Pass -> Return -> Idle`.

At the first Return deferral in the decisive run, the Return lane was empty:

```text
worker=submitted:0/replaced:0/started:0/completed:0/running:0/pending:0
```

The first Return job then completed in `52.743 ms` with
`solver=solved`, `physical=accepted`, and `published=1`.  About 3 ms later the
phase changed `Pass -> Return`; `Return -> Idle` followed after approximately
1.47 s.

## Classification

The old 2.824-second delay was a shared-worker head-of-line scheduling defect,
not a Return timeout or clearance problem.  Isolating the execution lane makes
the first current-world Return physically feasible and available in time.

Later episodes in the same and other runs still show ShiftOut wall/corridor
failures.  They must be investigated as separate frozen failure snapshots and
must not be hidden by altering this Return Slice.
