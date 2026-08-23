# Follow row provenance audit validation

## Static

- `make autoware-build`: succeeded.
- `test_persistent_osqp`: passed.
- `test_mpcc_progress`: passed.
- Full package test: 38/38 CTest entries passed.
- Aggregate result: 1623 tests, 0 errors, 0 failures, 0 skipped.

## Dynamic

- Run: `output/20260823-150821`, `make dev3`.
- Follow shadow: 1253/1275 accepted (98.27%).
- Typed row rejects: curvature-rate 21, other 1 (stage-0 acceleration), all other typed counters 0.
- Solve time: 2.43 ms weighted average, 22.24 ms maximum.
- Authority remained `shadow`, `selected=0`.

The instrumentation did not participate in solver acceptance or command selection.
