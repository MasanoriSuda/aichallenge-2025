# Audit result: rejected

## Static evidence

- Failure-first construction tests failed before implementation because no
  explicit solver configuration or provenance field existed.
- After the bounded candidate was implemented, the focused persistent-OSQP
  suite passed 10/10 tests.
- `make autoware-build` succeeded.
- The complete `multi_purpose_mpc_ros` test run passed 38/38 suites and
  1607/1607 tests.

Static success was not treated as production evidence.

## Dynamic falsification

Run: `output/20260823-110820`

The trace confirmed that the intended dedicated context was active through
`scaled_termination=1`.  The first easy sections were certified, but the first
difficult segment produced a severe solve/constraint cascade:

- one-second windows dropped to 4/41 and 1/41 certified solves
  (9.8% and 2.4%);
- average iterations rose to about 2492 and 2106, with a maximum of 4000;
- solver times reached about 21.2 ms;
- 32 status-transition lines reported `solve-failure` during the short run;
- 29 canonical Track/Cruise Emergency decisions were recorded;
- one stuck confirmation occurred before a complete lap;
- no callback overrun was logged, but the solve/certification collapse alone
  meets the rejection rule.

Typical failure remained a physical-unit constraint rejection despite OSQP
reporting `solved`, for example:

```text
stage=constraint_check
max_violation=0.00992458
tolerance=0.00646748
status=solved
iter=2725
scaled_termination=1
```

## Conclusion

OSQP scaled termination is not the missing Track/Cruise execution contract.
It changes the convergence coordinate but does not guarantee the downstream
physical-unit row tolerances.  It also materially worsens conditioning in the
difficult segment.  Extending this run to six laps would add risk without
further discriminating evidence.

All source and test changes from this experiment were removed.  Wall margins,
Recovery, QP weights, tolerances, and tactical left/right solvers were never
changed.  The six-lap baseline `output/20260823-104739` remains the production
reference, and Slice 3 remains open.
