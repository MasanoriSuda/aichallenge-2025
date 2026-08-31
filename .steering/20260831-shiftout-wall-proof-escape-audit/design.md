# Design: ShiftOut wall-proof escape audit

## Observed causal chain

1. ShiftOut sequence 1010 is exactly certified and published.
2. A newer current-world sequence 1073 solves its seven-state QP.
3. QP-level progress and physical-wall refinement report accepted.
4. The final exact trajectory proof rejects a hard wall contact at stage 40.
5. No newer normal artifact can replace sequence 1010.
6. The old artifact eventually fails current-world continuation and a
   certified Stop correctly takes authority.
7. Target loss and Recovery happen only after the vehicle has stopped.

The target-loss Recovery is therefore a symptom.  The first unresolved
boundary is the mismatch between a solved/refined candidate and the exact
physical certificate.

## Missing evidence

`mpcc_rate_resolved_shadow` records QP solve and its in-solver physical-proof
failures.  The outer `evaluate_rate_resolved_pipeline` performs a second,
canonical exact physical wall evaluation after the solver returns.  A failure
there is only appended to the runtime detail string and is not frozen.

Without that exact current-world snapshot, A/B/C/D cannot distinguish:

- persistent Mission lifecycle failure;
- stateless candidate-generation failure;
- single-SQP approximation failure;
- physical infeasibility;
- model/certificate mismatch.

## Change

At the outer exact-proof rejection boundary, call the existing source-only
architecture recorder with the same immutable snapshot.  This path executes
inside the planning worker, not the control callback.  The recorder's existing
key limits output to one sample per intent, physical homotopy, stage and
failure outcome.

No candidate, Store, mailbox, publisher or authority state is modified.
