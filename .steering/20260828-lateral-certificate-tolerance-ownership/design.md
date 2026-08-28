# Design

`physical_global_tolerance` is computed from the largest projected absolute
QP row value. Its scale can therefore be dominated by course progress or
another non-lateral quantity. It is valid solver telemetry, but it is not a
metric lateral-wall tolerance.

The final current-world proof already uses the safer contract:

```text
max(1e-5 m, maximum accepted physical row violation + 1e-6 m)
```

This Slice gives that existing contract one owner in
`mpcc_rate_resolved_execution_artifact` and uses it at every lateral geometry
boundary:

1. immutable artifact lateral-corridor validation;
2. fresh nonlinear exact trajectory;
3. retained nonlinear continuation;
4. final current-world wall proof.

Steering, progress-input and timing checks continue to use their existing
contracts. No margin or OSQP setting changes.

When the stricter exact nonlinear rollout rejects after wall refinement, the
existing bounded post-refinement SQP loop relinearizes around the solved
primal. This is the intended correction path; no new state or fallback is
introduced.
