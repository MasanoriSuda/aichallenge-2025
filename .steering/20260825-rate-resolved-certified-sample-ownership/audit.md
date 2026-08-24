# Audit

## Causal finding

The QP assembly gives steering rate and predicted steering states explicit box
rows. `PersistentOsqpSolver` returns a solution only after every physical row
passes the row-scaled residual certificate. The former sampler then repeated
those same checks with a different `1e-12` tolerance and used the reconstructed
state-zero primal rather than the semantic current steering.

That downstream rejection was a second, inconsistent constraint authority.

## Responsibility audit

- The QP certificate remains the owner of steering-rate, predicted-state and
  dynamics feasibility.
- The certified sampler requires a finite normalized whole-QP violation in
  `[0, 1]`; an uncertified solve cannot enter this API.
- The integration origin is `snapshot.request.current_steering_rad`, which is
  the immutable semantic actuator state already validated by the adapter.
- The solver-reconstructed state zero remains telemetry only.
- Publication time and the actual sampled steering/curvature remain strict and
  fail closed.
- The standalone strict sampler is unchanged for non-QP callers.

## Authority audit

Only the observation-only rate-resolved shadow evaluator uses the new API.
The controller change adds one telemetry field. No result reaches a plan store,
authority selector, steering history or final command publisher. All 24
single-authority source-contract tests pass.

## Static conclusion

The duplicate ownership is removed without weakening a physical limit,
changing solver settings or adding a fallback. Dynamic shadow evidence is
required before this contract can support any production promotion.
