# Design

## Ownership before

```text
QP physical rows + OSQP row certificate
  -> strict sampler repeats initial/rate/terminal limits at 1e-12
  -> rejects a result the upstream certificate accepted
```

The duplicate sampler check is not stronger safety evidence because it ignores
the solver's per-row tolerance provenance and substitutes the reconstructed
state-zero primal for the known current actuator state.

## Ownership after

```text
QP certificate
  owns: steering-rate box, predicted steering-state boxes, dynamics

Certified sampler
  owns: semantic current steering, publication time, sampled steering,
        curvature conversion
```

`evaluate_certified_actuation_sample()` requires a finite whole-QP normalized
violation in `[0, 1]`. It never clamps. It integrates the certified first rate
from the semantic current steering and rejects if the requested publication
time is outside the first stage or the actual 40 Hz sample leaves the physical
steering box.

The existing strict `evaluate_actuation_sample()` remains unchanged for data
that does not carry a QP certificate.

## Expected evidence

- initial/rate/terminal duplicate rejects become zero;
- publication-after-stage remains typed rather than silently shortened;
- sampled-steering violation remains zero or exposes a real formulation bug;
- build/solve/mailbox failures remain zero;
- all results remain `authority=shadow, selected=0`.
