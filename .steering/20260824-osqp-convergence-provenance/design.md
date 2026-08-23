# Design

## Causal boundary

Expected:

```text
OSQP success
-> exact physical-row certificate
-> executable five-state primal
```

Observed in `output/20260824-020904`:

```text
OSQP success
-> common adapter accepts by a global mixed-unit tolerance
-> execution-primal checks one input/state row in physical units
-> certified-bound-violation
-> canonical Emergency Stop
```

The downstream check is currently a detector. The first unknown boundary is
why OSQP's successful termination and the exact physical row report disagree.
Successful Track/Cruise outcomes discard `pri_res` and `dua_res`, so the
existing log cannot decide between the following hypotheses.

| Hypothesis | Supporting evidence | Falsifier |
|---|---|---|
| H1: mixed-unit global termination admits a small physical row outside its own tolerance | rejected rows coexist with 20 m-class progress rows and default `scaled_termination=0` | OSQP residual is already below the rejected row's own physical tolerance and no large global scale is involved |
| H2: the QP pins commands exactly to hard bounds and ordinary ADMM error crosses the semantic boundary | rejects cluster at acceleration/velocity/progress-speed limits | rejected rows are not active bounds or have substantial requested reserve |
| H3: warm transport is the producer | many rejects were warm | cold rejects persist and zero-stage transport did not remove the failure |

H3 was falsified by `.steering/20260824-stage-aligned-warm-start-transport`.
This Slice measures H1 versus H2; it does not repair either.

## Telemetry contract

`SolveTelemetry` records the exact `OSQPInfo` and termination settings used for
that solve:

- primal/dual residual;
- objective, rho updates and rho estimate;
- `eps_abs`, `eps_rel`, scaling iterations and scaled-termination mode;
- whether physical-row preconditioning was active and its maximum row scale;
- the physical global constraint scale/tolerance used by the adapter.

`SolveResult` already owns per-row physical violations and tolerances. The
Track/Cruise cycle result joins their maximum normalized row/ratio with the
solver telemetry and the semantic execution-primal rejection.

## Behavior preservation

All new values are write-only diagnostics. No decision predicate consumes
them. The existing solver, certificate, authority and publisher flow remains
unchanged.

## Deletion

No production branch is added or removed in this observation-only Slice. It
has a named deletion milestone: once the root producer is repaired and the
dynamic gate proves the mismatch absent, verbose convergence fields may be
reduced to aggregate counters while the common solver provenance remains.
