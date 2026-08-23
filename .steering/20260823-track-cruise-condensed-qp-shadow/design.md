# Design

## Exact affine elimination

For the expanded canonical decision vector

```text
z = [x0 ... xN, u0 ... uN-1]
```

the dynamics rows define

```text
x0     = measured state
x[k+1] = F[k] x[k] + B[k] u[k] - c[k]
```

and therefore

```text
x = G u + g
z = T u + t
```

without solving an equality-constrained variable. For the expanded objective

```text
0.5 z' H z + q' z
```

the condensed objective is

```text
Hc = T' H T
qc = T' (H t + q)
```

State boxes become `lx-g <= G u <= ux-g`; input boxes and curvature-rate rows
are copied unchanged. Dynamics equality rows disappear from the solver problem.

## Reconstruction and proof

The condensed primal is expanded as `[G u + g, u]`. The unchanged expanded
matrix and bounds are then evaluated in physical units. Dynamics residual must
be numerical roundoff, and the existing semantic execution normalizer consumes
the reconstructed full primal and expanded per-row residuals.

The first Slice records:

- condensed solve status, iterations and elapsed time;
- expanded maximum dynamics residual;
- semantic execution acceptance/rejection and rejected field/stage;
- first acceleration, curvature, virtual progress and predicted-speed
  difference versus the production expanded result.

It does not run the physical wall certificate twice and does not store or
publish the condensed result. A later production-replacement Slice must pass
the complete existing physical/world certificate before deleting expanded
Track/Cruise authority.

## Warm-start provenance

The observer owns a dedicated persistent OSQP workspace. Its warm start is
input-only and its dual uses the condensed row layout. Both are shifted by
semantic stage blocks. Expanded production warm-start state is never reused.

## Rejection rule

Reject and remove the observer if it causes callback overruns, persistent
maximum iterations, non-finite results, non-equivalent objective/constraints,
or fails to reduce semantic rejects. Do not rescue it with solver settings.
