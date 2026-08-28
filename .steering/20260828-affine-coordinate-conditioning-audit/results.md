# Results

## Frozen failures

The audit used two exact, affine-feasible production snapshots:

| snapshot | stage | variables / rows | original OSQP |
|---|---|---:|---|
| d1 sequence 2461 | Cruise initial | 207 / 494 | maximum iterations |
| d2 sequence 301 | Cruise wall refinement | 207 / 574 | maximum iterations |

HiGHS had already proved both affine feasible. The wall-refined problem is
especially narrow: progress, lag and heading are constrained to physical wall
proof buckets as small as `0.05 m` and `0.025 rad` across the horizon.

## Affine-centred coordinate hypothesis

Moving each finite box centre to solver-coordinate zero reduced the raw scale
ratio, but did not solve either QP at the unchanged 4,000-iteration boundary:

| snapshot | scale-only | affine-centred |
|---|---|---|
| d1 2461 | maximum iterations | maximum iterations |
| d2 301 | maximum iterations | maximum iterations |

The affine-centred transform therefore is not promoted. It is mathematically
valid, but does not address the observed production boundary.

## Equality-condensing hypothesis

The 149 exact equality rows have rank 147. Exact null-space elimination
reduces each QP from 207 variables to the 60 independent control variables.
Both condensed problems still reached 4,000 OSQP iterations without a solved
status. Sparse state expansion alone is therefore not the root cause.

## Independent solver comparison

The unchanged physical QP was solved through independent CasADi conic
backends:

| snapshot | HiGHS active-set | qpOASES | ProxQP |
|---|---:|---:|---:|
| d1 2461 | solved, 342 QP iterations, ~71 ms | solved, 1071 iterations | solved |
| d2 301 | solved, 143 QP iterations, ~37 ms | solved, 1379 iterations | solved |

HiGHS and qpOASES returned physical constraint violations below `1.6e-14`.
ProxQP returned violations below `4.8e-6`. The physical QP and its convex
candidate are therefore valid; the remaining failure is specific to the
current ADMM backend on a heavily active, narrow feasible set.

The public ETH MPCC implementation likewise uses HPIPM for its structured
time-varying QP and represents track boundaries as linear half spaces. Its
obstacle-avoidance description uses a coarse dynamic-programming decision to
modify the track corridor before MPCC refinement:

- https://github.com/alexliniger/MPCC
- https://github.com/alexliniger/MPCC/tree/master/C%2B%2B

The local upper-rank trace in `.steering/ano` is consistent with that split:
`N=20`, `dt=0.12`, a continuously solved main GMPCC, and asynchronous left /
right tactical branches. Typical main solves are tens of milliseconds and
do not expose an OSQP maximum-iteration lifecycle.

## Classification

This is not:

- physical infeasibility;
- a stale Mission or current-world lifecycle failure;
- a warm-start provenance failure;
- solely a scale-origin or sparse-equality representation failure.

It is a **QP solver/backend mismatch for the current narrow active-set
formulation**. Raising OSQP iterations, loosening tolerance or accepting a
maximum-iteration iterate would hide that boundary and is rejected.

## Decision

No production code or parameter is changed in this audit. A future solver
Slice may compare a structured OCP backend (HPIPM/acados) or an active-set QP
backend under the same immutable physical certificate. That work must include
dependency/submission compatibility, warm start, p95/p99 latency and a single
normal authority cutover; it may not be installed as a second normal fallback.

The current root-cause campaign proceeds to Pass/Return/Stop lifecycle and
candidate classification instead of modifying OSQP settings.
