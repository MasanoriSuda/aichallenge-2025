# Evidence

## Static and unit acceptance

- Host controller suite: `89 passed`.
- Host E2E launch contract: `3 passed`.
- Docker contract suite: `47 passed`.
- Docker executor suite: `4 passed`.
- Docker E2E launch contract: `3 passed`.
- `make autoware-build`: 25 packages completed successfully.
- Python compilation and `git diff --check`: passed.

The subprocess contract test proves that the worker reports an exact artifact
SHA, exact self-described runtime configuration, `OPENBLAS_NUM_THREADS=1`, a
distinct process ID and the same loaded parameter count as the parent-side
contract verifier.  Wrong artifact identity fails closed during worker
initialization.

## Scope boundary

This evidence does not promote recurrent steering authority.  The production
publisher, production checkpoint, spatial authority, longitudinal settings and
all dynamic Gate thresholds remain unchanged.

## Single-vehicle dynamic acceptance

Run: `output/20260902-e2e-recurrent-process-single`

The production kart completed all three required laps without a penalty or a
post-start stall:

| Metric | Observed |
|---|---:|
| Laps | 84.4332 / 83.9535 / 83.8735 s |
| Total | 252.2603 s |
| Mean forward speed | 3.8030 m/s |
| Longest low-speed interval | 0.0000 s |
| Positive-acceleration stall | 0.0000 s |

The recurrent process Gate also passed:

| Metric | Required | Observed |
|---|---:|---:|
| Coverage | >= 99% | 99.9834% |
| Minimum scan frequency | >= 19 Hz | 19.94 Hz |
| Async errors / stale / drops | diagnostic except errors=0 | 0 / 0 / 0 |
| Hidden-state resets | 0 | 0 |
| Recurrent authority applications | 0 | 0 |
| Weighted async inference | diagnostic | 1.2975 ms |
| Maximum async inference | diagnostic | 10.54 ms |

The strict motion, competition and recurrent reports all returned `pass`.

## Three-vehicle dynamic acceptance

Run: `output/20260902-e2e-recurrent-process-peer`

Domain 3 was the frozen TinyLidarNet production kart.  Domains 1 and 2 were
MPC peers and are traffic, not candidates whose completion may be substituted
for the domain-3 Gate.  Domain 3 completed the required race first and without
a penalty:

| Metric | Observed |
|---|---:|
| Final position | 1 / 3 |
| Laps | 84.9280 / 84.3333 / 84.5032 s |
| Total | 253.7645 s |
| Penalties | 0 |
| Mean forward speed | 3.8180 m/s |
| Longest low-speed interval | 0.0000 s |
| Positive-acceleration stall | 0.0000 s |

The process-isolated recurrent Gate passed under the three-container load:

| Metric | Required | Observed |
|---|---:|---:|
| Coverage | >= 99% | 99.6757% |
| Minimum scan frequency | >= 19 Hz | 19.74 Hz |
| Errors / stale | 0 / 0 | 0 / 0 |
| Hidden-state resets | 0 | 0 |
| Latest-wins drops | diagnostic | 1 |
| Recurrent authority applications | 0 | 0 |
| Weighted async inference | diagnostic | 4.0594 ms |
| Maximum async inference | diagnostic | 33.77 ms |

One pending observation was deliberately replaced by latest-wins scheduling;
it did not become a stale result, reset recurrent state, or affect production
authority.  The exact recurrent artifact SHA and self-described runtime
contract matched the worker startup proof.

## Decision

**Accept the process-isolated, authority-disabled recurrent observation
architecture.**  It removes non-authoritative recurrent inference from the
production publisher's process-wide BLAS/threading policy and passes both
frozen dynamic Gates.  This decision does not accept recurrent steering
authority, does not package the recurrent artifact, and does not revive the
synchronous authority experiment.  Any authority promotion requires a new
Slice and separate dynamic evidence.

## Test harness note

The first combined `colcon test` invocation reported a transient Python module
collection error for the controller package while the same installed module
was directly importable.  Running the installed package tests directly inside
the same Docker image passed (47 + 4 tests), and the launch package passed in
both modes.  This is recorded as a test-runner/overlay observation rather than
hidden as a product failure.
