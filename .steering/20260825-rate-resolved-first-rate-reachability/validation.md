# Validation

## Build

```bash
make autoware-build
```

PASS: 25 packages completed. The only stderr was the existing Python
`setup.py install` deprecation warning.

## Package tests

```bash
cd /aichallenge/workspace
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --verbose
```

PASS:

- 44 CTest targets;
- 1,830 tests/assertions;
- 0 errors, 0 failures, 0 skipped;
- exact semantic-boundary, solver-residual, empty-interior and immutable
  tolerance tests pass;
- all 24 single-authority source-contract tests pass.

The stale `joycon_contract_guard/package.xml` parser warning occurs after the
selected package tests and is not a test failure.

## Static gates

- `git diff --check`: PASS.
- strict publication sampling remains unchanged.
- first-stage reachability is represented in the QP input row rather than an
  output clamp.
- no parameter, solver setting, fallback or authority change exists.

## Dynamic gate

Committed source `e3e6661` was exercised with `make dev2`.

Evidence: `output/20260825-012740`.

| Metric | Domain 1 | Domain 2 | Combined |
|---|---:|---:|---:|
| submitted | 404 | 4,701 | 5,105 |
| consumed | 392 | 4,653 | 5,045 |
| solved and publishable | 392 | 4,642 | 5,034 |
| sample rejected | 0 | 11 | 11 |
| publication after stage | 0 | 11 | 11 |
| initial / rate / terminal / sampled steering reject | 0 | 0 | 0 |
| build / assembly / solve / nonfinite / exception | 0 | 0 | 0 |
| maximum compute time | 3.667 ms | 11.104 ms | 11.104 ms |
| maximum solve time | 3.471 ms | 11.032 ms | 11.032 ms |

Publishability is 5,034/5,045 = 99.7820%. The 19 real publication-point
steering-limit rejects from `output/20260825-011102` fell to zero. Boundary
telemetry also shows the first-stage rate interval narrowing and moving inward
near the steering limit; for example a physical lower bound of
`-0.000040339 rad/s` became a solver lower bound of `0.001661363 rad/s` with
the recorded `0.001701702 rad/s` certificate margin.

All 65 aggregate records remained `authority=shadow, selected=0`. Mailbox
invalid, rollback and unsubmitted counts were zero.

## Decision

Accept this Slice. The semantic first-rate reachability defect is closed
without a clamp or parameter change. Production authority remains blocked.

The remaining 11 rejects are exclusively the already-separated time-base
invariant: the 25 ms publication boundary can cross the first immutable stage
boundary. Repair that by integrating the certified piecewise steering-rate
sequence to the publication time in a new Slice; do not enlarge the first
stage or suppress the typed rejection.
