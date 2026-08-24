# Validation

## Build

Command:

```bash
make autoware-build
```

Result: PASS. All 25 packages completed. The only stderr was the existing
Python `setup.py install` deprecation warning.

## Package tests

Command inside the repository Docker overlay:

```bash
cd /aichallenge/workspace
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --verbose
```

Result: PASS.

- 44 CTest targets passed;
- 1,827 tests/assertions reported;
- 0 errors, 0 failures, 0 skipped;
- typed actuation-sample and shadow provenance tests passed;
- all 24 single-authority source-contract tests passed.

The reported stale `joycon_contract_guard/package.xml` parser warning occurs
after the selected package tests and does not represent a test failure.

## Source and formatting gates

- `git diff --check`: PASS.
- typed reason/value provenance is consumed only by shadow validation and
  aggregate telemetry.
- no configuration, tolerance, solver, fallback or authority branch changed.

## Dynamic gate

Committed source `617d839` was exercised with:

```bash
make dev2
```

Evidence: `output/20260825-005557`.

| Metric | Domain 1 | Domain 2 | Combined |
|---|---:|---:|---:|
| submitted | 406 | 4,044 | 4,450 |
| consumed | 394 | 3,995 | 4,389 |
| solved and publishable | 360 | 3,514 | 3,874 |
| actuation-sample rejected | 34 | 481 | 515 |
| publication after stage | 0 | 10 | 10 |
| initial steering limit | 0 | 261 | 261 |
| steering-rate limit | 34 | 191 | 225 |
| terminal steering limit | 0 | 19 | 19 |
| sampled steering / other | 0 | 0 | 0 |
| build / assembly / solve / nonfinite / exception | 0 | 0 | 0 |
| mailbox invalid / rollback / unsubmitted | 0 | 0 | 0 |
| maximum compute time | 2.556 ms | 10.995 ms | 10.995 ms |
| maximum solve time | 2.450 ms | 10.913 ms | 10.913 ms |

The accepted-sample ratio was 3,874/4,389 (88.2661%). All records remained
`authority=shadow, selected=0`, and all observed Domain 1 callback windows had
zero 25 ms overruns.

The dominant rejects occur at the certified physical boundary, not at an
invalid input or failed solve. Examples include optimized steering rate
`0.700001047 rad/s` against `0.700000000 rad/s`, with normalized constraint
violation still well below the solver acceptance threshold. Initial-steering
rejects likewise print at the physical steering boundary. This demonstrates a
contract mismatch: the QP/OSQP certificate admits its numerical feasibility
tolerance while the downstream sampler applies an unrelated `1e-12` absolute
physical tolerance.

Ten publication-after-stage rejects are a distinct minority timing issue and
must not be hidden by the numerical-boundary repair.

## Decision

Accept the typed diagnostic Slice. Do not relax the sampler or clamp its
output. The next failure-first Slice must audit and align physical bound
ownership across semantic input, QP rows, solver certification and executable
sampling. It must separately preserve typed visibility of the ten genuine
time-base rejects.
