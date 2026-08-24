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
- 1,828 tests/assertions;
- 0 errors, 0 failures, 0 skipped;
- the new certified QP sample test covers accepted solver-boundary rate,
  invalid certificate, publication-after-stage and sampled physical violation;
- all 24 single-authority source-contract tests pass.

The stale `joycon_contract_guard/package.xml` parser warning occurs after the
selected package tests and is not a test failure.

## Static gates

- `git diff --check`: PASS.
- strict standalone sampling behavior remains unchanged.
- no clamp, solver setting, parameter, fallback or authority change exists.

## Dynamic gate

Committed source `9a5428f` was exercised with `make dev2`.

Evidence: `output/20260825-011102`.

| Metric | Domain 1 | Domain 2 | Combined |
|---|---:|---:|---:|
| submitted | 405 | 4,054 | 4,459 |
| consumed | 382 | 4,005 | 4,387 |
| solved and publishable | 382 | 3,976 | 4,358 |
| sample rejected | 0 | 29 | 29 |
| publication after stage | 0 | 10 | 10 |
| sampled steering limit | 0 | 19 | 19 |
| initial / rate / terminal duplicate rejects | 0 | 0 | 0 |
| build / assembly / solve / nonfinite / exception | 0 | 0 | 0 |
| mailbox invalid / rollback / unsubmitted | 0 | 0 | 0 |
| maximum compute time | 3.871 ms | 14.498 ms | 14.498 ms |
| maximum solve time | 2.277 ms | 14.417 ms | 14.417 ms |

Publishability increased from 88.2661% to 99.3390%. The predicted duplicate
initial/rate/terminal rejects fell from 505 to zero, confirming the ownership
diagnosis. All records remained `authority=shadow, selected=0`.

The remaining 19 sampled-steering rejects are real and must not be accepted by
certificate delegation. The QP dynamics use the solver-reconstructed state
zero, which may move inward from semantic current steering within its equality
tolerance. With no direct first-rate reachability intersection from the
semantic actuator state, an outward rate can be QP-certified yet move the real
25 ms sample outside the physical steering box.

The ten publication-after-stage rejects remain the independent time-base issue
as intended.

## Decision

Accept this responsibility cleanup. Production authority remains blocked. The
next Slice must constrain first-stage steering rate from semantic current
steering and stage duration with a solver-certificate interior margin. It must
not clamp the 25 ms sample. Piecewise cross-stage sampling follows separately.
