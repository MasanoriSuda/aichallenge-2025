# Validation

## Static gates

### Build

Command:

```bash
make autoware-build
```

Result: PASS. 25 packages completed. The only stderr was the existing Python
`setup.py install` deprecation warning.

### Package tests

Command (inside the repository Docker overlay):

```bash
cd /aichallenge/workspace
colcon test --packages-select multi_purpose_mpc_ros
colcon test-result --verbose
```

Result: PASS.

- 44 CTest targets passed;
- 1,827 tests/assertions reported;
- 0 errors, 0 failures, 0 skipped;
- the four new shadow solver/mailbox tests passed;
- the 24 single-authority source-contract tests passed.

The reported `joycon_contract_guard/package.xml` parser warning is an existing
stale result-artifact warning after all selected package tests passed; it is not
a test failure.

### Source and formatting gates

- `git diff --check`: PASS.
- Track/Cruise eligibility/wall-contract audit: PASS.
- Production-authority reachability audit: PASS; the result is consumed only
  by observation telemetry and is never a selector candidate.

## Dynamic gate

Committed source `9697d35` was exercised with:

```bash
make dev2
```

Evidence: `output/20260825-004100`.

| Metric | Domain 1 | Domain 2 | Combined |
|---|---:|---:|---:|
| submitted | 460 | 3,966 | 4,426 |
| consumed | 437 | 3,917 | 4,354 |
| solved and publishable | 386 | 3,473 | 3,859 |
| actuation-sample rejected | 51 | 444 | 495 |
| build / assembly / solve / nonfinite / exception | 0 | 0 | 0 |
| mailbox invalid / rollback / unsubmitted | 0 | 0 | 0 |
| maximum compute time | 2.724 ms | 11.696 ms | 11.696 ms |
| maximum solve time | 2.656 ms | 11.622 ms | 11.622 ms |

All 58 aggregate records retained `authority=shadow, selected=0`. There was no
legacy-normal authority trace, abrupt-speed-loss trace or wall-contact trace in
either controller log. Domain 2 had zero 25 ms callback overruns. Domain 1 had
13 overruns in two start-grid windows (maximum 61.362 ms); the asynchronous
shadow's maximum compute time in Domain 1 was 2.724 ms, so this run does not
attribute those overruns to the worker, but it also does not claim a clean
two-domain deadline Gate.

The numerical runtime is viable: every consumed QP built and solved, with no
non-finite result. The executable-sample Gate is **not accepted**: only
3,859/4,354 consumed results (88.6311%) supplied a certified 25 ms sample, while
495 were rejected by the combined sample validator. The current diagnostic
does not distinguish publication-period/stage-duration mismatch from steering
or steering-rate boundary failure, so changing a bound or tolerance would be
speculative.

## Decision

Accept this Slice as observation-only runtime infrastructure. Do not promote
the six-state formulation or add a fallback. The next failure-first Slice must
make the actuation-sample rejection reason typed, replay the same scenario, and
repair only the earliest violated time-base/actuator invariant.
