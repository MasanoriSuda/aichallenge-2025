# Verification

## Runtime audit

The development image contains NumPy `1.26.4` linked to OpenBLAS.  Without an
environment override, `threadpoolctl` reports:

```text
internal_api: openblas
threading_layer: pthreads
num_threads: 12
```

The peer-512 current-sample GRU was benchmarked for 1,000 calls after warm-up
using the exact converted artifact:

| OpenBLAS workers | Mean [ms] | p95 [ms] | p99 [ms] | Max [ms] |
|---:|---:|---:|---:|---:|
| default 12 | 0.257 | 0.367 | 2.135 | 5.355 |
| 1 | 0.164 | 0.191 | 0.255 | 0.303 |

One worker is faster on average and removes the multi-millisecond isolated
tail.  Dynamic racing evidence is still required because this benchmark does
not include ROS callbacks, AWSIM or peer processes.

## Static and package evidence

- Python syntax: pass.
- Host E2E launch contract: 3 passed.
- Host TinyLidarNet controller tests: 85 passed.
- `make autoware-build`: 25 packages passed.
- Docker `colcon test`:
  - TinyLidarNet: 45 core, 2 executor, 8 governor and 6 safety cases passed;
  - submit launch: 2 localization and 3 E2E contract cases passed;
  - aggregate result: 2,351 tests, 0 errors, 0 failures.
- `git diff --check`: pass.

## Dynamic rerun order

Use the accepted peer-512 artifact as authority-disabled async shadow.  First
run `e2e-single`; only after it passes, run `e2e-peer-audit-student`.  Require
the same race, motion and recurrent async Gates as the accepted isolation
Slice.  Confirm startup contains `OpenBLASThreads: 1`.

## Dynamic evidence

### Single vehicle: pass

- Valid run: `/output/20260902-e2e-thread-budget-single-v2`.
- The preceding `...-single` run was interrupted externally after about 140
  seconds and is not acceptance evidence.
- Race: finished 3/3 laps, position 1/1, penalty 0.
- Laps: `84.5582`, `83.7886`, `84.2583` seconds; total `252.6051` seconds.
- Motion Gate: pass; distance `1018.16 m`, longest low-speed and positive-
  acceleration stall both `0.0 s`.
- Production and recurrent async Gates: pass.
- Recurrent coverage: `99.9823%` (`5643 / 5644`).
- Minimum scan rate: `19.98 Hz`.
- Production inference weighted mean / maximum: `1.367 / 4.44 ms`.
- Async recurrent weighted mean / maximum: `0.826 / 3.62 ms`.
- Async dropped / stale / error and hidden reset: `0 / 0 / 0 / 0`.

The accepted pre-budget single reference had production mean `5.651 ms` and
async recurrent mean / maximum `3.859 / 38.13 ms`.  One worker materially
reduced both average and tail latency in this run.

### Three vehicles: reject

- Run: `/output/20260902-e2e-thread-budget-peer`; student domain `d3`.
- The simulator and all three domains reached `Grounded`; this is a valid run.
- Race: finished 3/3 laps, position 1/3, but penalty count 2.
- Laps: `85.4577`, `206.6560`, `85.2728` seconds.
- Penalties: crash `26.145 s`, wall `76.110 s`.
- Motion Gate: fail; longest low-speed `60.237 s`, longest positive-
  acceleration stall `21.839 s`.
- Recurrent compute evidence itself remained healthy: coverage `99.9892%`,
  minimum scan `19.92 Hz`, weighted async mean / maximum `1.298 / 27.77 ms`,
  dropped / stale / error and hidden reset `0 / 0 / 0 / 0`.

The strict competition and recurrent Gates reject the run because computation
quality is not sufficient when the frozen production controller incurs a
race penalty and stall.

## Root-cause separation

`interaction-divergence.json` compares the rejected run with the accepted
pre-budget peer run.  During the ten seconds before the rejected run's first
sustained stall, the diagnostic side-clearance teacher found a right-side
hazard with right distance p05 `0.330 m`.  The published command pointed
toward the obstructed side for `85%` of those hazard samples, and the frozen
spatial candidate under-projected the teacher escape for `90%`.  The clean run
did not encounter the same state distribution.

The exact production spatial artifact was also replayed over the same 7,805
LiDAR frames with OpenBLAS worker counts 1 and 12.  The generated reports were
byte-identical (SHA-256
`3f68c5f6380ea572c4f66689ee18e9b0999cee9749f976dff8d6b6ce3811a08d`).
This rules out a changed offline learned function.  It does not prove that
closed-loop scheduling cannot change the encounter, so the production race
Gate remains authoritative.

## Decision

Reject production promotion of the hard-coded one-worker launch environment.
Remove the launch environment, startup telemetry and its launch assertion.
Keep recurrent authority false.  Do not compensate by relaxing penalty,
stall, freshness or coverage thresholds.

The benchmark and dynamic evidence still establish a useful architectural
requirement: a future recurrent-authority worker needs a separately isolated
one-thread process or equivalent compute boundary so its resource budget does
not alter the production controller process.  The rejected peer run is also
retained as immutable interaction data for that future Slice; no obstacle
avoidance patch is added here.
