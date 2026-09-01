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
