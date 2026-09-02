# Requirements

## Objective

Move authority-disabled recurrent observation out of the TinyLidarNet command
process so its BLAS pool, inference tail and failure lifecycle cannot delay or
change production command publication.

## Root-cause boundary

- The accepted thread executor removes recurrent work from the scan callback,
  but it still shares the production process and its OpenBLAS pool.
- A process-wide one-worker budget reduces recurrent and production latency,
  but the strict peer race Gate rejected that production change.
- Offline replay proves the frozen spatial function is byte-identical with one
  and twelve OpenBLAS workers; it does not prove identical closed-loop timing.
- Therefore the resource budget belongs on a separate recurrent process, not
  on the command publisher.

## Constraints

- Keep the sole `/control/command/control_cmd` publisher unchanged.
- Keep production checkpoint, spatial authority, acceleration, speed cap,
  safety distances and all Gate thresholds unchanged.
- Keep recurrent authority disabled; the worker is observation-only.
- The worker process must set `OPENBLAS_NUM_THREADS=1` before importing NumPy.
- Retain one-running/one-pending latest-wins scheduling and explicit submitted,
  completed, dropped, stale, error and reset telemetry.
- Verify artifact SHA and self-described runtime config inside the worker.
- Do not retain the old in-process recurrent evaluator after process promotion.
- A worker failure remains local to diagnostics and may not stop production.

## Definition of Done

- Unit tests prove latest-wins, generation reset, worker failure isolation and
  clean shutdown across the process boundary.
- Production callback no longer owns a recurrent model evaluation thread.
- Single and three-vehicle authority-disabled recurrent Gates pass with zero
  penalties, stalls, hidden resets and inference errors.
- Scan rate is at least 19 Hz and recurrent coverage at least 99%.
- Runtime evidence identifies the separate worker and one-thread budget.
- Recurrent steering authority remains false after acceptance.
