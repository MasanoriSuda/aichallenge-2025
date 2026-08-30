# Results

## Implementation

- The pure successor observer now reports steering deltas against four
  existing state owners: command-control-origin, current-time physical,
  response-control-origin, and previous-published.
- Optional physical/response axes may be absent without invalidating the
  canonical pose/speed/command-origin sample.
- Per-cycle INFO/WARN output was replaced by a one-second telemetry window.
  The window reports reason and intent counts plus mean/max absolute error.
- Production authority, command selection, solver, Mission lifecycle,
  fallback and vehicle parameters are unchanged.

## Static verification

- `make autoware-build`: passed, 25 packages.
- successor observation: 5/5 passed.
- physical adapter: 20/20 passed.
- retained revalidation: 51/51 passed.
- command/production adapter: 14/14 passed.
- single-authority source contract: 85/85 passed.

## Dynamic verification

Run: `output/20260830-143906` (`make dev2`).

The D1 controller emitted bounded summaries rather than individual samples.
The first 1.025 s window contained 40 valid samples and no rejects:

| Steering owner | mean abs error | max abs error |
|---|---:|---:|
| command-control-origin | 0.008772 rad | 0.017856 rad |
| previous-published | 0.008772 rad | 0.017856 rad |
| current-time physical | 0.041158 rad | 0.106869 rad |
| response-control-origin | 0.037042 rad | 0.086366 rad |

Later windows preserved the same ordering. Command-control-origin and
previous-published were numerically identical in every summary. The physical
and response axes were generally several times farther from the expected Stop
successor state.

This run did not enter ShiftOut/Pass, so it does not replace the previous
execution-intent evidence from `output/20260830-142647`. That earlier run had
220 sampled ShiftOut joins and 26 sampled Pass joins with the same discrete
command-origin steering error pattern.

## Classification

The mismatch is not caused by comparing the successor against raw physical or
yaw-response steering. The canonical next problem and the last serialized
command intentionally share one steering state, while the exact Stop suffix
usually expects a state one rate step away.

The next root-cause Slice must inspect the publication-boundary sample index
and initial steering used by terminal Stop integration. It must not connect the
successor to production until the off-by-one ownership is explained and the
command-origin join is exact enough under ShiftOut/Pass.
