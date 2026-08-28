# Validation

## Static gates

- `make autoware-build`: 25 packages passed.
- package test suite: 52 CTest targets and 2,061 tests passed with zero
  failures.  `colcon test-result --verbose` retained the pre-existing stale
  `joycon_contract_guard/package.xml` diagnostic while reporting a successful
  package-test summary.
- `git diff --check`: passed.

## Dynamic evidence

The first bounded run, `output/20260828-211949`, did not create an active
Overtake window.  The second run, `output/20260828-212255`, reached a stopped
domain-2 vehicle at about 1.9 m and correctly entered SafetyBrake/Recovery;
it is excluded from the timing classification because no ShiftOut executed.

The third bounded run, `output/20260828-212704`, reproduced the post-entry
hotspot and identified its owner:

| Decision | `update_overtake_line` | Horizon | Live receding | Solved validation | Wall cache |
|---|---:|---:|---:|---:|---|
| 1429 | 38.599 ms | 0.269 ms | 38.041 ms | 0.000 ms | 40 requests / 32 misses / 4,308 poses |
| 1494 | 26.522 ms | 0.352 ms | 22.087 ms | 4.025 ms | 40 requests / 19 misses / 2,563 poses |
| 1544 | 22.431 ms | 0.946 ms | 21.425 ms | 0.000 ms | 40 requests / 20 misses / 2,633 poses |

The expensive owner is therefore
`optimize_live_overtake_line_horizon()`.  Its input-building loop requests a
hard and preferred footprint-aware lateral wall interval at every one of the
20 stages.  The path-heading-dependent cache key changes as the receding
profile advances, so 19--32 of the 40 per-call requests miss and synchronously
scan thousands of footprint poses inside the 25 ms control callback.

The core lateral optimizer, the baseline horizon evaluation and solved-path
certificate are not the dominant cost.  Increasing the cache bucket,
weakening wall clearance or changing solver settings would therefore treat a
symptom and is not justified.

## Classification

This is a computation-ownership defect: a live one-dimensional receding
planner constructs footprint-aware wall corridors synchronously while the
canonical seven-state producer already uses a latest-only worker and an
immutable certified execution artifact.  The follow-up Slice must determine
which corridor data belongs in the asynchronous producer and which minimal
current-world join belongs in the control callback.  Physical wall proof and
production authority remain unchanged in this observation Slice.
