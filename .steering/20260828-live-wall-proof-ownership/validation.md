# Validation

## Static gates

- `git diff --check`: passed.
- focused source contract: 68 tests passed.
- `make autoware-build`: 25 packages built successfully.
- `colcon test --packages-select multi_purpose_mpc_ros`: 52/52 CTest
  targets passed.
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros
  --verbose`: 2,028 tests, zero errors, failures or skips.

## Dynamic gate

Three bounded `make dev2` attempts were kept separate:

- `output/20260828-214844` ran, but did not enter ShiftOut and therefore is
  not evidence for this Slice;
- `output/20260828-215124` lost AWSIM odometry after about 16 seconds and is
  excluded as an upstream simulator failure;
- `output/20260828-215316` entered ShiftOut and is the acceptance run.

The frozen pre-change run `output/20260828-212704` attributed the following
work to a single invocation of `optimize_live_overtake_line_horizon()`:

| Decision | Live receding time | Wall requests | Wall misses | Poses scanned |
|---|---:|---:|---:|---:|
| 1429 | 38.041 ms | 40 | 32 | 4,308 |
| 1494 | 22.087 ms | 40 | 19 | 2,563 |
| 1544 | 21.425 ms | 40 | 20 | 2,633 |

In the acceptance run, the ShiftOut interval reported no
`OvertakeLine runtime ownership` warning.  The cache counters changed to
approximately one current-pose anchor request per control cycle rather than
40 stage-envelope requests per optimizer call:

| One-second window | Cache requests | Misses | Callback avg/max | Overruns |
|---|---:|---:|---:|---:|
| 1697.89--1698.90 | 40 | 1 | 11.105 / 22.247 ms | 0 |
| 1698.90--1699.93 | 41 | 3 | 12.253 / 23.798 ms | 0 |
| 1699.93--1700.94 | 41 | 2 | 11.300 / 23.770 ms | 0 |
| 1700.94--1701.97 | 41 | 3 | 11.526 / 23.595 ms | 0 |

The removed synchronous scan did not remove physical proof.  During the same
ShiftOut episode the latest-only worker produced, among others:

- solution 3054: `intent=shiftout`,
  `physical_wall=accepted/applied=1/solved=1`,
  `exact_physical_final=accepted`, 2,509 exact samples;
- solution 3136: `intent=shiftout`,
  `physical_wall=accepted/applied=1/solved=1`,
  `exact_physical_final=accepted`, 2,578 exact samples.

Production published certified ShiftOut artifacts, including decision 4203
with `canonical-rate-resolved-shiftout-executed-retained` and a complete
execution-contract join.

## Separate failure retained for the next audit

The same run later followed this typed path:

`ShiftOut -> FollowPrepare` because
`Pass entry physical wall gate unresolved`, then
`FollowPrepare -> Recovery` because
`dynamic Mission wait has no wall-feasible lateral authority`.

This is not hidden by the ownership change and is not patched in this Slice.
It is the next frozen failure to audit.  No solver tolerance, wall clearance,
lease, grace period, timeout, fallback or production-authority policy changed.

## Result

The Slice passes: synchronous duplicate wall-corridor construction is removed
from live OvertakeLine reference generation, while worker-side physical
refinement, exact proof and certified publication remain active.
