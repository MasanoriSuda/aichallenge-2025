# Evidence

## Runtime contract

- authority is default-off and requires an explicit checkpoint and flag;
- stale/missing speed and inference errors publish candidate3 unchanged;
- legacy residual and spatial authority cannot be enabled together;
- every applied correction is bounded to 0.12 rad and separately counted;
- the production checkpoint and `fixed_lidar_brake` longitudinal owner are
  unchanged.

## Single-vehicle A/B

| run | authority | finish | laps | total | penalty | stall |
|---|---:|---:|---:|---:|---:|---:|
| `20260901-174303` | off (shadow) | yes | 3/3 | 276.617 s | 0 | 0 s |
| `20260901-175609` | on, 0.12 rad | yes | 3/3 | 278.716 s | 0 | 0 s |

The authority run was 2.099 s (0.76%) slower.  It processed 6,541 of 6,544
scans with no inference errors, clipped 133 corrections and never exceeded the
0.12 rad contract.  This proves bounded execution but not a performance gain.

## Runtime-NPC A/B

Frozen baseline `20260901-152109` and authority run `20260901-180313` used the
same deterministic seed and both failed on lap 3 at the same course event:

| run | authority | laps | wall event | positive-accel stall |
|---|---:|---:|---:|---:|
| `20260901-152109` | off | 2/3 | 282.371 s | 117.046 s |
| `20260901-180313` | on | 2/3 | 282.413 s | 130.630 s |

The authority run retained 99.978% spatial coverage, zero inference errors and
the 20 Hz scan contract.  It applied 9,059 corrections and clipped 1,684, but
did not avoid the existing wall failure.  The production and spatial gates
therefore reject it.

The trajectories are not identical.  Near the failure, the baseline reaches a
front return of about 1.05 m while the authority run retains about 8.05 m in
front but approaches the right side wall to 0.47 m.  The new policy moved
around the frontal obstruction, then entered a different unsupported state and
was pinned against the wall.  This is closed-loop distribution shift, not a
new failure caused solely by the authority flag.

The state-coverage audit for the final 10.5 s reports:

- production-policy embedding above its cross-sequence p95: 98.57%;
- single-frame action aliasing: 53.81%;
- successor-teacher material correction: 100%;
- mean required correction: 0.608 rad;
- opposite-sign teacher correction: 56.67%.

Raw geometry is not outside the historical geometry envelope, while the frozen
candidate3 embedding is.  The next experiment must therefore compare raw
spatial and short-history representations.  Increasing the authority bound or
adding this run to the same frozen-feature model would not address the proven
representation/observability defect.

## Decision

Reject production promotion.  Keep the default-off mechanism only as an
auditable experiment harness for future offline-gated candidates.  Candidate3
remains the shipped controller.

## Verification

```text
focused host contracts             38 passed
TinyLidarNet full suite           131 passed
make autoware-build                passed
single-vehicle dynamic Gate        passed
runtime-NPC competition Gate       failed (expected rejection)
```
