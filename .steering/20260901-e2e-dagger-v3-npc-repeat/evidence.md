# Evidence

## Seed 2027 run

`output/20260901-191114`

- ego result: Finish, 3/3 laps, position 1
- laps: `102.423 / 89.655 / 102.593 s`
- penalty: `0`
- post-start and positive-acceleration stall: `0 s`
- distance: `1023.47 m`
- mean forward speed: `3.267 m/s`
- front / left-front / right-front minimum LiDAR ranges:
  `1.392 / 0.770 / 1.041 m`

## Runtime authority

- coverage: `8853 / 8859` (`99.932%`)
- skipped: `6`
- error/stale intervals: `0 / 0`
- applied samples: `8853`
- clipped samples: `210` (`2.37%`)
- weighted mean absolute applied correction: `0.00883 rad`
- maximum applied correction: `0.12 rad`
- minimum scan rate: `19.94 Hz`
- weighted average inference: `5.68 ms`
- maximum inference: `42.02 ms`

Persisted reports:

- `output/20260901-191114/d1/e2e-run-analysis.json`
- `output/20260901-191114/e2e-competition-analysis.json`
- `output/20260901-191114/e2e-spatial-authority-analysis.json`

## Reproducibility

| seed/run | ego Finish | laps | total | penalty | stall |
|---|---|---|---:|---:|---:|
| 2026 `190146` | yes | 3/3 | 286.981 s | 0 | 0 s |
| 2027 `191114` | yes | 3/3 | 294.672 s | 0 | 0 s |

Both starts include close dynamic-obstacle encounters, yet neither reproduces
the candidate3/v2 wall lock.  The slower third lap in seed 2027 is traffic
sensitivity, not a stall or penalty.  Average total time across the two seeds
is `290.827 s`.

## Decision

The v3 dynamic-obstacle improvement is reproducible across two start seeds.
Qualify v3 as the leading promotion candidate.  Before changing defaults,
audit the submission artifact path and launch/package contract so the promoted
model is actually included in `aichallenge_submit/`, remains a single learned
steering owner, and can be rolled back explicitly.
