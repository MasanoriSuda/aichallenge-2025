# Evidence

## Run

`output/20260901-190146`

- scenario: one learned ego plus two runtime NPCs
- v3 SHA:
  `3b30f567d9a6bdf5384611ff8dfd759d79c8ed683c34e326e7d940afb2e67a5f`
- authority: enabled, bounded to `0.12 rad`
- ego result: Finish, 3/3 laps, position 1
- laps: `102.363 / 89.086 / 95.532 s`
- penalty: `0`
- post-start and positive-acceleration stall: `0 s`
- distance: `1022.58 m`
- mean forward speed: `3.346 m/s`
- minimum front LiDAR range: `1.558 m`

The two runtime NPCs did not finish within the session, but the ego finished
first.  The per-domain v3 result is the identity authority because the AWSIM
v2 summary repeats `vehicle_number=1` for all runtime vehicles.

## Runtime authority

- coverage: `8758 / 8762` (`99.954%`)
- skipped: `4`
- error/stale intervals: `0 / 0`
- applied samples: `8758`
- clipped samples: `136` (`1.55%`)
- weighted mean absolute applied correction: `0.00744 rad`
- maximum applied correction: `0.12 rad`
- minimum scan rate: `19.92 Hz`
- weighted average inference: `5.82 ms`
- maximum inference: `49.53 ms`

Persisted reports:

- `output/20260901-190146/d1/e2e-run-analysis.json`
- `output/20260901-190146/e2e-competition-analysis.json`
- `output/20260901-190146/e2e-spatial-authority-analysis.json`

## Comparison with v2

| candidate/run | ego Finish | laps | wall penalty | longest stall |
|---|---|---:|---:|---:|
| v2 `20260901-180313` | no | 2/3 | 1, 137.35 s | persistent after wall |
| v3 `20260901-190146` | yes | 3/3 | 0 | 0 s |

The v3 run encountered a dynamic close-range episode: front distance fell to
`2.69 m`, the candidate requested right correction, then switched left in the
next status interval.  The slow-clearance safety policy was active for only
part of one interval, after which forward motion resumed.  This is consistent
with the held-out zero-delay transition result and unlike v2's sustained wall
commitment.

## Decision

Accept the v3 NPC hypothesis for an independent repeat.  It fixes the frozen
v2 failure in this run without parameter changes or runtime special cases.
Do not yet change production defaults: one repeat is required to distinguish
model improvement from simulator variance, and the single-vehicle result did
not show a speed benefit.
