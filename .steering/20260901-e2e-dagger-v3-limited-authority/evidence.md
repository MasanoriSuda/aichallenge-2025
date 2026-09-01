# Evidence

## Run

`output/20260901-185403`

- candidate: DAgger v3 SHA
  `3b30f567d9a6bdf5384611ff8dfd759d79c8ed683c34e326e7d940afb2e67a5f`
- production base: candidate3 SHA
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- runtime mode: `fixed_lidar_brake`
- authority bound: `0.12 rad`
- laps: `101.144 / 88.176 / 89.291 s`
- Finish: `3/3`
- penalty: `0`
- post-start and positive-acceleration stall: `0 s`
- mean forward speed: `3.392 m/s`

## Runtime authority

- coverage: `6640 / 6645` (`99.925%`)
- errors/stale intervals: `0 / 0`
- applied samples: `6640`
- clipped samples: `97` (`1.46%` of applied samples)
- weighted mean absolute applied correction: `0.00671 rad`
- maximum applied correction: `0.12 rad`
- minimum scan rate: `19.94 Hz`
- weighted average inference: `6.13 ms`
- maximum inference: `37.03 ms`

Persisted reports:

- `output/20260901-185403/d1/e2e-run-analysis.json`
- `output/20260901-185403/e2e-competition-analysis.json`
- `output/20260901-185403/e2e-spatial-authority-analysis.json`

## Comparison

| run | authority | total 3-lap time | best lap | penalty/stall |
|---|---|---:|---:|---:|
| v3 shadow `184620` | off | 276.427 s | 87.302 s | 0 / 0 |
| v3 limited `185403` | on | 278.611 s | 88.176 s | 0 / 0 |
| v2 limited `175609` | on | 278.716 s | 88.641 s | 0 / 0 |

The v3 authority run is safe and stable in the deterministic single-vehicle
gate, but is `2.184 s` slower in aggregate than its shadow control.  This is
not a demonstrated speed improvement.  It is effectively equal to the prior
v2 limited-authority single-vehicle run.

## Decision

Accept the v3 implementation for the next NPC closed-loop diagnostic because
it passes runtime and motion gates and fixed the held-out transition delay.
Do not promote it to a default and do not claim a lap-time improvement.  The
NPC scenario is the required test of the behavior that v3 was trained to
improve.
