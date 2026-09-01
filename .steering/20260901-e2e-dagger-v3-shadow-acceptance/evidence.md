# Evidence

- v3 SHA-256:
  `3b30f567d9a6bdf5384611ff8dfd759d79c8ed683c34e326e7d940afb2e67a5f`
- production base SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`

## Run

`output/20260901-184620`

- production mode: `fixed_lidar_brake`
- production checkpoint: candidate3, expected SHA matched
- spatial authority: disabled for every interval
- laps: `100.145 / 88.981 / 87.302 s`
- Finish: `3/3`
- penalty: `0`
- post-start stall: `0 s`
- positive-acceleration stall: `0 s`
- mean forward speed: `3.402 m/s`

## Shadow runtime

- admitted: `6341 / 6343` (`99.968%`)
- startup/freshness skips: `2`
- errors: `0`
- stale intervals: `0`
- minimum scan rate: `19.94 Hz`
- weighted average inference: `6.24 ms`
- maximum inference: `39.49 ms`
- minimum inferred capacity: `104.12 Hz`
- weighted mean absolute correction: `0.00905 rad`
- maximum interval p95 absolute correction: `0.32023 rad`
- nonzero intervals: `59 / 63`
- applied to production command: `0`

Persisted reports:

- `output/20260901-184620/d1/e2e-run-analysis.json`
- `output/20260901-184620/e2e-competition-analysis.json`
- `output/20260901-184620/e2e-spatial-shadow-analysis.json`

## Decision

The v3 runtime artifact and shadow plumbing pass the frozen acceptance gate.
Because it also removed the held-out direction-transition delay without using
that failed run for training, v3 may proceed to one explicit limited-authority
A/B at the unchanged `0.12 rad` bound.  Production defaults remain unchanged;
this result is not a promotion.
