# Evidence

## Candidate

- spatial checkpoint:
  `checkpoints/spatial-production-normal-v2/20260901_171913/candidate.npy`
- SHA-256:
  `6ae9d618ea8093b1ff7d212cae760e90c71f84749f986af479681f5f729155d1`
- production base SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- offline gate: pass (`output/20260901-e2e-spatial-production-normal-v2-gate.json`)

## Static evidence

- `make autoware-build`: pass
- full ML suite: 131 passed
- controller/launch contract suite: 32 passed
- PyTorch/NumPy spatial component parity: pass
- embedded-base mismatch, missing speed and shadow inference isolation: pass

## Closed-loop evidence

Final run: `output/20260901-174303`

- laps: 100.120 / 88.231 / 88.266 s (3/3 Finish)
- penalty: 0
- stall: 0
- production competition gate: pass
- shadow runtime gate: pass
- admitted: 6838 / 6843 (99.927%)
- skipped: 5 startup/freshness samples
- error: 0
- minimum scan frequency: 19.94 Hz
- weighted average callback: 5.90 ms
- maximum callback: 48.04 ms
- minimum inferred capacity: 104.66 Hz
- weighted mean absolute correction: 0.00721 rad
- maximum interval p95 absolute correction: 0.35843 rad
- nonzero intervals: 64 / 68

Independent preceding run `output/20260901-173425` also completed 3/3 laps
with penalty/stall 0 under the same frozen production authority.

## Decision

The runtime plumbing and candidate are accepted for an explicit,
limited-authority A/B only. Production remains candidate3 plus
`fixed_lidar_brake`; the shadow checkpoint default remains empty.
