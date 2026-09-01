# Verification

## Static and unit evidence

- Host controller tests: 85 passed.
- Recurrent analyzer tests: 5 passed.
- Launch and single/multi-domain interface contracts: 17 passed.
- `make autoware-build`: 25 packages built successfully.
- Docker `colcon test --packages-select tiny_lidar_net_controller`: four
  registered test targets passed, including 45 core-contract cases and two
  latest-wins executor cases.
- Fatal Python static checks (`E9,F63,F7,F82`) passed.  The repository-wide
  style profile still reports pre-existing docstring, quote and line-length
  debt and is not used as an admission Gate for this Slice.

## Frozen artifact

- Path:
  `/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/conv5-recurrent-final-peers-capacity512-v1-nospeed/20260902_064016/candidate.npy`
- SHA-256:
  `b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830`
- Authority: false

## Dynamic rerun order

First run the single-vehicle shadow:

```bash
env \
  TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH=/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/conv5-recurrent-final-peers-capacity512-v1-nospeed/20260902_064016/candidate.npy \
  TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256=b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830 \
  TINY_LIDAR_RECURRENT_AUTHORITY_ENABLED=false \
  make e2e-single LOG_DIR=/output/20260902-e2e-recurrent-shadow-isolated-single
```

Then run the deterministic peer-load shadow:

```bash
env \
  TINY_LIDAR_RECURRENT_SHADOW_CKPT_PATH=/aichallenge/ml_workspace/tiny_lidar_net/checkpoints/conv5-recurrent-final-peers-capacity512-v1-nospeed/20260902_064016/candidate.npy \
  TINY_LIDAR_RECURRENT_SHADOW_EXPECTED_SHA256=b4b292e0223444c84bf85523d31d2c475386e7743416fc9d4eaff31dc7243830 \
  TINY_LIDAR_RECURRENT_AUTHORITY_ENABLED=false \
  make e2e-peer-audit-student LOG_DIR=/output/20260902-e2e-recurrent-shadow-isolated-peer
```

Analyze the student with domain 1 for the single run and domain 3 for the peer
run.  Add `--expect-async-shadow true` to
`analyze_recurrent_shadow_run.py`.

## Acceptance

- Exact recurrent artifact path and SHA.
- Recurrent authority enabled/applied/clipped counts remain zero.
- Async telemetry is present in every interval.
- Production command Gate passes with no stall or penalty.
- Scan frequency remains at least 19 Hz.
- Coverage is at least 99%.
- Async stale and inference error counts are zero.
- Hidden resets are zero during a continuous successful race.
- Dropped work is reported rather than hidden.  Any drop rate that prevents the
  99% coverage Gate rejects promotion; it does not justify queue growth or
  timeout relaxation.

No bounded-authority run is permitted until both dynamic reruns pass.
