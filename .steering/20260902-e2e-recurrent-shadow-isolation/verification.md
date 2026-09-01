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

## Dynamic evidence

### Single vehicle

- Run: `/output/20260902-e2e-recurrent-shadow-isolated-single`
- Race: finished 3/3 laps, position 1/1, penalty 0.
- Laps: `84.3783`, `84.0335`, `83.9885` seconds.
- Motion Gate: pass; distance `1018.41 m`, longest low-speed and positive-
  acceleration stall both `0.0 s`.
- Production Gate: pass.
- Recurrent async Gate: pass.
- Coverage: `99.9674%` (`6139 / 6141`).
- Scan minimum: `19.94 Hz`.
- Production inference weighted mean: `5.651 ms`.
- Async recurrent inference weighted mean / maximum: `3.859 / 38.13 ms`.
- Async dropped / stale / error: `1 / 0 / 0`.
- Hidden reset maximum: `0`.

### Three-vehicle peer load

- Valid run: `/output/20260902-e2e-recurrent-shadow-isolated-peer-v2`.
- The preceding `...-peer` attempt never received the first AWSIM state and
  produced no race evidence.  It is an invalid simulator-start attempt, not a
  controller failure.
- Student domain: `d3`.
- Race: finished 3/3 laps, position 1/3, penalty 0.
- Laps: `85.0729`, `83.8286`, `84.0534` seconds.
- Motion Gate: pass; distance `1013.34 m`, longest low-speed and positive-
  acceleration stall both `0.0 s`.
- Production Gate: pass.
- Recurrent async Gate: pass.
- Coverage: `99.0543%` (`9741 / 9834`).
- Scan minimum: `19.22 Hz`.
- Production inference weighted mean: `16.969 ms`.
- Async recurrent inference weighted mean / maximum: `20.580 / 144.80 ms`.
- Async dropped / stale / error: `47 / 0 / 0`.
- Hidden reset maximum: `0`.

The peer session result file is written when the full three-vehicle session
ends.  Domain 3 itself completed in `252.95 s`; its rosbag stopped at Finish
and contains `265.01 s` of motion.  Domain 1 timing out later does not alter the
student's domain-3 acceptance result.

## Comparison with the rejected synchronous baseline

The frozen synchronous peer-load run had `95.7445%` recurrent coverage,
minimum scan rate `18.09 Hz`, 401 hidden resets and six non-ok intervals.  The
isolated run improves coverage by about `3.31` percentage points, restores the
19 Hz scan Gate, and removes all hidden resets and non-ok intervals.  The
weighted production inference time falls from about `30.91 ms` to `16.97 ms`.

Long recurrent tails still exist under three-container CPU load, but they are
now contained by the one-running/one-pending latest-wins executor.  They cause
reported diagnostic drops instead of delaying or replacing the published
production command.

## Decision

The recurrent shadow-isolation Slice is accepted.  The candidate remains
shadow-only: this result qualifies the observation architecture, not recurrent
steering authority.  Any future bounded-authority Slice must start from a new
steering decision and may not reuse the async observation worker for command
authority.
