# Evidence

## Frozen boundary

This was an offline observation-length comparison.  Production runtime,
checkpoint, launch defaults, final command topic and fixed LiDAR brake were not
changed.  The seed-disjoint v4 data, signed-mixture head, optimizer and gate
thresholds were held fixed against the preceding two-frame experiment.

## Dataset audit

Before implementation, the correction direction's mean persistence within the
preceding eight samples was measured per run:

- ordinary train runs: `0.656--0.778`;
- hard failure sequences: `0.951--0.964`;
- validation runs: `0.480--0.894`.

This supported testing a finite window.  Negative, anchor and positive classes
still had heavily overlapping base-steering quantiles, so a base-steering-only
context was not selected.

## Diagnostic checkpoint

Generated checkpoint (not committed):

```text
aichallenge/ml_workspace/tiny_lidar_net/checkpoints/residual-history8-v1/20260901_133539
```

The input consisted of the current scan and seven preceding normalized scans.
Every sequence padded its beginning with its own first real frame.

| subset | material MAE improvement | anchor MAE |
|---|---:|---:|
| d1 hard tail | 71.4% | 0.0814 rad |
| d2 hard tail | 89.0% | N/A |
| seed 2027 train run | 7.8% | 0.0231 rad |
| unseen seed 2028 run | 11.2% | 0.0233 rad |
| full validation | 5.0% | 0.0290 rad |
| historical d3 anchor tail | N/A | 0.0982 rad |

Independent normal leakage was `0.0259 rad`.  Every admission subset reported
`fail` against the unchanged `0.01 rad` leakage limits and 30% material-gain
minimum.

## Classification

The history preserves local maneuver direction and makes both memorized hard
tails easier, but it does not generalize activation of the corrective policy.
The full successful runs and independent normal run remain counterexamples.

This rejects the narrow hypothesis that stacking a longer history into the
same strided CNN and residual objective is sufficient.  More frames do not
repair a representation/training contract which reduces the problem to
independent residual samples.

## Decision

No checkpoint is promoted and no runtime history state is added.  The next
architecture must preserve sequence semantics during training and should
retain per-beam spatial information and ego dynamics, instead of adding more
channels to the existing residual CNN.

## Verification

```text
Python syntax check:          passed
focused residual tests:      20 passed
TinyLidarNet ML test suite:  74 passed
git diff --check:             passed
```

ROS/runtime build gates were not repeated because the new input mode exists
only in the ML workspace and no runtime package or launch file consumes it.
