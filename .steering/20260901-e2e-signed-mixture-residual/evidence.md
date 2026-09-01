# Evidence

## Frozen production boundary

This Slice is offline-only.  It did not change the production TinyLidarNet
checkpoint, runtime NumPy model, launch defaults, fixed LiDAR brake or final
control output.  The frozen base checkpoint remains:

```text
de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa
```

## Hypothesis

The preceding binary material gate combined negative and positive teacher
corrections in one regression head.  The signed-mixture head tested whether an
explicit negative / anchor / positive classifier and two non-negative
magnitudes could avoid averaging opposing pass directions.

The model is exactly zero before training because equal negative and positive
probabilities and magnitudes cancel.  Train and validation used the
seed-disjoint `precontact_residual_base_v4` dataset and `scan_delta` input.

## First fit

Generated diagnostic checkpoint (not committed):

```text
aichallenge/ml_workspace/tiny_lidar_net/checkpoints/residual-signed-v1/20260901_132417
```

With anchor leakage weight `0.5`, the model separated the two strong training
tails but opened the correction on ordinary observations:

| subset | material MAE improvement | anchor MAE |
|---|---:|---:|
| d1 hard tail | 83.9% | 0.0117 rad |
| d2 hard tail | 68.3% | N/A |
| seed 2027 train run | 14.9% | 0.0381 rad |
| unseen seed 2028 run | 13.6% | 0.0381 rad |
| full validation | 6.4% | 0.0400 rad |
| historical d3 anchor tail | N/A | 0.2117 rad |

Independent normal leakage was `0.0448 rad`, against the `0.01 rad` limit.

## Bounded regularization A/B

One final A/B increased anchor leakage weight from `0.5` to `5.0` without
changing the model, dataset or admission thresholds.

Generated diagnostic checkpoint (not committed):

```text
aichallenge/ml_workspace/tiny_lidar_net/checkpoints/residual-signed-v2/20260901_132752
```

| subset | material MAE improvement | anchor MAE |
|---|---:|---:|
| d1 hard tail | 43.2% | 0.0022 rad |
| d2 hard tail | 2.4% | N/A |
| seed 2027 train run | 1.1% | 0.0090 rad |
| unseen seed 2028 run | 1.3% | 0.0092 rad |
| full validation | 3.4% | 0.0156 rad |
| historical d3 anchor tail | N/A | 0.4502 rad |

Independent normal leakage improved to `0.0106 rad`, but still exceeded the
limit.  More importantly, the opposing d2 correction and full-run material
gain collapsed.  Every admission result remained `fail`.

## Classification

The signed head confirms that direction classes are learnable in memorized
hard tails (97.6--98.8% class accuracy), but two consecutive LiDAR scans do not
identify when the teacher's tactical correction should be active.  Stronger
anchor regularization trades one direction away instead of solving that
ambiguity, and the historical normal tail is a decisive counterexample.

The bottleneck is therefore not another loss-weight choice.  The teacher target
is non-Markov or multi-modal with respect to the current observation contract:
similar scan pairs can require left, anchor or right action depending on route,
vehicle state and encounter history.

## Decision

No signed-mixture checkpoint is promoted and no closed-loop run is permitted.
The production policy remains unchanged.  Further work must change observable
state or the learning formulation (for example recurrent history plus ego
state, or a direct policy with explicit behavior context), rather than relax
the leakage gates or continue a parameter sweep.

## Verification

```text
Python syntax check:          passed
git diff --check:             passed
TinyLidarNet ML test suite:   72 passed
```

The evaluator executed every signed checkpoint end-to-end through NumPy
checkpoint loading and reported `fail` for all six admission subsets.  Runtime
and ROS build gates were not repeated because this offline Slice changes no
runtime package, launch file or production checkpoint.
