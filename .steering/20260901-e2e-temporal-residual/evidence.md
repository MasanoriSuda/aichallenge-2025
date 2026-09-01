# Evidence

## Frozen production baseline

This Slice did not change the admitted TinyLidarNet checkpoint, fixed LiDAR
brake, default launch mode or production command path.  A residual is still
constructed only when an explicit residual checkpoint is supplied.  The new
architecture selector defaults to `stateless` and is forwarded only with that
explicit checkpoint.

## Contract verification

- Offline and runtime inputs are `current` and `current - previous`.
- The first delta in every recorded sequence and after a runtime stale reset is
  exactly zero.
- Dataset history cannot cross a sequence boundary.
- Torch and NumPy two-channel inference agree in the contract test.
- A stateless checkpoint cannot be loaded as `scan_delta`, or vice versa,
  because the first-convolution shape is checked exactly.

Focused verification before the full gate:

```text
ML residual tests:       15 passed
runtime contract tests:  17 passed
launch contract tests:    3 passed
system contract tests:    7 passed
```

Final regression gate:

```text
TinyLidarNet ML tests:       69 passed
TinyLidarNet runtime tests:  33 passed
submit launch tests:          5 passed
system interface tests:       7 passed
make autoware-build:         25 packages finished, successful
```

## Frozen experiment

Dataset:

```text
aichallenge/ml_workspace/tiny_lidar_net/dataset/precontact_residual_base_v2
```

Diagnostic checkpoint directory (generated, not committed):

```text
aichallenge/ml_workspace/tiny_lidar_net/checkpoints/residual-temporal-v1/20260901_124839
```

Training used `scan_delta`, run-balanced sampling, anchor leakage weight 0.5,
learning rate `1e-4`, and an upper bound of 30 epochs.  Early stopping ended at
10 epochs.

The validation-selected checkpoint remained the exact zero residual:

```text
best weighted validation loss: 0.0377705
material improvement:          0.0%
normal leakage MAE:             0.0 rad
```

The final diagnostic checkpoint learned both newly observed opposite-action
failure tails:

| subset | material MAE improvement | anchor MAE |
|---|---:|---:|
| new d1, final 10 s | 53.6% | 0.0792 rad |
| new d2, final 10 s | 63.5% | N/A (all material) |

However, it failed every admission gate that protects normal behavior:

```text
independent normal leakage MAE: 0.0978 rad (limit 0.01)
full validation anchor MAE:     0.0654 rad (limit 0.01)
full validation material gain:  11.7%     (minimum 30%)
historical d3 anchor tail MAE:   0.0129 rad (limit 0.01)
```

## Decision

The experiment falsifies the narrow hypothesis that one scan difference is
enough to identify the corrective action safely.  It contains useful motion
signal—the opposite hard tails became learnable—but the same signal is not
sufficient to distinguish those actions from ordinary driving without severe
steering leakage.

No temporal checkpoint was promoted and no closed-loop run was attempted.
Further work should add explicit state/history (for example recurrent scan
features and vehicle speed) or collect separable demonstrations, rather than
increase epochs or relax leakage gates.
