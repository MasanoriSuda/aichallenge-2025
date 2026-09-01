# Evidence

Reports:
`output/20260901-e2e-correction-cnn-separability-seed{2026,2027,2028}.json`.

The correction-specific model used four trainable 1D convolutional layers on
the normalized physical 750-beam scan, adaptive angular pooling, and a small
head conditioned on synchronized wheel speed and frozen-base steering.  It was
diagnostic only and produced no runtime artifact.

## Three-seed result

| Seed | Representation | Balanced | Material sign | Normal false material | Focus sign | Focus tail | Peer sign |
|---:|---|---:|---:|---:|---:|---:|---:|
| 2026 | projected frozen + base | 0.89362 | 0.89904 | 0.09010 | 0.91007 | 1.00 | 1.00 |
| 2026 | trainable scan CNN + base | 0.84341 | 0.84073 | 0.11439 | 0.83633 | 1.00 | 1.00 |
| 2027 | projected frozen + base | 0.90384 | 0.91123 | 0.09684 | 0.92266 | 1.00 | 1.00 |
| 2027 | trainable scan CNN + base | 0.82352 | 0.84943 | 0.13936 | 0.86511 | 0.00 | 1.00 |
| 2028 | projected frozen + base | 0.89974 | 0.90339 | 0.08183 | 0.89388 | 1.00 | 1.00 |
| 2028 | trainable scan CNN + base | 0.83629 | 0.83377 | 0.13008 | 0.82194 | 0.00 | 1.00 |
| mean | projected frozen + base | **0.89906** | **0.90455** | **0.08959** | **0.90887** | **1.00** | 1.00 |
| mean | trainable scan CNN + base | 0.83441 | 0.84131 | 0.12795 | 0.84113 | 0.33 | 1.00 |

The learned local encoder regressed every aggregate Gate, increased normal
false-material actions by 3.84 percentage points, and failed the focus-tail
direction in two seeds.  Early stopping completed after 14--20 epochs, so the
result is not a single unfinished optimization run.

## Decision

Reject a static correction-specific CNN and do not train a steering authority
from it.  Frozen projected features, full frozen conv5, binned geometry, raw
MLP and a learned local CNN have now all failed to resolve the same
normal/material ambiguity.  Architecture width or projection tuning is no
longer the next independent variable.

The remaining justified question is causal observability: whether a proper
spatiotemporal geometry encoder can distinguish a stationary wall/normal curve
from a moving or newly intruding obstacle.  This must account for sequence
boundaries and ego motion, use the same held-out runs, and first remain a
diagnostic classifier.  If it also fails three seeds, the current teacher/zero-
normal label contract is not learnable from the allowed LiDAR/speed observation
and must be redesigned rather than tuned.
