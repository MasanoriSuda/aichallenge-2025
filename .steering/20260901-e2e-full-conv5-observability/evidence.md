# Evidence

## Observability control

Report: `output/20260901-e2e-full-conv5-observability.json`.

| Representation | Material inside normal p50 / p95 | Normal inside material p50 / p95 | Four-peer d2 tail p50 / p95 |
|---|---:|---:|---:|
| projected v11 input | 8.26% / 29.23% | 7.76% / 44.38% | 60% / 100% |
| full conv5 + speed + base | 5.97% / 25.03% | 5.81% / 75.39% | 61% / 100% |
| physical binned geometry | 3.24% / 19.83% | 3.17% / 28.42% | 0% / 100% |

Removing the projection reduces aggregate strong conflict, but it does not
recover the safety-critical distinction in `/output/20260901-121938/d2`.
Normal-to-material p95 overlap also becomes much worse.  The full representation
used 1,088 conv5 values plus speed and base steering; 60 dimensions had no
variation in the sampled admitted normal corpus and were scale-floored.

## Three-seed classifier control

Reports:
`output/20260901-e2e-full-conv5-separability-seed{2026,2027,2028}.json`.

| Seed | Representation | Balanced | Material sign | Normal false material | Focus sign | Focus tail | Peer sign |
|---:|---|---:|---:|---:|---:|---:|---:|
| 2026 | projected + base | 0.89362 | 0.89904 | 0.09010 | 0.91007 | 1.00 | 1.00 |
| 2026 | full conv5 + base | 0.88919 | 0.89121 | 0.08976 | 0.91906 | 1.00 | 1.00 |
| 2027 | projected + base | 0.90384 | 0.91123 | 0.09684 | 0.92266 | 1.00 | 1.00 |
| 2027 | full conv5 + base | 0.90198 | 0.90078 | 0.08689 | 0.91547 | 1.00 | 1.00 |
| 2028 | projected + base | 0.89974 | 0.90339 | 0.08183 | 0.89388 | 1.00 | 1.00 |
| 2028 | full conv5 + base | 0.88993 | 0.88947 | 0.08740 | 0.91547 | 1.00 | 1.00 |
| mean | projected + base | **0.89906** | **0.90455** | 0.08959 | 0.90887 | 1.00 | 1.00 |
| mean | full conv5 + base | 0.89370 | 0.89382 | **0.08802** | **0.91667** | 1.00 | 1.00 |

Full conv5 loses balanced accuracy and material direction in every seed.  Its
small normal/focus improvements do not compensate for the aggregate regression
and it does not fix the known four-peer tail.

## Decision

Reject a full-map version of v11.  Do not train a new authority or change the
runtime artifact.  The fixed projection is a contributor to aggregate overlap,
but the upstream frozen base conv5 itself lacks the physical distinction needed
by the failure tail.

The next bounded experiment is a diagnostic trainable 1D convolutional action
probe on physical LiDAR, speed and base steering.  It tests whether a
correction-specific local geometry encoder can outperform both the frozen base
features and the already rejected raw/geometry MLPs.  Only a three-seed strict
improvement can justify an offline residual candidate; otherwise the static
input/label contract is the limiting factor and richer causal observation must
be reconsidered.
