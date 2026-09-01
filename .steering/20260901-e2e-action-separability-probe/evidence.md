# Evidence

## Immutable inputs

- dataset: generated `dataset/recurrent_direct_v3`
- train: 9 sequences / 29304 samples
- validation: 4 sequences / 12313 samples
- peer `20260901-153143/d3`: validation only
- frozen candidate3 SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- material correction threshold: 0.02 rad

The generated reports remain outside git at
`output/20260901-e2e-action-separability-probe-seed{2026,2027,2028}.json`.

## Three-seed result

| representation | mean balanced accuracy | mean material-sign accuracy | interpretation |
|---|---:|---:|---|
| frozen `fc3` + speed | 0.5245 | 0.4656 | compact policy embedding collapses corrective geometry |
| frozen spatial `conv5` + speed | 0.8779 | 0.8654 | corrective direction is strongly separable |
| frozen spatial `conv5` + short deltas + speed | 0.8420 | 0.8227 | history deltas do not improve the spatial feature |

Across seeds, spatial `conv5` improves balanced accuracy over compact `fc3` by
0.3534 and material-sign accuracy by 0.3998.  Temporal deltas reduce those
metrics relative to static spatial input by 0.0359 and 0.0426 respectively.
This ordering is identical in all three seeds.

The peer-d3 validation sequence contains only 16 material samples and all are
right corrections.  The static spatial probe classifies all 16 correctly in
every seed, with neutral false-material fractions from 0.0069 to 0.0183.  This
is positive evidence but not bilateral peer-d3 proof and must not be inflated
into a closed-loop claim.

## Decision

Do not train another compact recurrent adapter and do not add temporal runtime
state based on this experiment.  The supported next bounded experiment is a
frozen-base, static spatial adapter whose head sees the full frozen `conv5`
map and speed.  It must pass continuous residual magnitude, normal-anchor and
seed-disjoint gates before shadow or closed-loop use.  Production candidate3
and `fixed_lidar_brake` remain unchanged.

## Verification

```text
focused action-probe tests       4 passed
TinyLidarNet full tests        106 passed
three deterministic CUDA probes seeds 2026/2027/2028 completed
```
