# Evidence

## Inputs

- teacher dataset: `recurrent_direct_v6_wheel_speed_seed2030`
- current normal dataset: `production_normal_anchor_v3_wheel_speed_current`
- frozen base SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- sampling: sample-proportional
- deterministic seeds: 2026, 2027, 2028

Generated reports are
`output/20260901-e2e-current-temporal-separability-seed{2026,2027,2028}.json`.

## Three-seed means

| Representation | Balanced accuracy | Material sign | Normal false-material | Focus sign | Focus-tail sign |
|---|---:|---:|---:|---:|---:|
| static conv5 + speed | 0.8888 | 0.8889 | 0.0866 | 0.8897 | 1.0000 |
| static conv5 + speed + base | **0.8991** | **0.9046** | 0.0896 | **0.9089** | **1.0000** |
| temporal conv5 + speed | 0.8601 | 0.8497 | 0.0873 | 0.8249 | 0.8333 |
| temporal conv5 + speed + base | 0.8737 | 0.8721 | **0.0850** | 0.8687 | 0.6667 |

For the unconditioned spatial pair, temporal-minus-static was negative for all
three seeds:

- balanced accuracy: -0.0206, -0.0209, -0.0446;
- material-sign accuracy: -0.0322, -0.0331, -0.0522.

All variants classified the peer sequence's 16 material samples with 100%
direction accuracy, so this small unilateral subset does not distinguish the
architectures.

## Decision

The existing causal lag representation does not separate no-intervention from
correction and consistently harms the focused failure sequence.  Do not train
or integrate another short-history or compact recurrent adapter from this
evidence.

The static base-conditioned spatial representation remains the strongest
classifier, while its normal false-material rate remains about nine percent.
The next root-cause question is whether the teacher corpus and zero-normal
corpus contain observationally equivalent states with contradictory desired
actions.  That label/observability audit precedes any new model architecture.
