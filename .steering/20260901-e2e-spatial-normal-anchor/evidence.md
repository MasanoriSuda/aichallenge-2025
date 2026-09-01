# Evidence

## Candidate

- generated artifact:
  `checkpoints/spatial-adapter-normal-v1/20260901_162703/candidate.npy`
- SHA-256:
  `acb942a88b812065575a30b520421e4ba7963ffa2e57bca764e178cc577a9bce`
- best epoch: 9 of 16
- generated Gate report:
  `output/20260901-e2e-spatial-normal-anchor-gate.json`

The model, loss, optimizer and Gate thresholds are identical to the unanchored
candidate.  Three `dagger_aggregate_v2/train` sequences were added as exact
zero-residual anchors through the provenance wrapper.

## Result

| metric | unanchored | normal-anchored | Gate |
|---|---:|---:|---|
| independent normal MAE | 0.01939 | **0.00992** | pass <= 0.01 |
| validation material improvement | 36.68% | **26.44%** | fail >= 30% |
| validation material direction | 83.20% | **75.81%** | fail >= 80% |
| validation anchor MAE | 0.00655 | 0.00688 | pass |
| peer-d3 material improvement | 56.63% | 44.08% | diagnostic |
| peer-d3 direction | 100% | 100% | pass, 16 right samples only |

The data-contract change repaired exactly the independent-normal leakage, but
the same candidate lost corrective recall.  Lowering either Gate or tuning a
loss weight would only choose one side of the trade-off without proving that
normal and corrective observations are separable.

## Decision

Reject the candidate; no shadow or runtime authority.  Before another steering
model, run a diagnostic classifier with train-only normal anchors and untouched
normal/teacher validation.  If even the classifier cannot retain teacher
direction while rejecting normal activation, the next input/data must resolve
observation overlap.  If it can, the failure is in continuous head/loss design.

## Verification

```text
focused spatial adapter tests    4 passed
TinyLidarNet full tests         110 passed
offline Gate                     fail (material improvement and direction)
```
