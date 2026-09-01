# Evidence

## Immutable split

- correction corpus: `recurrent_direct_v3`
- neutral train source: `dagger_aggregate_v2/train`, 3 sequences
- unseen neutral validation: `dagger_aggregate_v2/val`, 6130 samples
- frozen candidate3 SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- generated reports:
  `output/20260901-e2e-spatial-normal-separability-seed{2026,2027,2028}.json`

## Valid LiDAR-only comparison

| representation | mean material-sign accuracy | unseen normal false-material | mean balanced accuracy |
|---|---:|---:|---:|
| static full spatial | 85.06% | 14.56% | 85.60% |
| spatial + 1/8-step LiDAR history | 80.42% | 13.76% | 82.32% |

Static spatial correction remains learnable, but too many independent normal
states are classified as corrective.  Short LiDAR history reduces that error by
only 0.80 percentage points while losing 4.64 points of corrective sign
accuracy.  The same ordering is stable across all three seeds.  Peer d3 keeps
100% direction recall, but still has only 16 right-correction samples.

Speed-control variants are not admission evidence in this report.  The old
normal dataset does not store synchronized speed, so those variants receive an
invented zero speed while the teacher dataset receives real speed.  Their lower
normal false-material rate could therefore be trivial source discrimination.

## Classification

The current static LiDAR and short-history representations do not cleanly
separate normal and corrective states.  This supports an observation/data
overlap, not permission to tune the continuous loss.  Before adding a runtime
input, derive an immutable normal recurrent dataset by synchronizing speed from
the original bags, preserve train/validation identity, and repeat the probe
with real speed on both sides.

No candidate checkpoint was created and production remains unchanged.

## Verification

```text
focused action-probe tests       6 passed
TinyLidarNet full tests        112 passed
three deterministic CUDA probes completed
```
