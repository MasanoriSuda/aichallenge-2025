# Evidence

Generated, gitignored candidate:

`checkpoints/spatial-projection-adapter-v1/20260901_165035/candidate.npy`

SHA-256:

`d69970d67e9862880c59fa435d079b9cd83d7d1c0a121144bfac3c8e26c9fdf7`

Gate report:

`output/20260901-e2e-spatial-projection-gate.json`

- independent normal MAE: 0.00702 rad (pass)
- validation anchor MAE: 0.00769 rad (pass)
- material MAE improvement: 21.47% (fail)
- material direction accuracy: 72.41% (fail)
- embedded candidate3 identity: pass

The projection did not fix the continuous candidate.  To isolate the remaining
difference, the same classifier probe was run with production-equivalent
sequence-balanced sampling:

- sample sampling, spatial + speed: material sign 85.29%, normal false 12.18%
- sequence sampling, spatial + speed: material sign 70.32%, normal false 7.17%

Reports are `output/20260901-e2e-sequence-balanced-separability-seed{2026,2027,2028}.json`.
The classifier falls to the same range as the continuous candidate before any
magnitude loss is involved.  The active root cause is therefore the sampler
contract, not projection or multi-task gradient interference.  Three short
teacher runs (722, 936 and 937 samples) receive the same total probability as
4,000-6,000-sample runs under the current sampler.
