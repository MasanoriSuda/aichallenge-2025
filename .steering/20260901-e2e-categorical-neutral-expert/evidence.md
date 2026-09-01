# Evidence

## Candidate

- artifact: `spatial-production-wheel-base-seed2030-v14-categorical/20260901_224922/candidate.npy`
- SHA-256: `878afade36275da1e3ea6771276b563852e0dd7dfefaf10d2871103e4574666f`
- training objective: `categorical_expert`
- evaluated decode: `winner_take_all`
- all physical inputs, frozen base, representation, sample distribution and
  `+/-1.2 rad` range match the v12 comparison.

## Offline result

The exact winner-take-all audit is stored at
`output/20260901-e2e-wheel-base-seed2030-v14-categorical-audit.json` and failed.

| Metric | v11 production soft mixture | v14 categorical expert |
|---|---:|---:|
| Aggregate material improvement | 35.99% | 34.33% |
| Material sign accuracy | 89.30% | 89.73% |
| Anchor false-material rate | 5.12% | 5.78% |
| Aggregate anchor MAE | 0.005985 rad | **0.010205 rad** |
| Independent-normal MAE | 0.009699 rad | **0.015161 rad** |
| Peer material improvement | 68.53% | 50.59% |
| Focus full improvement | 37.59% | 27.88% |

The failed Gates are `anchor_leakage` and `independent_normal_leakage`.
Although neutral argmax outputs exact zero, a false left/right classification
now emits the complete learned magnitude rather than a probability-attenuated
correction.  Consequently a similar false-material classification rate causes
larger normal-track steering errors.

The focused validation tail happened to improve by 94.41%, but it contained
only two material samples and does not override the two primary leakage Gate
failures.  A separate closed-loop or failure-bag replay cannot make this
candidate admissible, so both were skipped.

## Decision

- Reject v14.
- Do not add winner-take-all decoding to production runtime.
- Keep v11 artifact and soft-mixture runtime unchanged.
- Do not sweep class weights, confidence thresholds or debounces.
- The next independent variable must add temporal evidence for stable mode
  selection; another static decoder cannot resolve this error mode.
