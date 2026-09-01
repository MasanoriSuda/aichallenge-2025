# Evidence

## Sampler contract

`--outcome-run-balanced-successor` is opt-in and mutually exclusive with the
existing sequence-balanced mode.  It requires
`metadata.outcome_certificate.source_run_id` for every sequence and fails
closed when the identity is absent or blank.

The training manifest records:

```text
mode=outcome_run_balanced
20260902-e2e-speed-committed-seed2034: 193 chunks
20260902-e2e-final-speed-committed-teacher-all-v2: 544 chunks
```

Despite unequal chunk counts, the weighted sampler assigns total probability
mass 0.5 to each certified run.  Within the four-domain peer run, empirical
chunk proportions are preserved.

## Candidate identity

Candidate directory:
`checkpoints/conv5-recurrent-final-peers-run-balanced-v1-nospeed/20260902_064408`

- training manifest SHA-256:
  `82ae6e3383c5809c8e46a645a2675b237fc1d643f2b240f16e15b04e9ab5745c`
- training summary SHA-256:
  `8338f3ebf6a976d1c2daf50f90daa167da84947072f943e717a594a25d6e42a7`
- checkpoint SHA-256:
  `d61c5b60ad8541dbff3e513144edcf200cb894329ea2483260e6791280f38319`

Manifest comparison against the natural-sampling 512-unit candidate found
only the sampling option and output identity.  Model configuration is exactly
equal.

Training result:

- best combined validation loss: 0.0016747032
- best successor validation loss: 0.0014727430
- best normal validation loss: 0.0000403921
- early stop after 20 epochs

## Evaluation

- seed2033 report:
  `output/20260902-e2e-peer-run-balanced-seed2033-gate.json`
  (SHA-256 `f5ecc7b76e9bc1f504187c8a77d87a5e433f1406cdba318ae45c9701368e065f`)
- seed2035 report:
  `output/20260902-e2e-peer-run-balanced-seed2035-gate.json`
  (SHA-256 `f196d14f4e4e3f774f0ccbfaedcd21dd3ef4afd529b12e3ef9fb417ad9e31ab2`)

All existing absolute gates pass, but the direct old/new acceptance boundary
does not:

| Dataset / metric | Previous | Natural 512 | Run-balanced 512 |
|---|---:|---:|---:|
| seed2033 all | 0.014210 | 0.012157 | 0.012355 |
| seed2033 material | 0.055790 | 0.056196 | 0.056634 |
| seed2033 anchor | 0.005930 | 0.003387 | 0.003537 |
| seed2035 all | 0.013821 | 0.012009 | 0.012337 |
| seed2035 material | 0.054414 | 0.054019 | 0.055027 |
| seed2035 anchor | 0.005416 | 0.003311 | 0.003498 |
| independent normal correction | 0.001966 | 0.001076 | 0.001099 |

Relative to the previous admitted candidate, material MAE regresses 1.51% on
seed2033 and 1.13% on seed2035.  Relative to natural 512 sampling, it regresses
on both worlds as well.

## Verification and decision

- focused recurrent/sampling tests: 32 passed
- complete TinyLidarNet workspace suite: 226 passed
- Python byte-code compilation: passed
- `git diff --check`: passed

Reject the model and do not convert or connect it.  Keep the sampler as an
explicit, provenance-safe research option; its default is off.  The evidence
does not support further post-hoc sampling-ratio tuning in this Slice.
Runtime ROS topics, launch files, packaged checkpoints and authority are
unchanged.
