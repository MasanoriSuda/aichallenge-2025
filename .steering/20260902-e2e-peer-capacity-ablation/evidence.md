# Evidence

## Candidate identity

Candidate directory:
`checkpoints/conv5-recurrent-final-peers-capacity512-v1-nospeed/20260902_064016`

- training manifest SHA-256:
  `2a8a23860f6ac162b9007dfc5438eb645dc01884e1248278193e0ebb277b88bc`
- training summary SHA-256:
  `0da3d39d1f090d3f2422154c3ae3e7d0bf805a254b6242f4bb829e86da8fdf48`
- checkpoint SHA-256:
  `10297e9484537d3c63f014050a25162e989f4edc3f7f5359af6a2c0501180e57`

Programmatic manifest comparison against the valid 64-unit peer candidate
found only:

- `hidden_dim`: 64 to 512;
- the expected output-root identity change.

No loss, optimizer, dataset, sampling, frozen artifact, decode, speed-input or
normal-anchor setting changed.

## Training result

- best combined validation loss: 0.0017148644
- best successor validation loss: 0.0014912143
- best normal validation loss: 0.0000447300
- early stop after 28 epochs

This is better than the 64-unit peer candidate's combined validation loss of
0.0021364702, supporting the capacity-limitation hypothesis.

## Evaluation

Both absolute evaluation reports pass every existing gate:

- seed2033: `output/20260902-e2e-peer-capacity512-seed2033-gate.json`
  (SHA-256 `ff39a12c5a32c8698b5c207c0ecb2c8d03855ef9e95ac59558c6792750c43afa`)
- seed2035: `output/20260902-e2e-peer-capacity512-seed2035-gate.json`
  (SHA-256 `b1f0e479b9fa13ce9eaf49bb200d6f2436b626245503c2f5593b0605abc10d76`)

Direct comparison with the previous admitted recurrent candidate:

| Dataset / metric | Previous | Capacity 512 | Change |
|---|---:|---:|---:|
| seed2033 all | 0.014210 | 0.012157 | -14.45% |
| seed2033 material | 0.055790 | 0.056196 | +0.73% |
| seed2033 anchor | 0.005930 | 0.003387 | -42.89% |
| seed2035 all | 0.013821 | 0.012009 | -13.11% |
| seed2035 material | 0.054414 | 0.054019 | -0.73% |
| seed2035 anchor | 0.005416 | 0.003311 | -38.87% |
| independent normal correction | 0.001966 | 0.001076 | -45.30% |

The larger model learns anchor/no-action behavior substantially better, but
does not consistently improve the material correction cases that motivate
peer augmentation.  Seed2033 material is slightly worse while seed2035 is
slightly better.

## Decision

Do not convert or connect this checkpoint.  The experiment establishes that
64-unit capacity contributes to the regression, but capacity alone does not
produce robust interaction improvement.  A following Slice should address
the corpus objective or source weighting explicitly, instead of silently
tuning this candidate or granting it runtime authority.

Runtime ROS topics, launch files, packaged checkpoints and authority remain
unchanged.
