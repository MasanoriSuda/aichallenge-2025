# Evidence

## Candidate

- generated artifact (excluded from git):
  `checkpoints/spatial-adapter-v1/20260901_162253/candidate.npy`
- candidate SHA-256:
  `545049deec05f719285d2b98b99775739c0d574627eb0afb03010b505d941cd7`
- embedded candidate3 SHA-256:
  `de5f156b271e292a7457d6c474de1267c0a0cf086c428ae5e6f8de4c5a0f4faa`
- best epoch: 17 of 24
- generated Gate report:
  `output/20260901-e2e-static-spatial-adapter-gate.json`

## Offline result

| Gate | result | evidence |
|---|---|---|
| embedded base identity | pass | every tensor bit-identical |
| finite / bounded | pass | maximum absolute correction 0.7521 rad |
| validation material improvement | pass | 36.68% (minimum 30%) |
| validation material direction | pass | 83.20% (minimum 80%) |
| validation anchor leakage | pass | 0.00655 rad (maximum 0.01) |
| peer-d3 direction | pass | 16/16 right corrections |
| peer-d3 material improvement | diagnostic | 56.63% |
| peer-d3 anchor leakage | pass | 0.00056 rad |
| independent normal leakage | **fail** | 0.01939 rad (maximum 0.01) |

The independent normal failure is material: p95 absolute correction is 0.1353
rad and maximum is 0.5989 rad.  Relaxing the 0.01 rad Gate would hide a normal
track-following regression.  The candidate is therefore rejected and receives
no shadow or runtime authority.

## Root-cause decision

The full spatial feature resolves the compact representation bottleneck, but
the corrective training corpus does not explicitly admit the independent
production normal states as zero-residual anchors.  The next Slice may add
train-split normal candidate3 states with exact zero correction while keeping
the independent normal validation sequence untouched.  It may not change the
Gate, production checkpoint, teacher validation split, or runtime path.

## Verification

```text
focused spatial/probe tests       7 passed
TinyLidarNet full tests         109 passed
offline training                  completed, deterministic seed 2026
offline Gate                      fail (independent normal leakage only)
```
