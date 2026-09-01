# Evidence

Generated, gitignored candidate:

`checkpoints/spatial-factorized-adapter-v1/20260901_165807/candidate.npy`

SHA-256:

`fc6b5f4ef08ba14b62e616b4843865a775f6850c7de3bf3ec1764d9a04c84980`

Gate report:

`output/20260901-e2e-spatial-factorized-gate.json`

- material direction accuracy: 84.07% (pass)
- teacher-validation anchor MAE: 0.00988 rad (pass)
- material MAE improvement: 26.16% (fail)
- independent normal MAE: 0.01355 rad (fail)
- peer direction and anchor leakage: pass
- embedded candidate3 identity and output bounds: pass

Factorization moved teacher-anchor leakage below threshold without destroying
direction classification, but it did not resolve independent normal leakage or
continuous magnitude quality.  The candidate is rejected.  Normal zero targets
and corrective teacher targets must now be audited for observation-conditioned
label consistency before another model or loss variant is attempted.
