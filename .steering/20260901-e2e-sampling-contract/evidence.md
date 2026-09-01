# Evidence

Generated, gitignored candidate:

`checkpoints/spatial-sample-adapter-v1/20260901_165519/candidate.npy`

SHA-256:

`589307cc8e4aae577b60769d5addb31219353e3315a4ce44c6b322200339aa0a`

Gate report:

`output/20260901-e2e-spatial-sample-gate.json`

- material MAE improvement: 30.51% (pass)
- material direction accuracy: 84.51% (pass)
- peer direction: 100%, peer material improvement 62.45% (pass; 16 samples)
- teacher-validation anchor MAE: 0.01086 rad (fail by 0.00086)
- independent normal MAE: 0.01486 rad (fail)
- embedded candidate3 identity and bounds: pass

The root-cause correction restored material performance as predicted, but the
candidate remains rejected.  The remaining trade-off is structural: the same
three-class softmax owns both neutral activation and left/right homotopy.
Sequence sampling favored neutral leakage at the cost of sign generalization;
sample sampling reversed that outcome.  The next candidate must separate the
binary activation decision from the conditional left/right decision instead
of retuning sampler mass or gate thresholds.
