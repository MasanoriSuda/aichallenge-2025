# Evidence

Generated, gitignored candidate:

`checkpoints/spatial-speed-fixednorm-v1/20260901_164834/candidate.npy`

SHA-256:

`54aa48eeca3064f572ba772864ac47c5fb75ce750804432cfe615d47ff33b677`

Gate report:

`output/20260901-e2e-spatial-speed-fixednorm-gate.json`

- independent normal MAE: 0.00927 rad (pass)
- validation anchor MAE: 0.00887 rad (pass)
- material MAE improvement: 18.88% (fail, threshold 30%)
- material direction accuracy: 71.28% (fail, threshold 80%)
- peer material direction: 100% (16 right-only material samples)
- embedded candidate3 identity: pass

Fixed population standardization did not reproduce the diagnostic probe and
made aggregate material performance worse than per-sample LayerNorm.  The
hypothesis is rejected.  Production candidate3 remains unchanged.  The probe
also uses a frozen 128-dimensional random projection, so representation
capacity/regularization remains a separate bounded hypothesis.
