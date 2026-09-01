# Evidence

## Dataset

Generated, gitignored root:

`aichallenge/ml_workspace/tiny_lidar_net/dataset/normal_anchor_recurrent_v1`

- 3 train sequences, 1 validation sequence, 19,714 samples
- train speed: mean 3.1283 m/s, p95 4.4169 m/s, max 4.7898 m/s
- validation speed: mean 3.1899 m/s, p95 4.2417 m/s, max 4.5394 m/s
- maximum synchronization delta: 36.12 ms (contract: at most 50 ms)
- stored target semantics: frozen production correction equals zero
- source steering arrays are not copied and cannot become teacher labels

## Three-seed separability audit

Reports:

- `output/20260901-e2e-normal-speed-separability-seed2026.json`
- `output/20260901-e2e-normal-speed-separability-seed2027.json`
- `output/20260901-e2e-normal-speed-separability-seed2028.json`

| representation | balanced accuracy | material sign | normal false material |
|---|---:|---:|---:|
| static conv5, no speed | 0.8630 | 0.8599 | 0.1417 |
| static conv5 + real speed | 0.8602 | 0.8529 | 0.1218 |
| temporal conv5, no speed | 0.8257 | 0.8109 | 0.1405 |
| temporal conv5 + real speed | 0.8226 | 0.8053 | 0.1197 |

Real speed reduces normal false activation by about two percentage points but
does not improve aggregate or material-direction accuracy.  Short LiDAR
history remains worse than the static spatial representation.

## Offline candidate gate

Generated, gitignored candidate:

`checkpoints/spatial-speed-adapter-v1/20260901_164415/candidate.npy`

SHA-256:

`81ff8d70546f706ec98082e0f670a1ab75bf253a51303d20c1abffd6bb0aa670`

Gate report:

`output/20260901-e2e-spatial-speed-adapter-gate.json`

- independent normal MAE: 0.00873 rad (pass, threshold 0.01)
- validation anchor MAE: 0.00608 rad (pass)
- material MAE improvement: 27.82% (fail, threshold 30%)
- material direction accuracy: 77.28% (fail, threshold 80%)
- peer material direction: 87.5% (pass, only 16 material samples)
- embedded candidate3 identity: pass

The candidate is rejected and is not connected to runtime.  The next bounded
hypothesis is the mismatch between the probe's train-feature standardization
and the continuous adapter's per-sample LayerNorm, not a ROS speed subscriber
or a threshold change.
