# E2E Imitation Baseline Evidence

## Teacher data

独立したMPC teacher runをrun単位で分離した。

| split | run | samples | result |
|---|---|---:|---|
| train | `output/20260901-024545` | 3439 | AWSIM Finish |
| val | `output/20260901-025352` | 2767 | AWSIM Finish |

scan/control同期rejectは両runとも0。平均同期差8.08 ms、p95 12.17 ms、最大16.50 ms。

## Training and offline validation

- run: `checkpoints/20260901_025820`
- warm-start SHA-256: `7a3f2702fe652a14970710aefde775d5328105d1a5370d4abf1fc88611043963`
- best PyTorch SHA-256: `b23c95fec09f05205ce253b87532cb63153f82aa169073fd52e6807a54c1d1d0`
- runtime candidate SHA-256: `8ebd93687b05b17cc20741fa34c7e78bec05d380993ddd6ff75ddb6e49d3af08`
- best validation loss: 0.00540581 (epoch 25)
- PyTorch / NumPy runtime parity: 64 samples、最大steering差 `1.49e-7`

独立validation run上のsteering指標:

| weights | MAE [rad] | RMSE [rad] | p95 abs [rad] | sign mismatch |
|---|---:|---:|---:|---:|
| previous production | 0.12079 | 0.15210 | 0.31814 | 14.02% |
| candidate | 0.07746 | 0.10475 | 0.24258 | 7.95% |

## Closed-loop acceptance

scenarioは固定start、1 vehicle、0 NPC、3 laps、fixed acceleration 0.6 m/s^2。

| weights/run | inferred laps [s] | stopped | result |
|---|---|---:|---|
| previous production `20260901-021032` | 111.74 / 90.81 / failure | 0.0 s | 2.24 laps、wall/stuck |
| candidate `20260901-030400` | 109.90 / 89.80 / 88.77 | 0.0 s | AWSIM Finish |
| candidate `20260901-031218` | 109.55 / 88.99 / 89.40 | 0.0 s | AWSIM Finish |

candidateは2/2 runで3周完走し、各lapのrun間差は1秒未満。cross-track p99は旧重み
3.77 mからcandidate 2.63--2.69 mへ低下した。steering 0.63 rad以上は
0.033--0.034%で、常時飽和ではない。

## Decision

単車3周baselineとしてcandidateをproductionへ昇格する。速度は約89秒/周でMPC teacherの
約41秒/周より大幅に遅いため、次Sliceは速度調整ではなくNPCを含む観測・教師data拡張とする。
