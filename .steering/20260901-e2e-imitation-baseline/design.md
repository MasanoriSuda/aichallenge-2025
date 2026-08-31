# E2E Imitation Baseline Design

## Warm Start

runtimeのNumPy parameter名（例:`conv1_weight`）をPyTorch state key
（例:`conv1.weight`）へarchitectureのexpected stateから対応付ける。missing/unexpected key、
shape、numeric dtype、finiteを全parameterで検証してからstrict loadする。

## Training

- architecture: existing TinyLidarNet, 750 inputs, 2 outputs
- input: normalized LiDAR
- target: `[acceleration, steering]`
- loss: acceleration weight 0、steering weight 1
- optimizer: Adam fine-tune
- split: independent bag/run
- seed: fixed and recorded

既存weightを直接更新しない。timestamp run directoryへbest/last/manifestを保存し、bestだけを
runtime NumPy候補へ変換する。

## Promotion

candidateはoffline validation lossだけで昇格しない。既存weightとの同一入力比較、PyTorchと
NumPy runtimeのparity、`e2e-single` 3周closed-loopを順に確認する。失敗時はproduction
weightを保持する。
