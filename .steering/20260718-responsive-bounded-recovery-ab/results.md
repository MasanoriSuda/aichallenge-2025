# Results

## Run 1: `output/20260718-232114`

加速度0.7 m/s2、速度上限1.0 m/s、強制再試行なしで実験した。

- D1: 10 step、1.737 mで`escape_step_limit_reached`。
- D2: 10 step、1.725 mで`escape_step_limit_reached`。姿勢は`e_psi=0.980 -> -0.011 rad`、
  横偏差は`e_y=1.494 -> 0.763 m`まで改善した。
- D3: 10 step、1.284 mで`escape_step_limit_reached`。姿勢差が大きいままwallへ接近した。
- `rejoin_complete`: 全Domain 0回。
- `aggressive_retry` / forced rejoin: 全Domain 0回。

結論: Fail。高い加速度が停止距離reserveを増やして各stepを短くした。ただしD2は2.0 m固定
targetより前に安全なrejoin姿勢へ到達しており、厳格なaligned early rejoin候補の根拠になった。

## Run 2

### `output/20260718-233001`

元の加速度0.5 m/s2へ戻し、static sweepを維持するaligned early rejoinを検証した。

- 約125秒までは3台とも走行を継続し、1周以上進んだ。
- D1は0.896 m、`e_y=1.691 m`、`e_psi=-0.034 rad`で早期rejoinへ入った。
  しかし5秒以内に横偏差が収束せずrejoin timeoutし、最終的にstep上限で停止した。
- D2は横偏差が2.0 m条件を外れ、1.395 m・10 stepで停止した。
- D3は大姿勢差のまま短いForward rejoinがtimeoutし、最後は`invalid_grid`による
  `maneuver_direction_unknown`で停止した。
- `rejoin_complete`: 全Domain 0回。

結論: Fail。早期遷移の静的gateは機能したが、単一点のpath feedbackでは大きな横偏差を
5秒以内に収束できない。実験機能はソースと有効設定から除去し、元の成功設定へ戻した。

## Final decision

- 採用: 元のbounded stepwise Recovery、positive Reverse acceleration、0.5 m/s2、0.8 m/s、
  rejoin 1.0 m/s、heading gain 1.20。
- 不採用: 高加速度化、積極再試行、forced rejoin、aligned early rejoin。
- 次の有望案: ReverseとForwardを同じ複数点経路で最適化するRecovery専用planner/MPC。

## Verification

- Run 1前の元Recovery source: 17 targets、413 tests、0 failures。
- early rejoin候補: 25 packages build成功、17 targets、415 tests、0 failures。
- 失敗候補除去後の最終source: `make autoware-build` 25 packages succeeded。
- 最終source/config: `git diff --check`成功、`aligned_early`実装残存なし。
