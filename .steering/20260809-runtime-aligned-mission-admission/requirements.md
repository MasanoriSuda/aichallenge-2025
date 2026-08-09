# Requirements

## 目的

- 初回採用時の Pass 距離・side 評価と、実行時の Pass horizon / SafeSeparation の距離契約を一致させる。
- 「入口では外側だったが、rear-clear 前に内側へ変わる」候補を左右比較で早期に不利と判定する。
- clearance、横加速度、絶対 Pass 上限は緩和しない。

## 対象

- `v2x_overtake_core` の動的 Pass 距離と runtime continuation reserve。
- `mpc_controller_cpp` の初回 Mission candidate 生成・rear-clear course-role 評価。
- pure unit test と走行ログの Mission diagnostics。

## 対象外

- SafeSeparation の hard guard 緩和。
- no-return 距離の変更。
- 接触・Recovery FSM の変更。

