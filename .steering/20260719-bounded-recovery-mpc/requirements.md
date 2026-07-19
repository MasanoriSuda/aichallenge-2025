# Requirements

## Purpose

dev3のシミュレーション予選で、従来のbounded Recoveryが退避後の大きな横偏差を
LowSpeedRejoinの単一点P制御だけでは5秒以内に収束できない問題を改善する。

## Scope

- `multi_purpose_mpc_ros`内にsimulation-onlyのRecovery専用有限ホライズン制御を追加する。
- 前進rejoinは各制御周期で再計画する。
- 後退・短距離前進は、既存primitive群のうちRecovery MPCの第一操舵に近い安全候補を選ぶ。
- `make autoware-build`、package test、`make dev3`で比較する。

## Constraints

- 通常走行のOSQP MPC、ROS topic/service/message、Domain契約は変更しない。
- 既存のFSM、距離・時間・速度・step・attempt上限を変更しない。
- static swept-footprint、V2X、gear report、Boost inactiveの各gateを迂回しない。
- MPCが計算不能または安全候補なしの場合は、従来のP制御／primitiveへ戻す。
- 実車へは適用しない。

## Acceptance

- 純粋関数のRecovery MPCがforward/reverseの符号付き運動をunit testで区別できる。
- 既存testに回帰がない。
- dev3ログにRecovery MPCの計画・選択が記録される。
- `rejoin_complete`へ到達し、その後に通常MPC走行が継続する車両が増える。
- 改善しない場合は設定の`recovery_mpc.enabled: false`だけで従来方式へ戻せる。
