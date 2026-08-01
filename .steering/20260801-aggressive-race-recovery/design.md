# Design

## Policy switch

`aggressive_force_motion_enabled` を追加する。以下を全て満たす場合だけ強制復帰を有効にする。

- `simulation_only: true`
- `aggressive_sim_recovery_enabled: true`
- `aggressive_force_motion_enabled: true`

## Supervisor

- `SafeStop` の同一snapshot待機を無効化し、`aggressive_retry_delay_sec` 後に再試行する。
- clearance timeout専用の恒久待機より、強制再試行を優先する。
- gear reportの一時欠損・timeout・invalidは停止後に再試行する。
- 非finite入力、時刻逆行、odometry/control喪失は移動計算不能なので既存hard stopを維持する。

## Candidate selection

- 既存の安全候補を最優先する。
- 候補が全滅した場合だけ、6方向の短距離rolloutから least-bad 候補を選ぶ。
- 評価順は、入力妥当性、接触増加、最終接触数、経路横偏差改善、衝突までの距離とする。
- `OutOfMap` や無効rolloutは強制候補にしない。

## V2X

- V2X rollout結果とblocker IDはログへ残す。
- 強制復帰中はV2X overlap/incompleteを移動禁止には使わない。
- Boost停止確認は従来どおり維持する。

## Speed

- 加速度上限は既存MPCの `a_max=1.0 m/s^2` を維持する。
- 前進escapeとRejoinの速度上限をReverseと同じ2.0 m/sへ揃える。
- 短い再合流後に再度stuckした場合、Reverse目標を最大8.0 mまで倍増する。
- 8.0 m目標の停止余裕として、独立hard capを9.0 m、時間上限を8.0秒とする。

## Verification

- core unit testsで、同一snapshotからの周期再試行とclearance timeout非終端化を確認する。
- candidate/V2X overrideはビルドと既存footprint testsで回帰確認する。
- 実走では `forced_candidate`、`v2x_override`、SafeStop滞在時間、rejoin完了を確認する。
