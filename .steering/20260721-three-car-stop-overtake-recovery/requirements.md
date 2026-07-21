# Requirements

## 目的

`output/20260721-184659`で再現した3車停止と、
`chatgpt-point-out-v2.md`で指摘された追突後の復帰不能を同時に修正する。

## 要件

- `maneuver_direction_unknown`、`escape_not_confirmed`など、再評価可能な
  Supervisor `SAFE_STOP`をシミュレーションレース中に永久ラッチしない。
- `clearance_wait_timed_out`は、回廊が塞がれている間は停止を維持し、相手が移動して
  連続してclearになった場合だけ復帰する。
- 車体と再合流回廊がclearなら、停止誤差を考慮した小さな距離許容内でescape完了を認める。
- 追い越しラインの最低横分離目標と、前車をgeneric front brakeから外す横分離を分ける。
- 横移動中の接近速度を落とし、車体が十分横へ出るまで前車速度制限を維持する。
- シミュレーションで衝突証拠、自車停止、前進意図が揃う場合は、SafetyBrake中でも
  stuck detectorが無条件に`deliberate_stop`除外されないようにする。
- 移動車両でも、選択した復帰rollout全体で分離が改善する場合は粗い矩形回廊だけで拒否しない。
- topic、service、message型、Domain、評価JSONの契約は変更しない。

## Definition of Done

- SAFE_STOPの再試行、clearance待機、escape距離許容を単体テストする。
- 追い越しライン目標とfront brake除外閾値が独立して設定できる。
- 衝突後のdeliberate stop上書きをsimulation-only条件込みで単体テストする。
- moving V2X obstacleにもrollout clearanceを適用するテストを追加する。
- `multi_purpose_mpc_ros`の対象テストとビルドが通る。

