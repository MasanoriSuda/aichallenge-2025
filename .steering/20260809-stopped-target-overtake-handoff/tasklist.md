# Tasklist

- [x] 最新ログと現行 ownership 条件を照合する
- [x] 安全な commit 済み停止車引継ぎ判定を core に追加する
- [x] committed side 固定で停止車 local path を評価する
- [x] 引継ぎ理由をログへ追加する
- [x] core 単体テストを追加・更新する
- [x] `multi_purpose_mpc_ros` をビルドする
- [x] 動的確認項目を記録する

## Definition of Done

- commit 済み同一 target が停止し、現在非重複かつ同一 side 経路が成立する場合に
  LowSpeedAvoidance へ移行できる。
- 現在重複、target 不連続、同一 side 経路不成立では引継がない。
- 新規停止車回避の既存 3 m 最小準備距離は維持される。
- 単体テストとパッケージビルドが成功する。

## 実走確認

- `low_confirm=3/3` 後に `low_speed=1` となること
- `stopped vehicle bypass owns target` が出ること
- 同じ event で SafetyBrake / Recovery / Reverse へ落ちないこと
- 引継ぎ前後で pass side が変わらないこと
- wall stop、local path infeasible の場合は従来どおり停止側へ戻ること

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core --gtest_filter=V2XOvertakeCoreLowSpeedBypass.*`:
  13 / 13 成功
- `test_v2x_overtake_core`: 436 / 436 成功
- 最終 `mpc_controller_cpp` target build: 成功
