# MPC V2X Gap Planner Tasklist

作成日: 2026-07-04
状態: Draft

## Definition of Done

- 既定設定では現行 C++ MPC の挙動が変わらない。
- `/v2x/vehicle_positions` を C++ 側で読み、最新他車情報として保持できる。
- 他車を horizon 上の動的障害物として近似できる。
- コース境界 `lb/ub` から他車占有 interval を差し引き、free gap を生成できる。
- 通過可能 gap がある場合、MPC の横方向制約または目標に反映できる。
- 通過可能 gap がない場合、減速または停止へ倒せる。
- `/control/command/control_cmd` の topic 契約を維持する。
- `make autoware-build` が成功する。
- シミュレータで動作確認し、追い越し・すり抜けを有効化する場合のリスクを記録する。

## Phase 0: 現状確認

- [x] 現行 C++ MPC にコース境界 `lb/ub` 制約があることを確認する。
- [x] `/v2x/vehicle_positions` の subscribe 入口があることを確認する。
- [x] C++ 側 V2X callback が空で、制約に反映されていないことを確認する。
- [x] Python 側に V2X tracker / obstacle manager / path constraints provider の材料があることを確認する。
- [x] 現状はすり抜け・追い越し planner が有効ではないと整理する。

## Phase 1: Steering / 設計

- [x] requirements を作成する。
- [x] design を作成する。
- [x] tasklist を作成する。
- [x] 実装前に `/v2x/vehicle_positions` の message field を確認する。
- [ ] 実装前に評価シナリオ上、追い越しが必要かを確認する。

## Phase 2: Config

- [x] `MpcConfig` に `use_v2x_gap_planner` を追加する。
- [x] `MpcConfig` に `v2x_vehicle_radius` を追加する。
- [x] `MpcConfig` に `v2x_prediction_margin` を追加する。
- [x] `MpcConfig` に `v2x_prediction_time` を追加する。
- [x] `MpcConfig` に `v2x_timeout_sec` を追加する。
- [x] `MpcConfig` に `gap_min_width` を追加する。
- [x] `MpcConfig` に `gap_target_bias` を追加する。
- [x] `MpcConfig` に `no_gap_target_velocity` を追加する。
- [x] key 未指定時の fallback を実装する。
- [x] `config.yaml` に既定 false の設定を追加する。

## Phase 3: V2X 入力

- [x] C++ 側 `/v2x/vehicle_positions` callback を実装する。
- [x] 自車 ID または自車近傍情報を除外する方針を決める。
- [x] 最新他車 list と timestamp を保持する。
- [x] stale な V2X 情報を破棄する。
- [ ] 受信数と有効他車数を debug log で確認できるようにする。

## Phase 4: 予測と座標変換

- [x] 他車を円近似 obstacle として扱う。
- [x] 速度が取れる場合の等速直線予測を実装する。
- [x] 速度が取れない場合の静止障害物扱いを実装する。
- [x] reference path 最近傍 waypoint を使った横方向 offset 計算を実装する。
- [x] horizon step と obstacle の対応付けを実装する。

## Phase 5: Gap 生成

- [x] base corridor `[lb, ub]` を取得する。
- [x] obstacle occupied interval を作る。
- [x] occupied interval を sort / merge する。
- [x] base corridor から occupied interval を差し引く。
- [x] `gap_min_width` 未満の interval を破棄する。
- [x] 2台間 gap と左右外側 gap を候補として扱えることを確認する。

## Phase 6: Gap 選択

- [x] 現在横位置に近い候補を優先する。
- [x] reference trajectory に近い候補を優先する。
- [x] gap 幅が広い候補を優先する。
- [ ] horizon 全体で継続する候補を優先する。
- [x] 候補切り替え時の急操舵を抑える hysteresis を検討する。

## Phase 7: MPC 反映

- [x] 選択 gap を `lb/ub` に反映する。
- [x] 必要なら `xr` を gap 中央へ寄せる。
- [x] infeasible になった場合の fallback を確認する。
- [x] no-gap 時に速度目標を下げる方法を決める。
- [x] no-gap 時の停止または減速を実装する。

## Phase 8: Build Verification

- [x] `make autoware-build` を実行する。
- [x] `use_v2x_gap_planner=false` で既存挙動を確認する。
- [ ] `use_v2x_gap_planner=true` で node が起動することを確認する。
- [ ] `/control/command/control_cmd` が publish されることを確認する。

## Phase 9: Runtime Verification

- [ ] 他車なしで通常 trajectory 追従になることを確認する。
- [ ] 前方1台で左右 gap が生成されることを確認する。
- [ ] 前方2台で中央 gap が生成されることを確認する。
- [ ] gap がない場合に減速または停止することを確認する。
- [ ] gap 候補切り替えで操舵が振動しないことを確認する。
- [ ] wall / crash / over penalty を確認する。
- [ ] lap time と安全性の tradeoff を記録する。

## Phase 10: Documentation

- [x] `README.md` に設定値と使い方を追記する。
- [x] `docs/spec/mpc-integration.md` に V2X gap planner の位置づけを追記する。
- [x] 2026 公式仕様未確認の内容は暫定と明記する。
- [x] 実車では未使用またはシミュレータ確認必須であることを明記する。

## 検証メモ

実装後に以下を記録する。

```text
use_v2x_gap_planner:
v2x_vehicle_radius:
v2x_prediction_margin:
gap_min_width:
trajectory:
scenario:
result:
notes:
```

## 実装メモ: 2026-07-04

- C++ 内部に `V2XGapPlanner` を追加した。
- `/v2x/vehicle_positions` は config の `use_v2x_gap_planner: true` または launch の `use_obstacle_avoidance=true` で subscribe する。
- V2X message に速度と yaw がないため、vehicle_id ごとの直近2点から速度を推定する。初回、異常ジャンプ、過大速度時は静止障害物として扱う。
- 他車は円近似し、reference path の horizon waypoint ごとに lateral occupied interval へ変換する。
- base corridor `[lb, ub]` から occupied interval を差し引き、`gap_min_width` 以上の free interval を選択する。
- 選択 gap は `lb/ub` と `xr` に反映する。
- 通過可能 gap がない場合は `no_gap_target_velocity` を速度上限として使う。
- 既定 `use_v2x_gap_planner: false` のため、通常設定では現行 trajectory tracking のまま。

## 検証メモ: 2026-07-04

```text
use_v2x_gap_planner: false
v2x_vehicle_radius: 1.25
v2x_prediction_margin: 0.2
gap_min_width: 1.8
trajectory: env/final_ver3/traj_mincurv.csv
scenario: build + short node startup
result:
  - make autoware-build succeeded.
  - ros2 run mpc_controller_cpp loaded config and reached odometry wait.
  - short startup used timeout 5s, so exit code 124 was expected.
notes:
  - Runtime gap behavior with V2X vehicles is not verified yet.
  - /control/command/control_cmd was not checked because odometry/trajectory runtime was not running.
```
