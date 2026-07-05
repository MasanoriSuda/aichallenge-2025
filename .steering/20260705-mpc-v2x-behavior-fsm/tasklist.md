# MPC V2X Behavior FSM Tasklist

作成日: 2026-07-05
状態: Draft

## Definition of Done

- 既定設定では現行 C++ MPC の挙動が変わらない。
- 他車なしでは通常 trajectory tracking を維持する。
- 前方車が近い場合は gap planner を無効化して追走側へ倒せる。
- 危険距離では gap planner を無効化して強制減速できる。
- 追い抜き禁止区間では `Overtake` に入らない。
- 追い抜き許可条件を満たしたときだけ既存 `V2XGapPlanner` を使える。
- 状態遷移が log で追える。
- `/control/command/control_cmd` の topic 契約を維持する。
- `make autoware-build` が成功する。

## Phase 0: 現状確認

- [x] 単独走行が安定している前提を整理する。
- [x] 現行 C++ MPC に `V2XGapPlanner` があることを確認する。
- [x] 現行機能は高レベル追い抜き戦略ではなく、横方向 gap selection であると整理する。
- [x] 今回の主目的を「抜きに行ってはいけない場面で gap planner を抑制すること」に絞る。

## Phase 1: Steering / 設計

- [x] requirements を作成する。
- [x] design を作成する。
- [x] tasklist を作成する。
- [x] 実装前に現在の `config.yaml` の V2X 関連値を記録する。
- [ ] 実装前にヘアピン入口の wp_id 範囲候補を確認する。

## Phase 2: Config

- [x] `MpcConfig` に `use_v2x_behavior_fsm` を追加する。
- [x] `MpcConfig` に `v2x_follow_distance` を追加する。
- [x] `MpcConfig` に `v2x_safety_brake_distance` を追加する。
- [x] `MpcConfig` に `v2x_follow_velocity` を追加する。
- [x] `MpcConfig` に `v2x_safety_brake_velocity` を追加する。
- [x] `MpcConfig` に `v2x_overtake_min_gap_width` を追加する。
- [x] `MpcConfig` に `v2x_overtake_max_curvature` を追加する。
- [x] `MpcConfig` に `v2x_overtake_forbidden_wp_ranges` を追加する。
- [x] `MpcConfig` に `v2x_state_hold_time` を追加する。
- [x] key 未指定時の fallback を実装する。
- [x] `config.yaml` に既定無効の設定を追加する。

## Phase 3: V2X Snapshot

- [x] `V2XGapPlanner` の tracked vehicles を FSM から参照できる形にする。
- [x] stale vehicle を除外する。
- [x] 自車近傍 vehicle を除外する。
- [x] 分類に必要な位置、速度、timestamp を snapshot として返す。

## Phase 4: Vehicle Classification

- [x] 自車 pose または reference path yaw を使って longitudinal / lateral を計算する。
- [x] `front_vehicle` を判定する。
- [x] `danger_vehicle` を判定する。
- [x] `side_vehicle` を判定する。
- [x] 判定距離と lateral range を config または定数で管理する。

## Phase 5: Overtake Permission

- [x] wp_id が forbidden range 内か判定する。
- [x] 曲率が `v2x_overtake_max_curvature` を超えるか判定する。
- [x] gap 幅が `v2x_overtake_min_gap_width` 以上か判定する。
- [x] stale / 不安定 V2X では `Overtake` を禁止する。
- [ ] 禁止理由を debug log で追えるようにする。

## Phase 6: FSM

- [x] `V2XBehaviorState` enum を追加する。
- [x] `V2XBehaviorOutput` を追加する。
- [x] `Cruise` 遷移を実装する。
- [x] `Follow` 遷移を実装する。
- [x] `Overtake` 遷移を実装する。
- [x] `SafetyBrake` 遷移を実装する。
- [x] `state_hold_time` による hysteresis を実装する。
- [x] `SafetyBrake` への即時遷移を許可する。
- [x] 状態遷移 log を追加する。

## Phase 7: MPC Integration

- [x] `use_v2x_behavior_fsm=false` では既存 `use_v2x_gap_planner` の挙動を維持する。
- [x] `Cruise` では gap planner output を使わない。
- [x] `Follow` では gap planner output を使わず、速度上限を下げる。
- [x] `Overtake` では既存 `V2XGapPlanner` を呼び出す。
- [x] `SafetyBrake` では gap planner output を使わず、速度上限を強く下げる。
- [x] 速度上限を `umax_dyn` と `ur` に反映する。
- [ ] infeasible fallback が既存通り動くことを確認する。

## Phase 8: Build Verification

- [x] `git diff --check` を実行する。
- [x] `make autoware-build` を実行する。
- [ ] `use_v2x_behavior_fsm=false` で build 後起動できることを確認する。

## Phase 9: Runtime Verification

- [ ] 他車なしで `Cruise` になることを確認する。
- [ ] 前方車ありで `Follow` になることを確認する。
- [ ] 前方近距離で `SafetyBrake` になることを確認する。
- [ ] 追い抜き禁止 wp range 内で `Overtake` にならないことを確認する。
- [ ] 低曲率かつ十分な gap がある場面で `Overtake` になることを確認する。
- [ ] ヘアピン入口で横目標が急変しないことを確認する。
- [ ] `/control/command/control_cmd` の publish を確認する。
- [ ] wall / collision penalty を確認する。

## Phase 10: Documentation

- [x] `README.md` に V2X behavior FSM の設定と使い方を追記する。
- [x] `docs/spec/mpc-integration.md` に FSM の位置づけを追記する。
- [x] 2026 公式仕様未確認の内容は暫定と明記する。
- [x] 実車では未使用またはシミュレータ確認必須であることを明記する。

## 検証メモ

実装後に以下を記録する。

```text
use_v2x_behavior_fsm:
use_v2x_gap_planner:
v2x_follow_distance:
v2x_safety_brake_distance:
v2x_follow_velocity:
v2x_overtake_forbidden_wp_ranges:
trajectory:
scenario:
result:
notes:
```

## 実装メモ: 2026-07-05

- 現在の `config.yaml` は `use_v2x_gap_planner: true`、`v2x_vehicle_radius: 1.25`、`v2x_prediction_margin: 0.1`、`gap_min_width: 1.8`。
- FSM は既定 `use_v2x_behavior_fsm: false` で追加した。
- FSM 有効時は `Overtake` のときだけ gap planner を使う。
- `Follow` と `SafetyBrake` は `umax_dyn` と `ur` に速度上限を入れる。
- `git diff --check` succeeded.
- `make autoware-build` succeeded.
