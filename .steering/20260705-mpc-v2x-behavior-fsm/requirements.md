# MPC V2X Behavior FSM Requirements

作成日: 2026-07-05
状態: Draft

## 目的

現行 C++ MPC の `V2XGapPlanner` を常時使うのではなく、他車状況とコース状況に応じて「使ってよい場面」と「使わず減速すべき場面」を切り替える最小の行動状態を追加する。

目的は高度な追い抜き戦略を作ることではない。まず、単独走行では安定している trajectory tracking を、多車両時に gap planner が不用意に横へ寄せて破綻させるケースを抑える。

## 現状認識

現行 C++ MPC には V2X gap planner がある。

- `/v2x/vehicle_positions` を subscribe できる。
- 他車を横方向 occupied interval に変換できる。
- free gap がある場合は `lb/ub` と `xr` を gap 側へ寄せられる。
- free gap がない場合は `no_gap_target_velocity` で速度上限を下げられる。

ただし、現状は「いつ gap planner を使うべきか」という行動判断が弱い。

- ヘアピン入口など、単独 trajectory の姿勢作りが重要な区間でも横回避が入り得る。
- 前方車に詰まっただけの場面で、追走よりも横回避が優先され得る。
- gap が毎周期変わると、MPC の横目標が揺れて操舵が不安定になる。
- 「抜く」「追走する」「単独で走る」の状態が明示されていない。

## 対象範囲

対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml`
- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/README.md`
- `docs/spec/mpc-integration.md`

対象外:

- `/v2x/vehicle_positions` の topic 名・message 型変更。
- `/control/command/control_cmd` の topic 名・message 型変更。
- 評価 FSM、Domain、AWSIM 管理、result JSON の変更。
- global trajectory の再生成。
- 学習ベース planner。
- Side-by-side の詳細戦略や、完全なレースライン最適化。
- 2026 公式仕様として未確認の追い抜きルールを確定扱いすること。

## 必要状態

初期実装では次の4状態に絞る。

| State | 役割 |
|---|---|
| `Cruise` | 周辺に対象他車がいない。通常 trajectory を走る |
| `Follow` | 前方車に詰まっている。横回避せず速度を落として追走する |
| `Overtake` | 直線寄り、低曲率、十分な gap がある場面だけ gap planner を許可する |
| `SafetyBrake` | 近すぎる、gap なし、衝突リスクが高い場合に強く減速する |

`AbortOvertake` は初期実装では独立状態にせず、`Overtake` から `Follow` または `SafetyBrake` へ戻す遷移として扱う。必要になったら独立状態へ昇格する。

## 入力分類要求

V2X 車両を自車基準または reference path 基準で分類する。

- 前方車: 自車より前方、かつ横方向距離が一定範囲内。
- 横並び車: 自車前後の近い範囲、かつ横方向距離が一定範囲内。
- 後方車: 自車より後方。
- 対象外車: stale、遠すぎる、自車近傍フィルタ対象、または corridor 外。

初期実装では、前方車の有無と距離を重視する。横並び車は `SafetyBrake` または `Follow` 側に倒すための保守的な入力として扱う。

## 追い抜き禁止要求

次の場面では `Overtake` を許可しない。

- ヘアピン入口など、事前に config した wp_id 区間。
- 曲率が大きい区間。
- 前方車との距離が近すぎる区間。
- gap 幅が不足している区間。
- V2X 情報が stale または不安定な区間。

追い抜き禁止時に前方車がいる場合は `Follow`、近すぎる場合は `SafetyBrake` へ倒す。

## MPC 反映要求

状態ごとの反映は次の通り。

| State | gap planner | 速度上限 |
|---|---:|---:|
| `Cruise` | 無効 | 通常 |
| `Follow` | 無効 | 前方車距離に応じて制限 |
| `Overtake` | 有効 | 通常または保守的に制限 |
| `SafetyBrake` | 無効 | 強く制限 |

重要な要求:

- `Follow` では gap planner を使わない。
- `SafetyBrake` では gap planner を使わない。
- `Overtake` のときだけ既存 `V2XGapPlanner` の `lb/ub` と `xr` 反映を許可する。
- `Cruise` では V2X 由来の横目標変更を入れない。

## Config 要求

少なくとも次を config 化する。

```yaml
mpc:
  use_v2x_behavior_fsm: false
  v2x_follow_distance: 8.0
  v2x_safety_brake_distance: 3.0
  v2x_follow_velocity: 5.0
  v2x_safety_brake_velocity: 0.0
  v2x_overtake_min_gap_width: 2.0
  v2x_overtake_max_curvature: 0.05
  v2x_overtake_forbidden_wp_ranges: []
  v2x_state_hold_time: 0.5
```

既定値では現行挙動を維持するため、`use_v2x_behavior_fsm` は `false` とする。

## 安全要求

- 判断できない場合は `Overtake` ではなく `Follow` または `SafetyBrake` に倒す。
- stale な V2X 情報は使わない。
- 状態遷移には最低保持時間または hysteresis を入れ、毎周期の状態揺れを避ける。
- ヘアピン入口では横回避より trajectory tracking の安定を優先する。
- 実車では使わない。シミュレータで確認済みの設定だけ扱う。

## 互換性要求

- 既定設定では現在の C++ MPC 挙動を変えない。
- `control_method=mpc` の launch 経路を維持する。
- `/v2x/vehicle_positions`、`/control/command/control_cmd`、`/localization/kinematic_state`、`/planning/scenario_planning/trajectory` の topic 契約を変えない。
- `aichallenge_system/` 側を変更しない。
- config key が未指定でも起動できる。

## 受け入れ条件

- `use_v2x_behavior_fsm=false` で現行挙動を維持する。
- 他車なしでは `Cruise` になり、通常 trajectory tracking になる。
- 前方車が近い場合は `Follow` になり、gap planner が無効化される。
- 前方車が近すぎる場合は `SafetyBrake` になり、速度上限が下がる。
- 追い抜き禁止区間では `Overtake` に入らない。
- 低曲率かつ十分な gap がある場合だけ `Overtake` になり、既存 `V2XGapPlanner` が使われる。
- 状態遷移が log で追える。
- `make autoware-build` が成功する。

## 未確定事項

- follow velocity を固定値にするか、前方車推定速度に合わせるか。
- 追い抜き禁止区間を wp_id 範囲で指定するか、曲率 threshold だけで判定するか。
- 横並び車を初期実装で `SafetyBrake` に倒すか、`Follow` に倒すか。
- `Overtake` から通常ラインへ戻る `ReturnToLine` を今回入れるか、既存 trajectory tracking に任せるか。
- 状態を debug topic として publish するか、log のみにするか。
