# MPC V2X Behavior FSM Design

作成日: 2026-07-05
状態: Draft

## 方針

既存 `V2XGapPlanner` は、横方向 gap を作って MPC の `lb/ub` と `xr` に反映する低レベル機能として残す。

今回追加する層は、その上に置く小さな behavior FSM とする。FSM は「今 gap planner を使ってよいか」「速度をどこまで落とすか」だけを決める。

初期実装の目的は、追い抜き性能を最大化することではなく、多車両時に横回避が入って単独走行で安定していたラインを壊すケースを止めることである。

## 現行構造

### `V2XGapPlanner`

現行 C++ MPC 内部に `V2XGapPlanner` がある。

責務:

- V2X 他車位置を保持する。
- 直近2点から速度を推定する。
- horizon 上の他車予測位置を作る。
- reference path に対する lateral occupied interval を作る。
- base corridor `[lb, ub]` から occupied interval を差し引く。
- free gap を選び、`lb/ub` と `target_ey` を返す。

この構造はそのまま使う。

### MPC 反映

`MPC::init_problem()` で base `lb/ub` を作った後、gap planner output が active なら制約を狭める。

```text
base lb/ub
  -> V2XGapPlanner
  -> dynamic lb/ub
  -> xr target_ey
  -> OSQP
```

今回の FSM は、この呼び出しの前段に入り、状態に応じて gap planner output を使うか無視するかを決める。

## 追加コンポーネント

### `V2XBehaviorConfig`

`MpcConfig` に追加する。

```cpp
struct V2XBehaviorConfig
{
  bool enabled{false};
  double follow_distance{8.0};
  double safety_brake_distance{3.0};
  double follow_velocity{5.0};
  double safety_brake_velocity{0.0};
  double overtake_min_gap_width{2.0};
  double overtake_max_curvature{0.05};
  double state_hold_time{0.5};
  std::vector<std::pair<int, int>> overtake_forbidden_wp_ranges;
};
```

### `V2XBehaviorState`

```cpp
enum class V2XBehaviorState
{
  Cruise,
  Follow,
  Overtake,
  SafetyBrake,
};
```

### `V2XBehaviorOutput`

```cpp
struct V2XBehaviorOutput
{
  V2XBehaviorState state{V2XBehaviorState::Cruise};
  bool allow_gap_planner{false};
  double target_velocity_limit{std::numeric_limits<double>::infinity()};
};
```

## 入力

FSM は次を入力にする。

- 現在 wp_id。
- 自車速度。
- 自車 pose。
- reference path の曲率。
- V2X 有効他車リスト。
- 既存 gap planner が見ている base corridor 情報。

初期実装では、既存 `V2XGapPlanner` 内部の tracked vehicles を再利用できる形にする。必要なら、planner に「分類用 snapshot」を返す軽い accessor を追加する。

## 他車分類

reference waypoint の yaw を使い、自車から見た longitudinal / lateral を計算する。

```text
dx = vehicle_x - ego_x
dy = vehicle_y - ego_y
longitudinal = cos(ego_yaw) * dx + sin(ego_yaw) * dy
lateral = -sin(ego_yaw) * dx + cos(ego_yaw) * dy
```

初期分類:

- `front_vehicle`: `0 < longitudinal < follow_distance` かつ `abs(lateral)` が一定範囲内。
- `danger_vehicle`: `0 < longitudinal < safety_brake_distance` かつ `abs(lateral)` が一定範囲内。
- `side_vehicle`: `abs(longitudinal)` が小さく `abs(lateral)` が一定範囲内。

`danger_vehicle` がある場合は `SafetyBrake` を優先する。

## 追い抜き禁止判定

`Overtake` を禁止する条件:

```text
wp_id が forbidden range 内
or abs(kappa) > overtake_max_curvature
or danger_vehicle あり
or V2X が stale
or gap 幅が overtake_min_gap_width 未満
```

wp_id range は config で複数指定できる形にする。

初期 YAML 表現案:

```yaml
mpc:
  v2x_overtake_forbidden_wp_ranges:
    - [120, 170]
    - [280, 330]
```

## 状態遷移

基本遷移:

```text
Cruise
  -> Follow       前方車あり、追い抜き禁止または gap 不足
  -> Overtake     前方車あり、追い抜き許可、gap 十分
  -> SafetyBrake  danger_vehicle あり

Follow
  -> Cruise       前方車なし
  -> Overtake     追い抜き許可、gap 十分
  -> SafetyBrake  danger_vehicle あり

Overtake
  -> Cruise       前方車なし
  -> Follow       追い抜き禁止になった、または gap 不足
  -> SafetyBrake  danger_vehicle あり

SafetyBrake
  -> Follow       danger_vehicle なし、前方車あり
  -> Cruise       danger_vehicle なし、前方車なし
```

状態チャタリングを避けるため、`state_hold_time` 未満では原則として状態を維持する。ただし `SafetyBrake` への遷移は即時許可する。

## MPC への反映

`MPC::init_problem()` の流れを次のようにする。

```text
1. base lb/ub を作る
2. V2XBehaviorFSM を評価する
3. allow_gap_planner=true のときだけ V2XGapPlanner を呼ぶ
4. allow_gap_planner=false のときは gap planner output を使わない
5. target_velocity_limit が有限なら umax_dyn / ur に反映する
6. xr は既存 center_bias / trajectory target に従う
```

状態ごとの処理:

| State | `allow_gap_planner` | velocity limit |
|---|---:|---:|
| `Cruise` | false | none |
| `Follow` | false | `follow_velocity` |
| `Overtake` | true | none or optional |
| `SafetyBrake` | false | `safety_brake_velocity` |

## Logging

状態遷移時だけ info log を出す。

例:

```text
V2X behavior: Cruise -> Follow, front_distance=6.2, wp_id=142
V2X behavior: Follow -> SafetyBrake, front_distance=2.1, wp_id=145
```

毎周期 log は出さない。必要なら debug throttle にする。

## Config

`config.yaml` には既定無効で追加する。

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

`use_v2x_behavior_fsm=false` のときは、既存の `use_v2x_gap_planner` 設定に従う。`true` のときは、FSM が `V2XGapPlanner` の使用可否を上書きする。

## 実装順

1. Config と enum / output struct を追加する。
2. V2X tracked vehicles の snapshot を取得できるようにする。
3. 自車基準の前方車 / danger vehicle 分類を追加する。
4. wp_id range と曲率による overtake 禁止判定を追加する。
5. FSM の状態遷移と hysteresis を追加する。
6. `MPC::init_problem()` で gap planner 呼び出し可否と速度制限を反映する。
7. 状態遷移 log を追加する。
8. docs と README を更新する。

## 検証

### Static / Build

- `git diff --check`
- `make autoware-build`

### Runtime

最低限:

- `use_v2x_behavior_fsm=false` で既存挙動を維持する。
- `use_v2x_behavior_fsm=true` かつ他車なしで `Cruise` になる。
- V2X position editor で前方車を置き、`Follow` になる。
- 前方近距離に置き、`SafetyBrake` になる。
- 追い抜き禁止 wp range 内で `Overtake` にならない。
- 低曲率区間で gap が十分ある場合のみ `Overtake` になる。

### Rosbag / Result

- `/control/command/control_cmd` の publish 周期を維持する。
- ヘアピン入口で横目標が急に変わらないことを確認する。
- wall / collision penalty を確認する。

## リスク

- follow velocity が低すぎると lap time が悪化する。
- follow velocity が高すぎると前方車との距離を詰めすぎる。
- wp_id range 指定が trajectory 差し替えでずれる。
- 曲率 threshold だけではヘアピン入口の早めの判断に足りない可能性がある。
- gap planner を抑制しすぎると追い抜き機会を失う。

## ロールバック

`use_v2x_behavior_fsm: false` に戻す。

さらに V2X gap planner も止める場合は `use_v2x_gap_planner: false` に戻す。
