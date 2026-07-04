# MPC V2X Gap Planner Design

作成日: 2026-07-04
状態: Draft

## 方針

既存 C++ MPC の外部インターフェースを壊さず、V2X 他車位置から動的 corridor を作る小さな planner 層を追加する。

初期実装は高度な行動計画ではなく、rule-based gap selector とする。目的は「前方の他車を見て、通れる横方向 interval を選び、MPC に渡す」ことである。

## 現行構造

### コース境界制約

`ReferencePath::update_simple_path_constraints()` は、reference path の `lb/ub` から horizon ごとの横方向制約を作る。

この制約はコース境界に対して効くが、他車を差し引いた corridor にはなっていない。

### V2X 入力

C++ 側は `use_obstacle_avoidance=true` のときだけ次を subscribe する。

```cpp
"/path_constraints_provider/path_constraints"
"/path_constraints_provider/border_cells"
"/v2x/vehicle_positions"
```

ただし現状 callback は空であり、受信内容は MPC 問題へ反映されない。

### Python 側の参考実装

Python 側には次の材料がある。

- `v2x_vehicle_tracker.py`
- `predictions_to_obstacles()`
- `obstacle_manager.py`
- `path_constraints_provider.py`

ただし現在の通常起動は C++ MPC であり、これらはそのままでは有効なすり抜け機能になっていない。

## 追加コンポーネント案

### `V2XGapPlanner`

C++ 内部に小さな class として追加する。

責務:

- 最新 V2X vehicle list を保持する。
- stale な他車を破棄する。
- 自車周辺の前方他車だけ抽出する。
- horizon ごとに他車占有 interval を作る。
- 通過可能 gap を選ぶ。
- MPC 用の `lb/ub` と任意の target offset を返す。

想定 interface:

```cpp
struct GapPlannerConfig
{
  bool enabled{false};
  double vehicle_radius{1.25};
  double prediction_margin{0.2};
  double prediction_time{3.0};
  double timeout_sec{1.0};
  double min_gap_width{1.8};
  double target_bias{1.0};
  double no_gap_target_velocity{0.0};
};

struct GapPlannerOutput
{
  bool active{false};
  bool feasible{true};
  std::vector<double> lb;
  std::vector<double> ub;
  std::vector<double> target_ey;
  double target_velocity_scale{1.0};
};
```

## 処理フロー

### 1. V2X callback

`/v2x/vehicle_positions` を受けたら、最新値を planner に渡す。

```text
V2XVehiclePositionArray
  -> tracked vehicles
  -> timestamp update
```

この callback では重い計算をしない。保持だけにする。

### 2. MPC update 時に gap 計算

MPC の各周期で、現在の自車状態と reference path を使って gap を計算する。

```text
current state
reference path
base lb/ub
tracked vehicles
  -> predicted obstacles
  -> occupied lateral intervals
  -> free intervals
  -> selected gap
  -> dynamic lb/ub + target_ey
```

### 3. 他車予測

初期実装:

- 速度が取れる場合は等速直線。
- 速度が取れない場合は静止障害物扱い。
- yaw が信頼できない場合は円近似にする。

各 horizon step `i` の時刻:

```cpp
double t = i * Ts;
```

予測位置:

```cpp
x_pred = x + vx * t;
y_pred = y + vy * t;
```

### 4. Path 座標変換

他車位置を reference path の最近傍 waypoint に投影し、横方向 offset `ey_obs` を求める。

初期実装では厳密な Frenet 投影ではなく、既存の nearest waypoint と path yaw を使う。

```text
dx = obs_x - ref_x
dy = obs_y - ref_y
ey = -sin(yaw) * dx + cos(yaw) * dy
```

`s` は waypoint index として近似してよい。

### 5. Occupied interval

他車を円近似する。

```text
occupied = [ey_obs - radius, ey_obs + radius]
radius = v2x_vehicle_radius + v2x_prediction_margin
```

同じ horizon step に複数台いる場合は occupied interval を sort して merge する。

### 6. Free interval

base corridor `[lb, ub]` から occupied intervals を引く。

例:

```text
base:      [-3.0, 3.0]
obstacle1: [-1.2, 0.2]
obstacle2: [ 1.0, 2.2]
free:      [-3.0,-1.2], [0.2,1.0], [2.2,3.0]
```

`gap_min_width` 未満の free interval は破棄する。

### 7. Gap 選択

候補 interval に score を付ける。

例:

```text
score =
  w_current * distance_from_current_ey
  + w_reference * distance_from_reference_ey
  - w_width * interval_width
  + w_switch * lane_switch_penalty
```

初期実装では重みを固定値にしてよい。必要になったら config 化する。

選んだ gap の中央を target とする。

```cpp
target_ey = 0.5 * (gap_lb + gap_ub);
```

### 8. MPC への反映

`MPC::init_problem()` で base `lb/ub` を取得した後、planner output が active なら上書きまたは交差を取る。

```cpp
lb[i] = std::max(lb[i], gap_lb[i]);
ub[i] = std::min(ub[i], gap_ub[i]);
```

`xr` は gap target に寄せる。

```cpp
const double base_target = cfg.center_bias * center_ey;
const double gap_target = planner_output.target_ey[i];
xr[nx + i * nx] =
  (1.0 - gap_target_bias) * base_target + gap_target_bias * gap_target;
```

## No-Gap 時の扱い

通過可能 gap がない場合、次のどちらかを選ぶ。

初期実装では安全側として速度目標を落とす。

```text
target_velocity_scale = 0.0
```

ただし現在の MPC が horizon 上の速度 reference をどこで更新しているかを確認し、無理に topic 契約を変えない。

## Config 設計

`mpc` 配下に追加する。

```yaml
mpc:
  use_v2x_gap_planner: false
  v2x_vehicle_radius: 1.25
  v2x_prediction_margin: 0.2
  v2x_prediction_time: 3.0
  v2x_timeout_sec: 1.0
  gap_min_width: 1.8
  gap_target_bias: 1.0
  no_gap_target_velocity: 0.0
```

`use_v2x_gap_planner=false` なら既存挙動を維持する。

## 実装順

1. config と構造体だけ追加し、既定 false で挙動を変えない。
2. V2X callback で受信数と stale 管理だけ実装する。
3. 他車を円 obstacle として horizon に投影する。
4. base `lb/ub` から free interval を作る。
5. 最も単純な gap 選択を実装する。
6. `lb/ub` へ反映する。
7. 必要なら `xr` へ target offset を反映する。
8. no-gap 時の減速を追加する。

## リスク

- V2X 座標系や timestamp の扱いを誤ると、存在しない障害物を避ける。
- obstacle を円近似すると、高速域や斜め姿勢で余裕不足になる。
- `safety_margin_scale=0.0` と併用すると、壁と他車の両方に対する余裕が不足しやすい。
- gap selector が頻繁に候補を切り替えると操舵が不安定になる。
- no-gap 減速が遅いと追突しやすくなる。
- 評価シナリオ上、追い越しが有利とは限らない。

## ロールバック

- `use_v2x_gap_planner: false`
- launch の `use_obstacle_avoidance` を `false`

この2点で通常の trajectory 追従 MPC に戻せる設計にする。
