# V2X Overtake / Recovery Line Design

作成日: 2026-07-06
状態: Implemented / runtime verification pending

## 方針

Pro との壁打ちで出た理想形は、`race_state_estimator -> track_projector -> race_strategy_planner -> trajectory_selector -> velocity_planner -> controller` である。

ただし現段階で一気に分離すると、既存 MPC/V2X の調整結果を壊しやすい。最初の実装では `mpc_controller_cpp.cpp` の内部に薄い overtake line planner を追加し、次の役割だけを担わせる。

- `Overtake` に入った後の横目標列を滑らかに作る。
- 追い越し側を lock する。
- 抜き切ったら基準 trajectory へ戻す。
- 危険なら Recovery へ倒す。

これは将来の `trajectory_selector` へ切り出せる形にする。

## 状態設計

既存:

```cpp
enum class V2XBehaviorState
{
  Cruise,
  Follow,
  Overtake,
  LowSpeedAvoidance,
  SafetyBrake,
};
```

追加案:

```cpp
enum class OvertakeLinePhase
{
  Idle,
  FollowPrepare,
  ShiftOut,
  Pass,
  Return,
  Recovery,
};
```

`V2XBehaviorState::Overtake` は大分類として残す。`OvertakeLinePhase` は横ライン生成専用の内部状態とする。

## データ構造

追加案:

```cpp
struct OvertakeLineConfig
{
  bool enabled{false};
  double shift_distance{8.0};
  double pass_distance{8.0};
  double return_distance{10.0};
  double lateral_offset{1.2};
  double target_bias{0.8};
  double min_wall_clearance{0.8};
  double max_lateral_accel{2.5};
  double max_target_change{0.25};
  double return_clear_distance{4.0};
  double phase_hold_time{0.3};
  bool debug_log_enabled{false};
};

struct OvertakeLineState
{
  OvertakeLinePhase phase{OvertakeLinePhase::Idle};
  int pass_side_sign{0};
  double target_ey{0.0};
  double phase_start_sec{0.0};
  double phase_start_ey{0.0};
  double locked_front_distance{infinity};
};
```

最初は `MpcProblem` のメンバとして持つ。後で別クラス化する。

## 入力

既存の `V2XBehaviorOutput` を利用する。

- `state`
- `front_distance`
- `front_speed`
- `front_risk_level`
- `overtake_pass_side_sign`
- `overtake_side_clearance`
- `overtake_gap_available`
- `overtake_forbidden`
- `overtake_start_curve_blocked`
- `has_side_vehicle`

不足する場合は、最低限の追加フィールドだけ `V2XBehaviorOutput` に増やす。

## フェーズ遷移

### Idle -> FollowPrepare

条件:

- `V2XBehaviorState::Follow`
- 前方車両あり
- side clearance が十分
- overtake forbidden ではない
- front risk が emergency ではない

動作:

- 既存 `follow_preposition` と統合する。
- 横目標は小さく、制約は狭めない。

### FollowPrepare -> ShiftOut

条件:

- `V2XBehaviorState::Overtake`
- reachable gap あり
- pass side が決まっている
- front distance が最小値以上
- curve guard に引っかからない

動作:

- pass side を lock。
- `phase_start_ey` を現在 `e_y` に設定。
- 横目標を `lateral_offset * pass_side_sign` に向けて ramp。

### ShiftOut -> Pass

条件:

- 横目標の 80% 以上に到達した。
- または shift_distance 相当の horizon 進行を消化した。

動作:

- pass side の横オフセットを維持。
- 前走車横を通過中は戻らない。

### Pass -> Return

条件:

- 対象前走車が後方へ `return_clear_distance` 以上抜けた。
- 復帰側に side vehicle がいない。
- overtake forbidden / curve guard が復帰を危険にしない。

動作:

- `target_ey` を base trajectory 側へ ramp。
- pass side lock は Return 完了まで維持。

### 任意 -> Recovery

条件:

- front risk emergency。
- `target_ey` が `lb/ub` 端へ張り付く。
- required lateral accel が最大値を超える。
- side vehicle が復帰先または pass side に割り込む。
- overtake gap が連続して失われる。

動作:

- 新規 Overtake を一定時間抑止。
- 速度上限を下げる。
- base trajectory または corridor center へ緩やかに戻る。

## 横目標生成

MPC horizon `i=0..N-1` について、距離方向の進捗 `p` を作る。

```text
p = clamp(s_i / phase_distance, 0, 1)
smooth = p*p*(3 - 2*p)
target_ey_i = start_ey + smooth * (goal_ey - start_ey)
```

候補:

- `ShiftOut`: `goal_ey = pass_side_sign * lateral_offset`
- `Pass`: `goal_ey = pass_side_sign * lateral_offset`
- `Return`: `goal_ey = 0.0` または `center_bias` 適用後の基準値
- `Recovery`: `goal_ey = safe_center_ey`

`target_ey_i` は `lb[i] + min_wall_clearance` / `ub[i] - min_wall_clearance` の範囲へ clip する。

最終反映:

```cpp
xr_e_y = (1.0 - target_bias) * xr_e_y + target_bias * target_ey_i;
```

## 横加速度ガード

各 horizon で、おおまかに次を評価する。

```text
t = max(s_i / max(v, min_speed), min_time)
required_lat_accel = 2 * abs(target_ey_i - current_ey) / (t * t)
```

`required_lat_accel > max_lateral_accel` の場合:

- ShiftOut 開始前なら Overtake に入らない。
- ShiftOut / Pass 中なら Recovery へ倒す。

## 既存機能との関係

### front risk arbitration

- EmergencyBrake は常に優先。
- BrakePrepare / AvoidCandidate の速度制限は維持。
- reachable gap の判定結果を OvertakeLine の開始条件に使う。

### LowSpeedAvoidance

- 停止車両列の gate2 は既存 local path planner を優先する。
- `OvertakeLinePhase` は原則 `LowSpeedAvoidance` には適用しない。
- gate2 が悪化する場合は、停止車両向けだけ従来経路を使う。

### Follow preposition

- `FollowPrepare` は既存 `v2x_follow_preposition_*` を置き換える候補。
- 初期実装では既存 preposition を残し、OvertakeLine は `Overtake` 以降に限定してもよい。

## Debug / 可視化

最低限ログ:

```text
OvertakeLine: Idle -> ShiftOut, side=right, target=-1.2, fd=8.4
OvertakeLine: ShiftOut -> Pass, ey=-0.9, target=-1.2
OvertakeLine: Pass -> Return, clear=5.1
OvertakeLine: Pass -> Recovery, reason=wall clearance
```

周期 debug:

- phase
- pass side
- target_ey
- current_ey
- front_distance
- side_clearance
- required_lateral_accel
- block/recovery reason

RViz marker は後続 Phase でよい。まずログで判断可能にする。

## Config 初期値

```yaml
mpc:
  v2x_overtake_line_enabled: false
  v2x_overtake_line_shift_distance: 8.0
  v2x_overtake_line_pass_distance: 8.0
  v2x_overtake_line_return_distance: 10.0
  v2x_overtake_line_lateral_offset: 1.2
  v2x_overtake_line_target_bias: 0.8
  v2x_overtake_line_min_wall_clearance: 0.8
  v2x_overtake_line_max_lateral_accel: 2.5
  v2x_overtake_line_max_target_change: 0.25
  v2x_overtake_line_return_clear_distance: 4.0
  v2x_overtake_line_phase_hold_time: 0.3
  v2x_overtake_line_debug_log_enabled: true
```

## 実装順

1. Config とデータ構造だけ追加する。
2. `Overtake` 中の横目標生成を既存 ramp から OvertakeLine に置き換える。
3. `Pass` / `Return` のフェーズ遷移を入れる。
4. Recovery は最小限、壁端張り付きと emergency のみから始める。
5. gate2 / dev2 / dev3 で確認する。

## リスク

- 横オフセットを強くしすぎると gate2 やヘアピンで壁側へ寄る。
- 復帰が早すぎると前走車へ戻り接触する。
- 復帰が遅すぎると外側に残って壁へ寄る。
- `Follow` preposition と OvertakeLine の target が競合する可能性がある。
- 既存 `gap_target_bias` と二重に効く可能性がある。

## 将来切り出し候補

この実装が安定したら、次の steering に分ける。

- `v2x-track-projector`: s/d 座標で他車と自車を扱う。
- `v2x-race-state-estimator`: lap、section、boost、penalty、opponents を統合する。
- `v2x-boost-velocity-planner`: boost と penalty を速度計画へ入れる。
- `v2x-defense-strategy`: 後方車両と防御ライン。
