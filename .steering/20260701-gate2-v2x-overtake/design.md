# Gate2 V2X Overtake Design

## Summary

Gate2 は V2X で見える前方 NPC を、縦方向速度制限だけでなく横方向 behavior へ分岐させる。

Gate1 の `V2XStopPlanner` は安全停止の最後の砦として残す。Gate2 では新しく `V2XOvertakePlanner` を追加し、追い越し可能なときだけ MPC の lateral path constraints に bias を与える。

初期方針は、MPC 本体の最適化問題を大改造しないこと。まず pure Python の判断器で状態、側方オフセット、速度 cap を返し、`mpc_controller.py` 側で既存参照パスへ安全に合成する。

## Components

### `v2x_overtake_planner.py`

新規 pure Python helper を追加する。

責務:

- `V2XVehiclePositionArray` 相当のデータを受ける。
- 自車 pose、参照経路、現在速度から追い越し候補を選ぶ。
- 追い越し可否、追い越し側、target lateral offset、speed cap、状態を返す。
- ROS 依存を持たせず、単体テストしやすくする。

想定 API:

```python
planner = V2XOvertakePlanner(config)
planner.update_v2x(msg, now_sec)
result = planner.compute_behavior(
    ego_x=pose.x,
    ego_y=pose.y,
    ego_yaw=pose.theta,
    ego_v_mps=v,
    reference_xy=waypoint_xy,
    reference_widths=waypoint_widths,
    now_sec=now_sec,
    velocity_lookup=v2x_tracker.velocity,
)
```

`result` の想定:

```python
V2XOvertakeResult(
    active=True,
    state="overtaking",
    vehicle_id="d2",
    side="left",
    gap_m=12.4,
    lateral_offset_m=0.6,
    target_lateral_offset_m=1.8,
    speed_cap_mps=5.0,
    reason="left_clear",
)
```

### `mpc_controller.py`

変更方針:

- `use_v2x_overtake` parameter を追加する。
- `use_v2x_overtake=true` のとき `/v2x/vehicle_positions` と `V2XVehicleTracker` を使う。
- Gate1 の `use_v2x_stop` は停止 fallback として残す。
- 制御周期ごとに overtake result を計算し、速度 cap と lateral bias を合成する。
- overtake result が inactive の場合は既存の Gate1 speed cap のみを使う。

合成順序:

```text
base_v_max = ref_vel_configulator or mpc.v_max
gate1_cap = v2x_stop_planner.compute_speed_cap(...)
overtake = v2x_overtake_planner.compute_behavior(...)

if overtake.active:
    effective_v_max = min(base_v_max, overtake.speed_cap_mps)
    apply_lateral_bias(overtake.target_lateral_offset_m)
else:
    effective_v_max = min(base_v_max, gate1_cap)
    apply_lateral_bias(0.0)
```

注意:

- Gate2 で overtake active の間だけ、Gate1 の停止保持を解除または弱める必要がある。
- 危険距離では overtake を中止し、Gate1 の停止 cap を優先する。
- 速度単位は m/s に統一して合成する。

### Config

`config.yaml` と `sim_config.yaml` に `v2x_overtake` section を追加する。

初期値案:

```yaml
v2x_overtake:
  enabled: true
  detection_range_m: 35.0
  min_overtake_start_gap_m: 6.0
  max_overtake_start_gap_m: 22.0
  abort_gap_m: 3.5
  return_clearance_m: 4.0
  corridor_half_width_m: 2.0
  preferred_side: "right"
  side_selection_policy: "largest_margin"
  lateral_offset_m: 1.3
  lateral_offset_rate_mps: 2.0
  constraint_half_width_m: 0.35
  constraint_transition_horizon_ratio: 0.25
  constraint_initial_progress: 0.45
  standby_lateral_offset_m: 0.35
  standby_side: "right"
  min_lateral_clearance_m: 1.2
  min_wall_clearance_m: 0.5
  overtake_speed_cap_kmph: 6.0
  prepare_speed_cap_kmph: 3.0
  follow_speed_cap_kmph: 5.0
  overtake_steer_rate_max: 1.2
  lateral_ready_threshold_m: -0.6
  steer_override_enabled: true
  steer_override_min_abs_rad: 0.16
  steer_override_until_ey_m: -0.3
  stale_timeout_sec: 2.0
  target_lost_hold_sec: 1.2
  log_throttle_sec: 1.0
```

### Launch

`aichallenge_submit_launch/launch/control/mpc.launch.xml` に以下を追加する。

```xml
<arg name="use_v2x_overtake" default="false"/>
<param name="use_v2x_overtake" value="$(var use_v2x_overtake)"/>
```

初期値は false にする。Gate1 と通常走行の退行を避け、Gate2 検証時に有効化する。

Gate2 用に Makefile や launch から明示的に有効化する場合は、既存の `make gate2` 経路を確認してから行う。

## State Machine

| State | 条件 | 出力 |
|---|---|---|
| clear | 前方追い越し対象なし | lateral offset 0、通常 V2X stop |
| follow | 前方対象あり、追い越し未許可 | 距離ベース speed cap、lateral offset 0 |
| prepare_overtake | 追い越し許可、横移動開始 | lateral offset を徐々に増やす |
| overtaking | 対象横または直後を通過中 | lateral offset 維持、速度 cap |
| return_to_line | 対象より十分前方 | lateral offset を 0 へ戻す |
| abort | 危険、V2X 異常、MPC infeasible | speed cap を下げ、Gate1 停止へ倒す |

遷移ルール:

1. `clear` から前方対象を検出したら `follow`。
2. `follow` で追い越し条件を満たしたら `prepare_overtake`。
3. lateral offset が目標値付近に到達したら `overtaking`。
4. 対象より `return_clearance_m` 以上前へ出たら `return_to_line`。
5. offset が 0 付近に戻ったら `clear`。
6. どの状態でも `abort_gap_m` 以下、V2X stale、横余裕不足、MPC infeasible なら `abort`。

## Lateral Behavior

初期実装は、参照経路の waypoint 自体は変更せず、MPC が参照する lateral path constraints の現在 horizon row を一時的に追い越し側へ寄せる。

```text
offset_cmd = ramp(current_offset, target_offset, lateral_offset_rate_mps * dt)
progress = max(initial_progress, min(1.0, horizon_col / transition_cols))
center = current_ey + progress * (offset_cmd - current_ey)
upper = min(wp.ub - safety_margin, center + constraint_half_width)
lower = max(wp.lb + safety_margin, center - constraint_half_width)
```

追い越し開始直後に offset ramp が小さい間でも反対側へ流れないよう、offset の符号に応じて制約帯を片側へ固定する。

```text
if offset_cmd > 0:  # left overtake
    lower = max(lower, 0.0)
if offset_cmd < 0:  # right overtake
    upper = min(upper, 0.0)
```

この方式を選ぶ理由:

- `V2XOvertakePlanner` を ROS-free に保てる。
- 参照 waypoint、曲率、`v_ref` を動的に書き換えずに済む。
- MPC 既存の `e_y` 制約を使って、追い越し側の狭い通行帯へ誘導できる。
- `/mpc/ref_path` の dynamic border cells で可視化しやすい。

Gate2 では通常走行の操舵レート上限を全体変更せず、`prepare_overtake` / `overtaking` / `return_to_line` と横 offset が残っている間だけ `overtake_steer_rate_max` を MPC 内部値へ変換して適用する。V2X target 検出後だけでは横移動が間に合わないため、`standby_lateral_offset_m` で target 未検出時から右側へ寄せて待機する。

追い越し開始後に V2X target が短時間 stale になった場合は、`target_lost_hold_sec` の間だけ選択済み side と lateral offset を保持し、`follow_speed_cap_kmph` で継続する。実際の `ey` が追い越し側へ入るまでは、`prepare_speed_cap_kmph` でさらに低速にして接近を抑える。

Gate2 の低速通過を優先し、MPC の corridor 制約だけでは初期横移動が間に合わない場合は `steer_override_enabled` を使う。これは `prepare_overtake` / `overtaking` 中だけ、かつ `ey=-0.3m` 付近へ入るまで、選択 side へ最低操舵角を入れる補助である。右へ行きすぎると後続 target を拾って停止しやすく、浅すぎると 1 台目へ接触するため、中間値として扱う。

`standby_lateral_offset_m` は target 未検出時の先読みだけに使う。`return_to_line` や `follow` で planner が offset 0 を返している場合は、standby offset を再適用せず 0 へ戻す。

実装時に確認すること:

- 制約帯が `wp.ub - safety_margin` と `wp.lb + safety_margin` の範囲内に収まるか。
- 制約帯が反転する場合は静的制約へ fallback すること。
- 候補ラインが occupancy grid / border cells 上で `min_wall_clearance_m` を満たすか。
- clear / abort 時に offset を 0 へ戻し、静的制約へ復帰できるか。

代替案:

- `use_obstacle_avoidance=true` と V2X 由来 obstacle を使い、既存の obstacle avoidance 制約で回避する。
- `path_constraints_provider` を V2X 入力へ対応させ、`PathConstraints` で通行可能幅を与える。

初期実装では、まず path constraints bias 方式で Gate2 を通す。MPC infeasible や壁接触が多い場合は、offset 幅、速度 cap、wall clearance の閾値を保守側へ調整する。

## Candidate Selection

Gate1 の前方対象選択を拡張する。

1. 自車 pose を `/localization/kinematic_state` から取得する。
2. 参照経路 `reference_xy` から自車最近傍 index を求める。
3. 各 V2X 対象について、対象最近傍 index を求める。
4. circular path として前方距離 `gap_m` を求める。
5. 対象の path からの横方向距離 `lateral_offset_m` を求める。
6. `V2XVehicleTracker` から target velocity と relative speed を推定する。
7. `gap_m` が最小の前方対象を primary target にする。
8. primary target 以外も周辺車両として lateral clearance 判定に使う。

## Overtake Permission

追い越し許可条件:

```text
min_start_gap <= gap_m <= max_start_gap
lateral_clearance(side) >= min_lateral_clearance_m
abs(target_lateral_offset_m) <= reference_path.max_width / 2 - safety_margin
wall_clearance(side, target_lateral_offset_m) >= min_wall_clearance_m
estimated_ttc > min_ttc_sec
```

`lateral_clearance(side)` は初期実装では以下の簡易判定にする。

- 追い越し側に V2X 対象がいない。
- 追い越し側の target offset が参照パス最大幅を超えない。
- 近傍の別対象が `gap_m` と lateral offset の矩形範囲に入らない。

## Side Selection

右へ行くか左へ行くかは、V2X だけでは決めない。V2X の車両配置に加えて、参照経路の左右壁距離を使って side ごとの安全余裕を計算する。

基本判定:

```text
left_available_m  = min(wp.ub       for wp in horizon)
right_available_m = min(abs(wp.lb)  for wp in horizon)

required_m = abs(target_lateral_offset_m)
           + vehicle_width_m / 2
           + safety_margin_m
           + min_wall_clearance_m

left_wall_safe  = left_available_m  >= required_m
right_wall_safe = right_available_m >= required_m

left_v2x_safe  = no vehicle in left overtake corridor
right_v2x_safe = no vehicle in right overtake corridor
```

選択ルール:

1. 追い越し開始後に `forced_side` がある場合、その側が safe なら維持する。
2. `forced_side` が unsafe になった場合、反対側へ切り替えず abort / follow に倒す。
3. 初回選択で片側だけ safe なら、その側を選ぶ。
4. 初回選択で両側 safe なら、`side_selection_policy=largest_margin` に従い wall margin が大きい側を選ぶ。
5. 左右の wall margin 差が `side_margin_tie_threshold_m` 以下なら `preferred_side` を tie-breaker にする。
6. 両側 unsafe なら追い越しせず `follow` または Gate1 stop fallback に倒す。

`wp.ub` は参照経路ローカル左側の壁までの距離、`wp.lb` は右側を負値で持つ距離である。絶対座標の東西南北ではなく、waypoint の進行方向に対する left / right として扱う。

## Wall Clearance

壁・コース境界の距離は Gate2 の追い越し許可条件に含める。`reference_path.max_width` は上限の粗い guard として使えるが、局所的な壁距離の代替にはしない。

初期実装では、既存 `ReferencePath` が occupancy grid map から計算している `wp.ub` / `wp.lb` / border cells / path constraints を使う。

判定方針:

1. ego 近傍 waypoint から horizon 内の左右 border cells を取得する。
2. 追い越し候補 offset の側について、各 waypoint の available lateral width を求める。
3. `abs(target_lateral_offset_m) + vehicle_half_width + safety_margin + min_wall_clearance_m` が available width 以下か確認する。
4. horizon 内で 1 点でも不足する場合、その side は unsafe とする。
5. 左右とも unsafe の場合は follow または Gate1 stop fallback に倒す。

既存コードの関連:

- `ReferencePath._compute_width()` は occupancy grid map から waypoint ごとの左右 border cell を求める。
- `ReferencePath.update_simple_path_constraints()` は static border cells から MPC の上下限制約を作る。
- `ReferencePath.update_path_constraints()` は障害物回避時に dynamic border cells を更新する。

Gate2 では、追い越し判断側でこの border cell 情報を読むだけに留め、最初から評価基盤や map 生成処理は変更しない。

## Speed Policy

Gate2 の速度 cap は状態ごとに分ける。

| State | speed cap |
|---|---|
| clear | 通常 `ref_vel.yaml` / `v_max` |
| follow | `follow_speed_cap_kmph` と距離ベース cap の min |
| prepare_overtake | `overtake_speed_cap_kmph` |
| overtaking | `overtake_speed_cap_kmph` |
| return_to_line | `overtake_speed_cap_kmph` から通常速度へ徐々に戻す |
| abort | Gate1 stop cap を優先 |

横移動中は速くしすぎない。最初の Gate2 通過までは lap time より接触回避を優先する。

実走ログでは Gate2 開始時点で `ey` が約 `+2.0m`、右追い越し target が `-1.8m` となり、短距離で約 3.8m の横移動が必要だった。一方で強い override では `ey=-3.2m` 以上まで右へ行きすぎ、後続 target を拾って停止した。浅い `lateral_offset_m=0.9m` では 1 台目との横分離が不足したため、Gate2 の初期通過優先値は `standby_lateral_offset_m=0.35m`、`lateral_offset_m=1.3m`、`constraint_initial_progress=0.45`、`prepare_speed_cap_kmph=3km/h`、`overtake_speed_cap_kmph=6km/h`、`follow_speed_cap_kmph=5km/h`、`overtake_steer_rate_max=1.2rad/s`、`steer_override_min_abs_rad=0.16rad` にする。

`bicycle_model.width` は実車幅 `1.45m` を使う。安全込みの `2.30m` を入れると、MPC safety margin と Gate2 の wall clearance が二重に効いて、初期位置が path constraint 外になりやすい。

## Validation

単体テスト:

- 前方対象だけ追い越し候補になる。
- 後方対象は無視される。
- 横方向に遠い対象は無視される。
- stale sample は追い越し許可しない。
- start gap 範囲外では追い越し許可しない。
- preferred side が空いていればその側を選ぶ。
- preferred side が塞がっていれば反対側または follow を選ぶ。
- abort gap 以下では overtake ではなく abort / stop になる。
- return clearance を超えたら `return_to_line` へ遷移する。

実機能確認:

```bash
make autoware-build
make gate1
make gate2
```

観測:

```bash
ros2 topic echo --once /v2x/vehicle_positions
ros2 topic echo --once /control/command/control_cmd
ros2 topic hz /control/command/control_cmd
```

確認ログ:

- `output/latest/d<N>/autoware.log`
- `output/latest/d<N>/result-summary.json`
- `output/latest/d<N>/result-details.json`
- `output/latest/d<N>/rosbag2_autoware.mcap`

期待ログ:

- `V2X overtake: follow ...`
- `V2X overtake: prepare_overtake ... side=...`
- `V2X overtake: overtaking ... offset=...`
- `V2X overtake: return_to_line ...`
- `V2X overtake: abort ... reason=...`

## Risks

- Gate1 の停止保持が強すぎると、Gate2 で横回避前に停止し続ける。
- `reference_path.max_width` だけに頼ると、局所的に壁へ近い区間で追い越し許可を誤る。
- lateral offset の制約帯が狭すぎると MPC infeasible になりやすい。
- 参照経路は動かさないため、横制約だけでは追い越し開始が遅れる可能性がある。
- V2X 更新周期が低い場合、追い越し対象の位置推定が粗くなる。
- preferred side がシナリオに合わない場合、片側固定では Gate2 が通らない。
- 追い越し中の速度が高いと MPC infeasible や壁接触が起きやすい。

## Migration To Race Behavior

Gate2 が通った後、同じ `V2XOvertakePlanner` を race behavior の初期版として使う。ただし race では複数台同時走行になるため、以下を追加で扱う。

- 複数先行車の優先順位。
- 後続車・横並び車両への配慮。
- 追い越し禁止区間や狭い区間の速度制限。
- 追走、停止、追い越しの切り替え hysteresis。
