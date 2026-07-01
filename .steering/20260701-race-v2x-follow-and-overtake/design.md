# Race V2X Follow And Overtake Design

## Summary

race behavior は Gate1 の縦方向 stop/follow と Gate2 の横方向 overtake を統合し、2 台同時走行で使える状態機械にする。

初期方針は、Gate2 で通した `V2XOvertakePlanner` を直接本番 mode にしないこと。Gate2 は安全ゲート通過のために低速・強い補助・待機 offset を含む。race では、まず先行車を安定して追走し、安全な区間だけ追い越しへ入る。

## Proposed Components

### `V2XRaceBehaviorPlanner`

新規 pure Python helper として追加する案。

責務:

- `/v2x/vehicle_positions` 相当の snapshot を保持する。
- 自車 pose、参照経路、左右壁距離、現在速度から target roles を決める。
- follow / prepare_overtake / overtaking / return_to_line / abort を返す。
- Gate1 / Gate2 helper の共通部分を再利用できる範囲で使う。
- ROS 依存を持たせず、単体テストしやすくする。

想定 API:

```python
planner = V2XRaceBehaviorPlanner(config)
planner.update_v2x(msg, now_sec)
result = planner.compute_behavior(
    ego_x=pose.x,
    ego_y=pose.y,
    ego_yaw=pose.theta,
    ego_v_mps=v,
    reference_xy=waypoint_xy,
    reference_widths=waypoint_widths,
    now_sec=now_sec,
    base_speed_mps=base_v_mps,
    current_lateral_offset_m=current_ey,
    velocity_lookup=v2x_tracker.velocity,
)
```

想定 result:

```python
V2XRaceBehaviorResult(
    active=True,
    state="follow",
    target_id="d2",
    target_role="front",
    side=None,
    speed_cap_mps=4.2,
    target_lateral_offset_m=0.0,
    gap_m=8.5,
    lateral_offset_m=0.3,
    relative_speed_mps=-0.8,
    target_speed_mps=3.5,
    left_wall_margin_m=1.2,
    right_wall_margin_m=0.8,
    yield_active=False,
    yield_target_id=None,
    reason="follow_gap_control",
)
```

### `mpc_controller.py`

追加方針:

- `use_v2x_race_behavior` parameter を追加する案を第一候補にする。
- 既存 `use_v2x_stop` は最後の安全停止 fallback として維持する。
- 既存 `use_v2x_overtake` は Gate2 専用検証の意味を残す。
- race behavior active の場合、速度 cap と lateral offset は race planner の結果を優先する。
- race planner が inactive / abort / unsafe の場合、Gate1 stop/follow に倒す。

合成順序:

```text
base_v_mps = min(ref_vel, configured_v_max)
race = race_planner.compute_behavior(...)

if race.active:
    effective_v_mps = min(base_v_mps, race.speed_cap_mps)
    apply_lateral_bias(race.target_lateral_offset_m)
    if race.stop_fallback_required:
        effective_v_mps = min(effective_v_mps, gate1_stop_cap)
else:
    effective_v_mps = min(base_v_mps, gate1_stop_cap)
    apply_lateral_bias(0.0)
```

注意:

- `v_max`、`speed_cap_mps`、`v_ref` は m/s で合成する。
- `use_v2x_race_behavior` と `use_v2x_overtake` が同時 true の場合は race behavior を優先し、warning を出す。
- Gate2 の `standby_lateral_offset_m` は race では既定 0 にする。
- Gate2 の `steer_override_enabled` は race では既定 false にする。

## Config

`v2x_race_behavior` section を追加する案。

```yaml
v2x_race_behavior:
  enabled: false
  detection_range_m: 40.0
  corridor_half_width_m: 2.0
  front_conflict_lateral_window_m: 2.6
  front_conflict_gap_m: 10.0
  self_ignore_radius_m: 0.75

  min_follow_gap_m: 3.0
  time_headway_sec: 0.8
  follow_kp_gap: 0.4
  follow_kd_closing: 0.8
  follow_speed_cap_kmph: 12.0
  emergency_stop_gap_m: 2.0

  yield_detect_range_m: 12.0
  yield_start_gap_m: 8.0
  yield_release_gap_m: 14.0
  yield_min_closing_speed_mps: 0.4
  yield_speed_cap_kmph: 8.0
  yield_timeout_sec: 4.0
  yield_side_hold_sec: 1.0
  catchup_detect_range_m: 120.0
  catchup_start_gap_m: 18.0
  catchup_release_gap_m: 8.0
  catchup_speed_cap_kmph: 10.0
  catchup_timeout_sec: 30.0
  approach_start_gap_m: 34.0
  approach_speed_cap_kmph: 18.0

  min_overtake_start_gap_m: 4.0
  max_overtake_start_gap_m: 24.0
  min_closing_speed_mps: 0.4
  max_overtake_target_speed_kmph: 12.0
  min_ttc_sec: 0.8

  preferred_side: "right"
  side_selection_policy: "largest_margin"
  side_margin_tie_threshold_m: 0.3
  lateral_offset_m: 1.4
  lateral_offset_rate_mps: 1.2
  constraint_half_width_m: 0.55
  min_lateral_clearance_m: 1.6
  min_wall_clearance_m: 0.5
  wall_safety_margin_m: 0.2
  wall_check_horizon_m: 16.0

  overtake_speed_cap_kmph: 12.0
  prepare_speed_cap_kmph: 9.0
  return_clearance_m: 8.0
  return_rear_clearance_m: 4.0
  return_offset_threshold_m: 0.25
  target_lost_hold_sec: 1.0
  stale_timeout_sec: 2.0
  overtake_cooldown_sec: 2.0
  side_lock_min_sec: 1.0
  side_switch_center_threshold_m: 0.45
  log_throttle_sec: 1.0
```

初期値は Gate2 より横移動を穏やかにする一方、race2 では前走車との距離が 3m 台で安定しやすいため、`min_overtake_start_gap_m` は 4.0m として追い抜き開始を早める。壁余裕は長い horizon を見るとコーナー手前で `no_safe_side` になり続けるため、race 用は `wall_check_horizon_m=16.0` とし、直近から少し先まで抜ける空間がある場合だけ試行する。2026-07-01 の試走では `wall_l=0.02m` でも `left_forced_clear` になり壁接触したため、`min_wall_clearance_m=0.5` に戻し、車体が片側へ寄っている間の反対側追い越し指示は `side_switch_center_threshold_m` で抑止する。Gate2 で使った `lateral_offset_m=1.65`、`prepare_speed_cap_kmph=3.0`、強制 steer override は安全ゲート用の最後の調整値であり、race の初期値にはしない。

## State Machine

| State | 条件 | 主出力 |
|---|---|---|
| clear | 前方 target なし | 通常速度、offset 0 |
| follow | 前方 target あり、追い越し未許可 | 距離ベース speed cap、offset 0 |
| yield | 後方 target が接近、抜かせる余地あり | 低め speed cap、offset 0 または現ライン維持 |
| catchup_wait | 後方 target が大きく離れ、追いつけない | 中速 speed cap、offset 0 |
| prepare_overtake | 追い越し許可、side lock | 低め speed cap、offset ramp |
| overtaking | target 横または直後を通過中 | offset 維持、side fixed |
| return_to_line | target より前方、復帰先 clear | offset 0 へ ramp |
| cooldown | 追い越し完了直後 | offset 0、再追い越し抑制 |
| abort | 危険、V2X stale、MPC fault | stop/follow fallback |

遷移:

1. `clear` で前方 target を検出したら `follow`。
2. `clear` または `follow` で後方 target が接近し、前方に危険がなければ `yield`。
3. `clear` で後方 target が `catchup_start_gap_m` 以上離れている場合は `catchup_wait`。
4. `catchup_wait` で後方 target が `catchup_release_gap_m` 以内へ詰める、または timeout したら `clear` / `follow` へ戻る。
5. `yield` で後方 target が追い越し終える、離れる、または timeout したら `clear` / `follow` へ戻る。
6. `follow` で overtake permission が true なら `prepare_overtake`。
7. side-aware lateral progress が閾値を超えたら `overtaking`。
8. target より `return_clearance_m` 以上前に出て、復帰先が clear なら `return_to_line`。
9. `ey` が `return_offset_threshold_m` 以下になったら `cooldown`。
10. cooldown 経過後に `clear` または `follow` へ戻る。
11. どの状態でも emergency gap、wall unsafe、MPC fault なら `abort`。

## Target Selection

参照パス上の polyline projection で `s` と lateral offset を求める。waypoint 最近傍だけに依存しない。

分類:

- `front_target`: `0 < signed_gap_m <= detection_range_m` かつ `abs(lateral_offset_m) <= corridor_half_width_m`
- `side_blocker`: 追い越し候補 side の corridor 内にいる車両
- `return_blocker`: 復帰ライン前方または後方の corridor 内にいる車両
- `behind_target`: `signed_gap_m < 0` で復帰時の後方余裕に関係する車両
- `yield_target`: 後方から同一 corridor で接近し、自車が先行側として速度を譲る対象
- `catchup_target`: 後方に大きく離れた同一 corridor の車両

`front_target` が複数ある場合は gap が最小の車両を優先する。ただし `target_lock` 中は、stale timeout まで現在 target を維持する。

## Follow Policy

追走速度 cap:

```text
desired_gap = max(min_follow_gap_m, time_headway_sec * ego_v_mps)
gap_error = front_gap_m - desired_gap
closing_speed = ego_v_mps - target_speed_mps
follow_v = target_speed_mps + follow_kp_gap * gap_error - follow_kd_closing * closing_speed
speed_cap = clamp(follow_v, 0.0, follow_speed_cap_mps)
```

target speed が推定できない場合は、現在の `base_v_mps` と距離ベース braking cap の min を使う。

`front_gap_m <= emergency_stop_gap_m` では Gate1 stop fallback を優先する。

横並び・斜め前の車両は、path projection だけだと一時的に前方 target から外れる場合がある。そのため `front_conflict_gap_m` 以内かつ `front_conflict_lateral_window_m` 以内の yaw-based 前方 target は follow 対象に含める。

ただし yaw-based relation が明確に後方を示す target は、path projection が前方に見えても front target へ昇格しない。2 台が横並びや旋回中に互いを front として見てしまうと、両車が `follow_speed_cap_kmph` に落ちて追い越し機会を作れないため。

`front_gap_m > max_overtake_start_gap_m` の遠い前方車は、まだ追走対象ではなく追いつく対象として扱う。ただし `approach_start_gap_m` 以内へ入ったら `approach_speed_cap_kmph` で段階的に減速し、追い越し開始距離に入る前の rear-end を避ける。`approach_start_gap_m` より遠い場合は、後続側も低速になって先行側の catch-up wait が効かなくなるため、通常速度を維持する。

MPC 統合では、race behavior が front target を処理している間は race 側の speed cap と emergency stop を正にする。Gate1 stop fallback を同時に掛けると、stop planner の stale hold が race follow 中の後続車を 0 m/s に落とすため、front target を持つ `follow` / `prepare_overtake` / `overtaking` / `return_to_line` では stop fallback を bypass する。

## Yield Policy

同じアルゴリズムの車両を 2 台同時に動かすと、先行側が全力走行を続ける限り、後続側は相対速度を作れず安全な追い越し開始条件に入りにくい。そのため race behavior は、先行側にも限定的な yield を持たせる。

近距離の yield とは別に、先行車が初期加速や経路差で大きく離れた場合の catch-up wait を持たせる。これは追い越し直前の譲りではなく、後続が追い越し判断へ入れる距離まで詰めるための pace cap である。

yield target:

```text
behind_gap = abs(signed_gap_m) for signed_gap_m < 0
same_corridor = abs(lateral_offset_m) <= corridor_half_width_m
closing_speed = target_speed_mps - ego_v_mps
yield_candidate =
    same_corridor
    and behind_gap <= yield_detect_range_m
    and (behind_gap <= yield_start_gap_m or closing_speed >= yield_min_closing_speed_mps)
```

yield 出力:

```text
speed_cap = min(base_v_mps, yield_speed_cap_mps)
target_lateral_offset_m = 0.0
```

補足:

- yield は「停止」ではなく「後続車が追い越し判断に入れる程度の減速」に留める。
- 後続車が追い越し side に入った場合、自車はその side へ寄らず現在ラインまたは中央を維持する。
- 後続車が自車より `yield_release_gap_m` 以上前方へ出たら yield を解除し、cooldown に入る。
- `yield_timeout_sec` を超えても追い越しが成立しない場合は yield を解除する。無限に譲り続けると race として成立しないため。
- 自車前方に emergency gap の target がいる場合は yield より Gate1 stop fallback を優先する。

catch-up wait:

```text
behind_gap = abs(signed_gap_m) for signed_gap_m < 0
catchup_candidate =
    same_corridor
    and catchup_start_gap_m <= behind_gap <= catchup_detect_range_m
```

出力:

```text
speed_cap = min(base_v_mps, catchup_speed_cap_mps)
target_lateral_offset_m = 0.0
```

解除条件:

- `behind_gap <= catchup_release_gap_m`
- `catchup_timeout_sec` 経過
- 前方 target が出現した場合

## Overtake Permission

追い越し許可条件:

```text
front_gap in [min_overtake_start_gap_m, max_overtake_start_gap_m]
target_speed <= max_overtake_target_speed
closing_speed >= min_closing_speed_mps or target_speed is slow
estimated_ttc >= min_ttc_sec
side_wall_margin >= min_wall_clearance_m
side_vehicle_clearance >= min_lateral_clearance_m
return_front_clearance >= return_clearance_m
return_rear_clearance >= return_rear_clearance_m
not in cooldown
```

左右の選択は Gate2 と同じく wall margin を優先する。wall margin 差が小さい場合だけ `preferred_side` を tie-breaker にする。

## Return Permission

復帰は追い越しより保守的にする。

- overtaken target より十分前にいる。
- 復帰ライン前方に別 target がいない。
- 復帰ライン後方に高速接近車がいない。
- 現在 side のまま走り続けても壁距離が不足しない。

復帰できない場合は、overtake side を維持しつつ速度 cap を低めにして次の判断周期へ送る。

## Alternating Overtake Trial

2 台で交互追い越しを試す場合、初期は以下の段階で進める。

### Trial A: One-Way Overtake

- d1 を ego、d2 を低速先行車として扱う。
- d1 が follow へ入り、gap を詰めすぎず維持できるか確認する。
- d1 が safe side を選び、d2 を抜いて復帰できるか確認する。
- d2 側は追い越ししなくてもよい。

### Trial B: Yielded Overtake

- d1 / d2 の両方で race behavior を有効にする。
- d2 が先行側になったら、d1 の接近を見て yield speed cap に入るか確認する。
- d1 が yield 中の d2 を追い越し、d2 が追い越し side へ寄らないことを確認する。
- d1 が復帰するまで、d2 が急加速しないことを確認する。

### Trial C: Mutual Awareness

- d1 / d2 の両方で race behavior を有効にする。
- 片方が追い越し中、もう片方が同じ side へ切り込まないことを確認する。
- 横並び時に両車が急に中央へ戻らないことを確認する。

### Trial D: Alternating Overtake

- 追い越し完了後、速度 cap や初期条件を変えて立場が入れ替わる状況を作る。
- 次の車両が follow -> overtake -> return へ入れることを確認する。
- cooldown により抜き返し振動が起きないことを確認する。

Trial D は最初から最速化しない。判断 log と collision-free を優先する。

## Launch / Command Plan

初期案:

- 既存 `make dev2` をベースにする。
- race behavior 有効化用に launch arg を追加する。
- 必要なら `make race2` または `SIM_MODE=race2` を追加する。

候補:

```bash
make dev2
make autoware-bash
ros2 topic echo --once /v2x/vehicle_positions
ros2 topic hz /control/command/control_cmd
```

race 用 target を追加する場合:

```bash
make race2
```

`make race2` を追加するかは、既存 Makefile の dev2/dev3/dev4 と評価起動フローを読んでから決める。既存 `dev2` の意味を壊さない。

## Logs

期待 log:

- `V2X race: clear`
- `V2X race: follow target=<id> gap=<m> v_cap=<m/s> reason=<reason>`
- `V2X race: yield target=<id> behind_gap=<m> v_cap=<m/s> reason=<reason>`
- `V2X race: prepare_overtake target=<id> side=<side> wall_l=<m> wall_r=<m>`
- `V2X race: overtaking target=<id> side=<side> offset=<m>`
- `V2X race: return_to_line target=<id> return_front=<m> return_rear=<m>`
- `V2X race: cooldown remaining=<sec>`
- `V2X race: abort reason=<reason>`

## Risks

- Gate2 の強い右寄せ・補助操舵を race に流用すると、横並び車両や壁へ近づきすぎる。
- 両車が同じ logic で同時に追い越しへ入ると、同じ side を取り合う可能性がある。
- yield speed cap が強すぎると、先行車が不自然に減速して race として遅くなりすぎる。
- yield timeout が短すぎると後続車が追い越し開始前に機会を失い、長すぎると譲り続ける。
- 追い越し後の復帰で後続車の前へ切り込むリスクがある。
- V2X 更新周期が低い場合、target speed と relative speed が粗くなり、追走が振動する。
- 2 台で安定しても 3〜4 台では target queue と return blocker が増える。

## Open Questions

- 公式 race 評価で参加者が各車両の個別 config を変えられるか。
- `/v2x/vehicle_positions` に自車 ID を含むか、含む場合の識別方法。
- V2X の座標系、更新周期、遅延の公式保証。
- 同時走行時に評価対象は全車か、自車 1 台か。
- 接触、壁接触、進路妨害の penalty 定義。
