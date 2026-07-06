# V2X Front Risk Arbitration Design

作成日: 2026-07-06
状態: Draft

## 方針

前方車両に対して、MPC 内で個別に積み上がっている braking guard、overtake guard、low-speed avoidance を、ひとつの前方リスク判断に寄せる。

最初の実装では大規模な planner 再設計は行わず、既存の `evaluate_v2x_behavior()` の中に risk 計算と arbitration を追加する。MPC の最適化問題そのものは大きく変えず、出力として次を選ぶ。

- `target_velocity_limit`
- `allow_gap_planner`
- `LowSpeedAvoidance` local path planner の利用可否
- `Overtake` へ入るかどうか
- `SafetyBrake` へ倒すかどうか

## 追加コンポーネント

### FrontRiskConfig

`V2XBehaviorConfig` に以下を追加する。

- `debug_log_enabled`
- `debug_log_period_sec`
- `follow_gap_planner_enabled`
- `follow_gap_planner_no_gap_speed_limit_enabled`
- `follow_gap_planner_respect_overtake_forbidden`
- `front_risk_arbitration_enabled`
- `front_risk_brake_prepare_limit_enabled`
- `front_risk_avoid_candidate_limit_enabled`
- `front_risk_distance_margin`
- `front_risk_comfort_decel`
- `front_risk_hard_decel`
- `front_risk_emergency_decel`
- `front_risk_min_closing_speed`
- `front_risk_prepare_time`
- `overtake_guard_reachable_gap_enabled`
- `overtake_guard_max_lateral_accel`
- `overtake_guard_min_gap_time`
- `overtake_guard_min_speed_for_reachable`
- `overtake_before_curve_enabled`
- `overtake_before_curve_max_front_speed`
- `overtake_before_curve_min_speed_advantage`
- `overtake_continue_in_forbidden_enabled`
- `overtake_front_velocity_limit_enabled`
- `overtake_gap_lookahead_distance`
- `overtake_target_ramp_enabled`
- `overtake_target_ramp_ratio`

既存の `front_decel_guard` は、移行中は併存させる。新 arbitration が安定したら `front_decel_guard` を薄くする。

### FrontRiskMetrics

前方車両1台に対して、次を保持する小さな構造体を追加する。

```cpp
struct FrontRiskMetrics
{
  bool valid{false};
  double front_distance{infinity};
  double front_speed{infinity};
  double ego_speed{0.0};
  double relative_speed{0.0};
  double available_distance{infinity};
  double required_decel{0.0};
  double ttc{infinity};
  bool moving_front{false};
  bool low_speed_front{false};
};
```

### FrontRiskLevel

required decel と TTC から段階化する。

```cpp
enum class FrontRiskLevel
{
  Clear,
  Follow,
  BrakePrepare,
  AvoidCandidate,
  EmergencyBrake,
};
```

分類の初期案:

- `relative_speed <= min_closing_speed`: `Follow`
- `available_distance <= 0`: `EmergencyBrake`
- `required_decel >= emergency_decel`: `EmergencyBrake`
- `required_decel >= hard_decel`: `AvoidCandidate`
- `required_decel >= comfort_decel`: `BrakePrepare`
- それ以外: `Follow`

### ReachableGapMetrics

gap planner の出力から、回避候補の到達可能性を評価する。

保持する値:

- `feasible`
- `pass_side_sign`
- `first_gap_distance`
- `max_lateral_shift`
- `estimated_time_to_gap`
- `required_lateral_accel`
- `required_steer_rate_margin`
- `block_reason`

初期実装は厳密な車両運動ではなく、保守的な近似でよい。

横加速度の近似:

```text
t = max(first_gap_distance / max(ego_speed, eps), min_time)
required_lateral_accel = 2 * abs(lateral_shift) / (t * t)
```

`required_lateral_accel` がしきい値を超える場合は、gap があっても到達不可とする。

## 評価フロー

`evaluate_v2x_behavior()` の中を次の順序に整理する。

1. V2X 車両一覧を取得し、自車・stale 情報を除外する。
2. 前方衝突幅に入る最も近い前方車を選ぶ。
3. `FrontRiskMetrics` を計算する。
4. `FrontRiskLevel` を決める。
5. 低速/停止車両なら `LowSpeedAvoidance` 候補を確認する。
6. 動く前走車なら通常 gap planner から reachable gap を確認する。
7. risk level と reachable gap で状態を決める。
8. velocity limit と reason を設定する。

## Arbitration ルール

初期ルール:

| Risk | Reachable gap | State | 速度 |
|---|---|---|---|
| Clear | 任意 | Cruise | 通常 |
| Follow | なし | Follow | 原則制限なし |
| BrakePrepare | なし | Follow | 既定では速度制限なし。`front_risk_brake_prepare_limit_enabled=true` の場合だけ required decel 由来の速度上限 |
| BrakePrepare | あり | Overtake | 必要なら前処理速度上限 |
| AvoidCandidate | なし | Follow | 既定では強めの速度上限。レース中に譲りすぎる場合は `front_risk_avoid_candidate_limit_enabled=false` で警戒レベルとして扱う |
| AvoidCandidate | あり | Overtake / LowSpeedAvoidance | 回避速度上限 |
| EmergencyBrake | 任意 | SafetyBrake | `v2x_safety_brake_velocity` |

カーブ中は reachable gap の基準を厳しくする。

`follow_gap_planner_enabled=true` の場合、Follow でも feasible な gap planner 出力だけを横制約と target に使う。`follow_gap_planner_no_gap_speed_limit_enabled=false` では gap 不成立時の `no_gap_target_velocity` を Follow に適用しないため、横へ競りに行けない瞬間でも譲り減速はしない。`follow_gap_planner_respect_overtake_forbidden=true` では、曲率または WP 範囲で overtake forbidden の区間は Follow の gap planner も止め、ヘアピンで横に張りに行く挙動を抑える。

直線の競り合いを残しつつヘアピンで前走車の減速に追従させる場合は、`front_decel_guard_enabled=true`、通常側の `front_decel_guard_distance=0.0` / `front_decel_guard_ttc=0.0`、curve 側の `front_decel_guard_curve_distance` / `front_decel_guard_curve_ttc` を有効値にする。ヘアピンで前走車が `moving_front_speed_threshold` 以下まで落ちる場合は、`front_decel_guard_curve_include_slow_front=true` にしてカーブ中だけ低速前走車も速度上限の対象にする。前走車が曲がり込みで横から進路を塞ぐ場合は、`front_decel_guard_curve_lateral_margin` でカーブ中の前方判定横幅を広げる。`front_decel_guard_curve_lookahead_distance` は速度上限用の曲率先読み距離で、`overtake_forbidden_curve_lookahead_distance` より短くする。

`overtake_forbidden_curve_lookahead_distance` を指定すると、MPC horizon `N` より先の曲率まで見て overtake forbidden を立てる。ヘアピン手前から横攻めを止めたい場合に使う。

- `overtake_forbidden == true` の場合、通常 Overtake は許可しない。
- ただし `overtake_before_curve_enabled=true` の場合、WP 明示禁止ではなく曲率先読みだけで禁止になっている区間では、前走車が十分遅く、自車に速度優位があり、かつ reachable gap がある場合だけ Overtake 開始を許す。
- `overtake_continue_in_forbidden_enabled=true` の場合、すでに Overtake 中なら soft forbidden 区間で Overtake 継続を許し、途中で Follow へ戻される挙動を抑える。
- ただし停止/低速車の `LowSpeedAvoidance` 継続中は既存ルールを尊重する。
- 動く前走車に対してはカーブ中の回避より減速を優先する。

前方車検出距離が MPC horizon より長い場合、通常の `N` 点だけで gap planner を呼ぶと、低速前走車を検出していても通過側 gap がまだ horizon に入らず Follow に残りやすい。`overtake_gap_lookahead_distance` を指定した場合は、通常 Overtake の候補確認と Overtake 中の gap planner 呼び出しだけ、reference path の raw constraints から `N` より長い lb/ub を作って先読みする。MPC の最適化サイズは変えず、最終的に現 horizon `N` 点ぶんだけ横制約と target_ey を適用する。

`overtake_target_ramp_enabled=true` の場合、長い lookahead 上で最初に選ばれた追い越し target へ、現在の lateral から ramp で近づく target_ey を現 horizon 側にも設定する。`overtake_target_ramp_ratio` は target 到達の速さで、値を小さくすると手前から強く通過側へ寄る。これは低速前走車の真後ろに長く居座る挙動を抑えるための補助であり、reachable gap guard が成立した Overtake のみに使う。

`Overtake` に入って reachable gap がある場合でも、従来の `apply_follow_velocity_limits()` をそのまま掛けると、前走車速度 + margin や curve risk limit によって高速車が低速車へ引きずられる。`overtake_front_velocity_limit_enabled=false` では Overtake 中の front velocity limit を掛けず、追い越しの速度差を維持する。`EmergencyBrake` と inside stopping distance は Overtake 判定より前に評価されるため、近距離の停止/衝突回避は残す。

## Velocity Limit 設計

required decel から、次周期以降に目指す速度上限を求める。

初期案:

```text
safe_speed = front_speed + sqrt(max(0, 2 * allowed_decel * available_distance))
velocity_limit = min(current_v_max, safe_speed)
```

`allowed_decel` は risk level に応じて変える。

- `BrakePrepare`: `comfort_decel`
- `AvoidCandidate`: `hard_decel`
- `EmergencyBrake`: `v2x_safety_brake_velocity`

## Reachable Gap 設計

既存の `overtake_guard_allows()` を拡張する。

追加判定:

- `first_gap_distance / ego_speed` で到達時間を見積もる。
- `target_ey - current_ey` から必要横加速度を見積もる。
- `required_lateral_accel <= v2x_overtake_guard_max_lateral_accel` を要求する。
- 最初の gap が `v2x_overtake_guard_min_gap_time` 未満で現れる場合は、横移動が間に合わない可能性が高いため候補から外す。
- `first_gap_distance >= min_prepare_distance` を維持する。
- 連続 gap 点数と gap 幅の判定を維持する。

## ログ設計

状態遷移ログには次を含める。

- `front_distance`
- `front_speed`
- `relative_speed`
- `required_decel`
- `risk_level`
- `reachable_gap`
- `block_reason`

例:

```text
Follow -> SafetyBrake, front_distance=4.2, required_decel=7.1, reason=front risk emergency
Follow -> Overtake, front_distance=12.0, required_decel=3.2, reason=reachable gap available
Overtake -> Follow, front_distance=8.5, reason=reachable gap lateral accel
```

Phase4 の切り分けでは、state 遷移が起きない場合も `v2x_behavior_debug_log_enabled=true` で周期ログを出す。周期ログには次を入れる。

- desired state と state hold 後の final state
- `allow_gap_planner` と velocity limit
- active V2X 車両数
- front / side / danger / start-grid grace
- front distance / front speed / ego speed
- relative speed / required decel / available distance / risk level
- overtake forbidden / forbidden WP / curve guard
- low-speed candidate / low-speed block
- overtake zone / before-curve exception / continue exception
- gap availability / gap planner horizon / reason / block reason

`Follow` に居続ける場合は、主に `reason` と `block` を見る。`overtake guard lateral accel`、`overtake guard prepare distance`、`overtake forbidden curve`、`front risk emergency` のどれで止まっているかを特定してから config またはロジックを調整する。

## 影響範囲

主な変更箇所:

- `V2XBehaviorConfig`
- `evaluate_v2x_behavior()`
- `overtake_guard_allows()`
- `front_decel_guard_velocity_limit()`
- `config.yaml`
- `docs/spec/mpc-integration.md`

変更しないもの:

- ROS 2 topic/service 契約
- launch entry
- result JSON
- AWSIM/評価 FSM

## 段階的実装

### Phase 1: required decel braking

前走車に対する required decel を計算し、`FrontRiskLevel` を出す。

この段階では reachable gap へは接続せず、`Follow` / `SafetyBrake` の速度上限だけ改善する。

### Phase 2: reachable gap 判定

既存の `OvertakeGuard` を拡張し、横加速度と到達時間で gap を除外する。

### Phase 3: arbitration 統合

required decel と reachable gap を同じ判断器で比較し、`Follow` / `Overtake` / `LowSpeedAvoidance` / `SafetyBrake` を決める。

追加で、前方車検出距離と MPC horizon の不一致により追い越し候補探索が遅れる場合へ対応する。`overtake_gap_lookahead_distance` で gap planner の探索距離を伸ばし、`overtake_target_ramp_enabled` で現 horizon 内の target_ey を早めに通過側へ立ち上げる。

### Phase 4: チューニングとログ検証

`make gate1`、`make gate2`、`make dev2`、`make dev3` でログを見ながら config を調整する。

## リスク

- required decel が強すぎると後続車が失速する。
- reachable gap 判定が厳しすぎると追い越せない。
- reachable gap 判定が緩すぎると横移動が急になりスリップまたは接触する。
- V2X 速度推定が不安定だと required decel が振動する。
- `state_hold_time` と組み合わせて状態遷移が遅れる可能性がある。

## 検証方針

最低限:

- `make autoware-build`
- `make gate1`
- `make gate2`
- `make dev2`
- `make dev3`

見るべきログ:

- `FrontRiskLevel`
- `required_decel`
- `target_velocity_limit`
- `front decel guard`
- `reachable gap` の可否
- V2X behavior state transition

見るべき挙動:

- 前走車減速時に距離が詰まりすぎない。
- ブレーキだけで間に合わない場面で、到達可能 gap があれば早めに回避準備へ入る。
- 到達不能 gap へ無理に入らない。
- カーブ中に無理な横移動をしない。
- 通常単独走行を壊さない。
