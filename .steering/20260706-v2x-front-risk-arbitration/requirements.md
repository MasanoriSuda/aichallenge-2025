# V2X Front Risk Arbitration Requirements

作成日: 2026-07-06
状態: Draft

## 目的

V2X で検出した前方車両に対して、ブレーキ・追従・回避を同じ判断器で選択できるようにする。

現状は、前走車の減速に対してブレーキは掛かるが距離が詰まりすぎ、停止または追従が間に合わず衝突するケースがある。また、通れそうな隙間があっても、MPC がその隙間へ到達可能かを事前判定できず、回避へ移れない、または無理な横移動になる。

本要求では、単なる距離しきい値ではなく、required deceleration と reachable gap を使って、止まるか、追従するか、交わすかを早めに決める構造を作る。

## 現状認識

現行 C++ MPC には次の V2X 関連機能がある。

- `/v2x/vehicle_positions` を受け取り、前方車・側方車・低速車を判定する。
- `V2XBehaviorState` として `Cruise` / `Follow` / `Overtake` / `LowSpeedAvoidance` / `SafetyBrake` を持つ。
- `front_decel_guard` により、近距離の動く前走車へ速度上限を掛ける。
- `OvertakeGuard` により、gap 幅、準備距離、横移動量、前方距離を見て通常追い越しを抑制する。
- `LowSpeedAvoidance` と local path planner により、停止/低速車両列を横抜けする構造がある。

一方で、以下が不足している。

- 前走車に対して「現在の速度差で止まるために必要な減速度」を明示的に計算していない。
- ブレーキ開始が距離/TTC 依存で、速度差や制動余裕に対して遅れることがある。
- gap が存在しても、現在速度、必要横移動量、操舵レート、横加速度から到達可能かを判断していない。
- 前方車は `v2x_follow_distance` などで遠くから検出している一方、追い越し候補の gap planner は MPC horizon `N` 点ぶんしか見ないため、低速前走車の真後ろに居座ってから遅れて追い越し判断することがある。
- ブレーキと回避の優先順位が独立気味で、どちらも中途半端になるケースがある。
- 速度差を付けた複数車両がカーブで詰まると、追い越し禁止・減速・回避判断が遅れやすい。

## 対象範囲

対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml`
- `docs/spec/mpc-integration.md`
- 必要に応じて `ref_vel.yaml` の速度設計との整合確認

対象外:

- `/v2x/vehicle_positions` の topic 名・message 型変更。
- `/control/command/control_cmd` の topic 名・message 型変更。
- 評価基盤、AWSIM 管理、Domain 設計、result JSON schema の変更。
- フルスケールの behavior planner / route planner 再設計。
- 学習ベース planner の導入。
- 実車環境での未検証な回避制御。

## 必要機能

### Front Risk 計算

前方車両ごとに次を計算する。

- 前方距離 `front_distance`
- 前走車速度 `front_speed`
- 自車速度 `ego_speed`
- 相対速度 `relative_speed = ego_speed - front_speed`
- 安全余白を引いた有効距離 `available_distance`
- 必要減速度 `required_decel = relative_speed^2 / (2 * available_distance)`
- TTC
- 前走車が停止/低速/移動中の分類

`available_distance <= 0` の場合は、緊急状態として扱う。

### Risk レベル分類

required decel を基準に、少なくとも次のレベルへ分類する。

- `Clear`: 前方リスクなし。
- `Follow`: 通常追従で十分。
- `BrakePrepare`: 接近リスクを検出するが、既定では速度上限を下げず競り合いを維持する。
- `AvoidCandidate`: ブレーキだけではタイムロスまたは接近が大きく、回避候補を探す。
- `EmergencyBrake`: 必要減速度が大きく、回避判断より先に停止/急減速へ倒す。

しきい値は config 化する。

### Brake Arbitration

Risk レベルに応じて速度上限を決める。

- `Follow`: 前走車速度 + margin を上限にするか、速度制限なし。
- `BrakePrepare`: 既定では警戒ログのみ。必要な場合は config で required decel 由来の速度上限を有効化する。
- `EmergencyBrake`: `v2x_safety_brake_velocity` へ倒す。

通常 Follow を無駄に遅くしないため、閉じ速度が小さい場合は強い制限を掛けない。

### Reachable Gap 判定

gap planner の結果に対して、次を確認する。

- gap 幅が足りる。
- gap が horizon 上で連続して存在する。
- gap までの準備距離が足りる。
- 現在 lateral から target lateral までの横移動量が許容内。
- 必要横加速度が `ay_max` または専用しきい値以内。
- 必要操舵レートが `steer_rate_max` 以内。
- カーブ中の無理な回避ではない。

到達不可 gap は候補から除外する。

### Brake / Avoid Arbitration

前方リスクと reachable gap を同じ判断器で比較する。

- `EmergencyBrake` では原則回避に賭けず停止側へ倒す。
- `BrakePrepare` かつ reachable gap がある場合のみ `Overtake` または `LowSpeedAvoidance` へ進む。
- reachable gap がない場合は早めに追従/減速へ倒す。
- カーブ中は回避より減速を優先する。
- 停止/低速車両列は `LowSpeedAvoidance` に渡し、動く前走車は通常 `Overtake` または追従へ渡す。

## Config 要求

少なくとも次を config 化する。

```yaml
mpc:
  v2x_behavior_debug_log_enabled: false
  v2x_behavior_debug_log_period_sec: 1.0
  v2x_follow_gap_planner_enabled: false
  v2x_follow_gap_planner_no_gap_speed_limit_enabled: false
  v2x_follow_gap_planner_respect_overtake_forbidden: true
  v2x_front_risk_arbitration_enabled: false
  v2x_front_risk_brake_prepare_limit_enabled: false
  v2x_front_risk_avoid_candidate_limit_enabled: true
  v2x_front_risk_comfort_decel: 2.0
  v2x_front_risk_hard_decel: 4.0
  v2x_front_risk_emergency_decel: 6.0
  v2x_front_risk_distance_margin: 3.0
  v2x_front_risk_min_closing_speed: 0.5
  v2x_front_risk_prepare_time: 1.5

  v2x_overtake_guard_reachable_gap_enabled: false
  v2x_overtake_guard_min_prepare_distance: 8.0
  v2x_overtake_guard_max_lateral_shift: 1.2
  v2x_overtake_guard_max_lateral_accel: 2.0
  v2x_overtake_guard_min_gap_time: 0.8
  v2x_overtake_guard_min_speed_for_reachable: 1.0
  v2x_overtake_before_curve_enabled: false
  v2x_overtake_before_curve_max_front_speed: 8.0
  v2x_overtake_before_curve_min_speed_advantage: 1.0
  v2x_overtake_continue_in_forbidden_enabled: false
  v2x_overtake_front_velocity_limit_enabled: true
  v2x_overtake_gap_lookahead_distance: 0.0
  v2x_overtake_target_ramp_enabled: false
  v2x_overtake_target_ramp_ratio: 0.7
```

既定では既存挙動を壊さないため、追加 arbitration は `false` から始める。

## 安全要求

- 判定不能な場合は回避ではなく減速側へ倒す。
- V2X 情報が stale の場合は使わない。
- required decel が emergency 以上なら、gap が見えても無理な横回避へ入らない。
- reachable gap が成立しない場合は、MPC に横目標を渡さない。
- カーブ中は回避の優先度を下げる。
- 既存の `SafetyBrake` を弱めない。
- 実車では使用しない。シミュレータで gate / dev 系を確認してから扱う。

## 互換性要求

- 既定設定では現行の C++ MPC 挙動を維持する。
- `control_method=mpc` の launch 経路を維持する。
- `/v2x/vehicle_positions`、`/control/command/control_cmd`、`/localization/kinematic_state`、`/planning/scenario_planning/trajectory` の topic 契約を変えない。
- `aichallenge_system/` 側を変更しない。
- config key が未指定でも起動できる。

## 受け入れ条件

- `v2x_front_risk_arbitration_enabled=false` で現行挙動を維持する。
- 前走車との相対速度と距離から required decel を計算できる。
- required decel が高い場合、現行の距離/TTC guard より早く速度制限が入る。
- 前走車減速に対して、ブレーキ開始が遅れて衝突するケースが減る。
- reachable gap がない場合、無理に横回避へ入らない。
- reachable gap がある場合、ブレーキだけで詰まりすぎる前に回避準備へ入れる。
- 前方車検出距離が MPC horizon より長い場合でも、`v2x_overtake_gap_lookahead_distance` により追い越し候補を先読みできる。
- 長い lookahead 上で見つけた追い越し target を、`v2x_overtake_target_ramp_enabled` で現 horizon の target_ey に反映できる。
- `make autoware-build` が成功する。
- `make gate1`、`make gate2`、`make dev2`、`make dev3` の少なくとも代表ケースでログ理由を追える。
- `v2x_behavior_debug_log_enabled=true` で、同じ state に居続ける場合でも追い越し不可理由を周期ログで追える。

## 未確定事項

- V2X の速度推定をどこまで信頼するか。
- required decel のしきい値を `a_min` と連動させるか、専用 config にするか。
- reachable gap の横加速度計算をどこまで厳密にするか。
- 低速車両列の local path planner と通常 Overtake の境界。
- 速度差を付けた車両でレース戦略としてどこまで追従を許容するか。
