# V2X Front Risk Arbitration Tasklist

作成日: 2026-07-06
状態: Draft

## Definition of Done

- 前走車に対する required decel を計算できる。
- required decel に基づいて `Follow` / `BrakePrepare` / `AvoidCandidate` / `EmergencyBrake` 相当の判断ができる。
- 到達可能な gap だけを回避候補にできる。
- ブレーキ・追従・回避が同じ arbitration で決まる。
- 既定無効で現行挙動を維持できる。
- `make autoware-build` が成功する。
- `make gate1`、`make gate2`、`make dev2`、`make dev3` の代表ケースでログ確認できる。

## Phase 0: 現状整理

- [ ] 現在の `front_decel_guard`、`OvertakeGuard`、`LowSpeedAvoidance` の設定値を整理する。
- [ ] `make dev2` / `make dev3` で衝突するケースのログを保存する。
- [ ] 衝突直前の `front_distance`、自車速度、前走車速度、state transition を確認する。
- [ ] 速度差を付けた `domain_v_max` 条件を記録する。

## Phase 1: Required Decel Braking

- [x] `FrontRiskMetrics` を追加する。
- [x] 前方車から `relative_speed`、`available_distance`、`required_decel`、TTC を計算する。
- [x] `FrontRiskLevel` を追加する。
- [x] config に required decel しきい値を追加する。
- [x] `Follow` 時の速度上限を required decel ベースに切り替え可能にする。
- [x] `SafetyBrake` を required decel の emergency 判定でも発火できるようにする。
- [x] state transition log に `required_decel` と risk level を出す。
- [x] `make autoware-build` を実行する。
- [ ] `make gate1` で停止/減速の挙動を見る。

## Phase 2: Reachable Gap

- [x] 既存 `GapPlannerOutput` から first gap distance を取得する helper を整理する。
- [x] target lateral と current lateral から横移動量を計算する。
- [x] gap 到達時間を見積もる。
- [x] 必要横加速度を近似計算する。
- [x] config に reachable gap しきい値を追加する。
- [x] `overtake_guard_allows()` に reachable gap 判定を追加する。
- [x] 到達不能時の block reason をログに出す。
- [x] `make autoware-build` を実行する。
- [ ] `make gate2` で停止車回避への影響を見る。

## Phase 3: Brake / Avoid Arbitration

- [x] `evaluate_v2x_behavior()` の V2X 前方判断を risk -> gap -> state の順に整理する。
- [x] `EmergencyBrake` では gap planner へ賭けず停止側へ倒す。
- [ ] `BrakePrepare` では reachable gap がなければ速度上限を下げる。
- [x] `AvoidCandidate` では reachable gap があれば Overtake / LowSpeedAvoidance へ渡す。
- [x] カーブ中の動く前走車では回避より減速を優先する。
- [x] 低速/停止車は既存 local path planner を優先する。
- [x] 前方車検出距離が MPC horizon より長い場合に備え、Overtake の gap planner を先読みできるようにする。
- [x] 長い lookahead 上で見つけた追い越し target を、現 horizon の target_ey に ramp で反映する。
- [ ] state hold により安全側遷移が遅れないことを確認する。
- [x] `make autoware-build` を実行する。

## Phase 4: Scenario Verification

- [ ] `make gate1` で前方車検出時に止まれるか確認する。
- [ ] `make gate2` で停止車両を通過できるか確認する。
- [ ] `make dev2` で並走・追従・追い越しの挙動を見る。
- [ ] `make dev3` で速度差を付けた複数車両のカーブ追突を確認する。
- [ ] `domain_v_max` を同速に戻した場合に単独/混走が悪化しないか確認する。
- [ ] RViz の MPC 予測点が到達不能 gap へ向かっていないか確認する。
- [x] `v2x_behavior_debug_log_enabled` で同一 state 中の追い越し不可理由を周期ログに出せるようにする。
- [x] Overtake 中の front velocity limit を config で切り離せるようにする。
- [ ] `make dev2` で `reason` / `block` を確認し、追い越し不可の主因を分類する。
- [ ] `/control/command/control_cmd` の publish が維持されることを確認する。

## Phase 5: Documentation

- [ ] `docs/spec/mpc-integration.md` に final config と判断式を反映する。
- [ ] 既存の暫定 `front_decel_guard` と新 arbitration の関係を書く。
- [ ] チューニング手順を書く。
- [ ] gate/dev での確認結果をステアリング配下にメモする。

## チューニング候補

- `front_risk_distance_margin`
- `front_risk_brake_prepare_limit_enabled`
- `front_risk_avoid_candidate_limit_enabled`
- `front_risk_comfort_decel`
- `front_risk_hard_decel`
- `front_risk_emergency_decel`
- `front_risk_min_closing_speed`
- `v2x_overtake_guard_max_lateral_shift`
- `v2x_overtake_guard_max_lateral_accel`
- `v2x_overtake_guard_min_gap_time`
- `v2x_overtake_guard_min_prepare_distance`
- `v2x_overtake_gap_lookahead_distance`
- `v2x_overtake_target_ramp_ratio`
- `v2x_overtake_front_velocity_limit_enabled`

## 注意点

- 後続車の失速が出た場合は、required decel しきい値を上げるか、閉じ速度しきい値を上げる。
- 追突が出た場合は、distance margin を増やすか、comfort/hard decel の発火を早める。
- 回避でスリップする場合は、reachable gap の横加速度しきい値を下げる。
- 回避に入らない場合は、reachable gap の準備距離・横移動量・横加速度の block reason を確認する。
- カーブ中のクラッシュは、追い越し禁止と減速 arbitration の優先順位を確認する。
