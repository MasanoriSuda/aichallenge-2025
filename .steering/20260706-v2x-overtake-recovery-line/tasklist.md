# V2X Overtake / Recovery Line Tasklist

作成日: 2026-07-06
状態: Implemented / gate2 verified

## Definition of Done

- 追い越し開始後の横目標が滑らかな U 字に近くなる。
- `Overtake` 中に pass side が頻繁に反転しない。
- 前走車を抜き切った後に `Return` 相当の復帰フェーズがある。
- 回避不能・壁側・emergency では `Recovery` または減速へ倒せる。
- `v2x_overtake_line_enabled=false` で現行挙動を維持できる。
- `make autoware-build` が成功する。
- gate2 を壊さず、dev2/dev3 で phase と理由をログ確認できる。

## Phase 0: 現状ログ整理

- [ ] 最新の dev2/dev3 で、追い越し時の `V2X debug` ログを保存する。
- [ ] S 字になるケースの `wp_id`、state、pass side、front distance を記録する。
- [ ] ヘアピンで壁へ向かうケースの `wp_id` と curve guard 状態を記録する。
- [ ] gate2 が通る config を保存する。

## Phase 1: Config / State 追加

- [x] `OvertakeLinePhase` を追加する。
- [x] `OvertakeLineConfig` を追加する。
- [x] `OvertakeLineState` を追加する。
- [x] `config.yaml` に `v2x_overtake_line_*` を追加する。
- [x] YAML パースを追加する。
- [x] 起動ログに enabled と主要パラメータを出す。
- [x] `v2x_overtake_line_enabled=false` で現行挙動が変わらないことを確認する。
- [x] `make autoware-build` を実行する。

## Phase 2: ShiftOut / Pass

- [x] `Overtake` 開始時に pass side を lock する。
- [x] `ShiftOut` phase を追加する。
- [x] smoothstep で横目標列 `target_ey[i]` を作る。
- [x] path constraints の `lb/ub` と `min_wall_clearance` で target を clip する。
- [x] 横加速度見積もりがしきい値を超える場合は target 変化を clamp する。
- [x] `ShiftOut -> Pass` の遷移条件を入れる。
- [x] `Pass` 中は target を維持し、pass side を反転しない。
- [x] `make autoware-build` を実行する。

## Phase 3: Return

- [x] 前走車を抜き切った判定を定義する。
- [x] `return_clear_distance` を config 化する。
- [x] `Pass -> Return` の遷移を入れる。
- [x] Return 中は基準 trajectory 側へ smoothstep で戻す。
- [x] Return 中の急復帰を横加速度ガードで抑制する。
- [x] Return 完了後に phase と pass side lock を解除する。
- [x] `make autoware-build` を実行する。

## Phase 4: Recovery

- [x] emergency front risk で SafetyBrake 優先にする。
- [x] target が `lb/ub` 端へ張り付く場合は target を clip する。
- [ ] side vehicle が割り込む場合の Recovery 条件を入れる。
- [x] gap 喪失時の Recovery 条件を入れる。
- [ ] Recovery 中は新規 Overtake を一定時間抑制する。
- [x] Recovery 中は速度上限を下げる。
- [x] `make autoware-build` を実行する。

## Phase 5: Debug

- [x] phase 遷移ログを追加する。
- [x] 周期 debug に phase、target_ey、current_ey、required lateral accel を出す。
- [ ] block reason / recovery reason をログへ出す。
- [ ] RViz marker が必要か判断する。

## Phase 6: Verification

- [x] `make gate2` で停止車両追い越しが悪化しないか確認する。
- [ ] `make dev2` で低速車を高速車が追い越し試行するか確認する。
- [ ] `make dev3` でヘアピン前後の並走・追走・復帰を見る。
- [ ] 直線追い越しの MPC 予測点が S 字ではなく U 字に近いか確認する。
- [ ] ヘアピン入口では OvertakeLine が始動しないことを確認する。
- [ ] 追い越し後に壁側へ残らないことを確認する。
- [ ] `/control/command/control_cmd` の publish 周期が維持されることを確認する。

## Phase 7: Documentation

- [x] `docs/spec/mpc-integration.md` に OvertakeLine の設計を反映する。
- [ ] `v2x-front-risk-arbitration` との関係を書く。
- [ ] チューニング手順を書く。
- [ ] gate/dev の確認結果をこの steering 配下に追記する。

## 実装メモ

- 2026-07-06: `mpc_controller_cpp.cpp` に `OvertakeLinePhase` / `OvertakeLineConfig` / `OvertakeLineState` を追加。
- 2026-07-06: `v2x_overtake_line_enabled=false` を既定にし、enabled 時だけ旧 overtake side target より優先して `xr[e_y]` を生成する。
- 2026-07-06: `LowSpeedAvoidance` と `SafetyBrake` は OvertakeLine より優先するため、gate2 local path には介入しない。
- 2026-07-06: `make autoware-build` 成功。走行検証は未実施。
- 2026-07-06: gate2 失敗を受け、`LowSpeedAvoidance` に入る前の停止/低速前方車も OvertakeLine 対象外にした。
- 2026-07-06: gate2 失敗ログでは、`LowSpeedAvoidance` 開始後に `v2x_low_speed_avoidance_min_prepare_distance` を下回って候補から外れ、SafetyBrake で停止していた。継続中かつ local path/gap が feasible な場合は、SafetyBrake / front risk emergency より LowSpeedAvoidance を保持するようにした。
- 2026-07-06: `make autoware-build` 成功。`make gate2` は `all_passed=true`。
- 2026-07-06: moving front 追従で `Follow` に落ちると lateral target が消えて真後ろへ戻るため、WP 明示禁止ではない soft curve forbidden では、内側へ刺さない側かつ前方距離が残る場合に `Follow` preposition を許可した。
- 2026-07-06: `v2x_follow_preposition_offset=0.9`、`target_bias=0.4`、`ramp_ratio=1.4` に調整し、Follow 中でもMPC予測点に追い越し準備の横方向意思が出るようにした。`make autoware-build` 成功。
- 2026-07-06: 最新ログでは `pass=-1` が soft curve forbidden の内側側に見えており、preposition が条件で落ちていた。Follow preposition 用の pass side を通常 Overtake 判定と分離し、soft curve forbidden ではカーブ外側を優先するようにした。
- 2026-07-06: 最新ログでは `fd=15.30` の時点で reachable な右側 fallback があったが、`v2x_overtake_guard_max_lateral_shift=2.2` により `shift=3.34` で拒否され、`fd=10.07` まで詰まってから Overtake に入り SafetyBrake へ戻されていた。絶対横移動量ガードを `0.0` で無効化し、時間込みの `v2x_overtake_guard_max_lateral_accel` に一本化した。
- 2026-07-06: 最新ログでは OvertakeLine 有効後も `fd=1.8-2.0` まで詰まった状態で `overtake fallback guard front distance, min=5` により追い越し横 target が止まっていた。相対速度が小さく横余裕がある close-follow 状態だけ `v2x_overtake_close_follow_*` で side target を許可する例外を追加した。

## チューニング候補

- `v2x_overtake_line_shift_distance`
- `v2x_overtake_line_pass_distance`
- `v2x_overtake_line_return_distance`
- `v2x_overtake_line_lateral_offset`
- `v2x_overtake_line_target_bias`
- `v2x_overtake_line_min_wall_clearance`
- `v2x_overtake_line_max_lateral_accel`
- `v2x_overtake_line_max_target_change`
- `v2x_overtake_line_return_clear_distance`
- `v2x_overtake_line_phase_hold_time`
- 既存 `v2x_overtake_guard_min_prepare_distance`
- 既存 `v2x_overtake_guard_max_lateral_accel`
- 既存 `v2x_overtake_start_curve_clearance_distance`

## 注意点

- gate2 は停止車両列なので、通常 OvertakeLine より既存 LowSpeedAvoidance を優先する。
- ヘアピンで内側へ飛び込む挙動は、ライン生成より先に開始条件で止める。
- 横目標を強くしすぎると壁へ吸われるため、初期 `target_bias` は控えめにする。
- `Follow` preposition と OvertakeLine が二重に効く場合は、OvertakeLine 有効時に preposition を弱める。
- うまく抜けない場合でも、まず phase ログで「開始不可」「ShiftOut中」「Return不可」を分類する。
