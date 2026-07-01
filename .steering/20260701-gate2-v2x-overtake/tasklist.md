# Gate2 V2X Overtake Tasklist

## Phase 0. Baseline

- [x] 既存変更を `git status --short` で確認し、今回の変更範囲を分ける。
- [ ] `make gate1` が現在通ることを確認する。
- [ ] `make gate2` の現状失敗ログを保存する。
- [ ] Gate2 中に `/v2x/vehicle_positions` が publish されるか確認する。
- [ ] Gate2 の NPC `vehicle_id`、更新周期、座標系、初期位置をログから確認する。
- [ ] `output/latest/d<N>/autoware.log` と result JSON の失敗理由を記録する。

## Phase 1. Pure Logic

- [x] `v2x_overtake_planner.py` を追加する。
- [x] config dataclass または軽量設定オブジェクトを定義する。
- [x] Gate1 の前方対象選択ロジックを再利用または共通化する。
- [x] target speed / relative speed 推定を `V2XVehicleTracker` から受け取る。
- [x] 追い越し状態機械を実装する。
- [x] preferred side と fallback side の判定を実装する。
- [x] 両側 safe 時に wall margin が大きい側を選ぶ判定を実装する。
- [x] margin 差が小さい場合だけ `preferred_side` を tie-breaker にする。
- [x] 追い越し開始後は `forced_side` を維持し、左右を切り替えない。
- [x] lateral clearance の簡易判定を実装する。
- [x] border cell / path constraints 由来の wall clearance 判定を実装する。
- [x] side ごとの `left_available_m` / `right_available_m` と required clearance を算出する。
- [x] speed cap 計算を実装する。
- [x] abort / return hysteresis を実装する。
- [x] stale timeout を実装する。

## Phase 2. Unit Tests

- [x] 前方対象を追い越し候補にするテストを追加する。
- [x] 後方対象を無視するテストを追加する。
- [x] 横方向に遠い対象を無視するテストを追加する。
- [x] stale sample では追い越し許可しないテストを追加する。
- [x] start gap 範囲外では follow / stop になるテストを追加する。
- [x] preferred side が空いている場合にその側を選ぶテストを追加する。
- [x] 右側 wall margin が大きい場合、preferred が左でも右を選ぶテストを追加する。
- [x] wall margin 差が小さい場合、preferred side を選ぶテストを追加する。
- [x] `side_selection_policy=preferred` では従来の preferred 優先になるテストを追加する。
- [x] 追い越し開始後に反対側が広くなっても forced side を切り替えないテストを追加する。
- [x] preferred side が塞がっている場合に反対側または follow を選ぶテストを追加する。
- [x] wall clearance が不足する側を unsafe にするテストを追加する。
- [x] 両側 safe の場合に `preferred_side` または margin が大きい側を選ぶテストを追加する。
- [x] 両側 unsafe の場合に overtake を許可しないテストを追加する。
- [x] abort gap 以下では abort / stop になるテストを追加する。
- [x] return clearance 到達で `return_to_line` へ遷移するテストを追加する。
- [x] state hysteresis により状態がチャタリングしないテストを追加する。

## Phase 3. MPC Integration

- [x] `mpc_controller.py` に `use_v2x_overtake` parameter を追加する。
- [x] `V2XOvertakePlanner` を V2X callback と control loop に接続する。
- [x] Gate1 の `V2XStopPlanner` と overtake planner の優先順位を整理する。
- [x] overtake active 中の speed cap 合成を実装する。
- [x] lateral offset を既存参照パスへ安全に合成する方法を実装する。
- [x] lateral offset 合成前に `min_wall_clearance_m` を満たすか確認する。
- [x] 参照パス原本を保持し、clear / abort 時に復帰できるようにする。
- [x] lateral offset の rate limit を実装する。
- [x] Gate2 中だけ MPC 操舵レート上限を引き上げる設定を追加する。
- [x] horizon 前半で追い越し側 corridor へ遷移する制約 transition を追加する。
- [x] 追い越し中の短時間 V2X target loss では lateral offset を保持する。
- [x] V2X target 検出前から右側へ寄せる standby lateral offset を追加する。
- [x] 実際の `ey` が右側へ入るまで prepare speed cap を適用する。
- [x] Gate2 低速時だけ選択 side へ最低操舵角を入れる steer override を追加する。
- [ ] MPC infeasible または制御出力異常時の abort を実装する。
- [x] throttled log を追加する。

## Phase 4. Config And Launch

- [x] `config.yaml` に `v2x_overtake` section を追加する。
- [x] `sim_config.yaml` に `v2x_overtake` section を追加する。
- [x] `side_selection_policy` と `side_margin_tie_threshold_m` を設定に追加する。
- [x] `constraint_transition_horizon_ratio`、`constraint_initial_progress`、`overtake_steer_rate_max` を設定に追加する。
- [x] `target_lost_hold_sec` を設定に追加する。
- [x] `standby_lateral_offset_m` と `prepare_speed_cap_kmph` を設定に追加する。
- [x] `steer_override_enabled`、`steer_override_min_abs_rad`、`steer_override_until_ey_m` を設定に追加する。
- [x] `control/mpc.launch.xml` に `use_v2x_overtake` arg / param を追加する。
- [x] `multi_purpose_mpc_ros/launch/mpc_controller.launch.py` に launch argument を追加する。
- [x] `make gate2` で Gate2 用に `use_v2x_overtake=true` を渡す方法を確認する。
- [x] Gate1 では `use_v2x_overtake=false` のままになることを確認する。

## Phase 5. Verification

- [x] `python3 -m py_compile` で追加 Python を確認する。
- [x] 対象 package の unit test を実行する。
- [x] YAML load check を実行する。
- [x] launch XML parse check を実行する。
- [x] `git diff --check` を実行する。
- [x] `make autoware-build` を実行する。
- [ ] `make gate1` を実行し、停止挙動が退行していないことを確認する。
- [ ] `make gate2` を実行する。
- [ ] `output/latest/d<N>/autoware.log` から selected target / side / offset / speed cap を確認する。
- [ ] `/control/command/control_cmd` のステア角、速度、加速度がスパイクしないことを確認する。
- [ ] `/mpc/ref_path` または `/mpc/prediction` で横方向回避と復帰を確認する。
- [ ] horizon 内の wall clearance が閾値未満の区間で overtake が許可されないことを確認する。
- [ ] result JSON で collision、wall contact、timeout の状態を確認する。

## Phase 6. Documentation

- [x] `docs/spec/safety-gates.md` に Gate2 実装方針と確認 topic を追記する。
- [x] `docs/spec/mpc-integration.md` に `use_v2x_overtake` と `v2x_overtake` config を追記する。
- [x] `docs/spec/v2x-multivehicle.md` に Gate2 から race behavior へ流用する方針を追記する。
- [x] `docs/interface/participant-interface.md` に契約変更が不要か確認する。

## Definition Of Done

- [ ] Gate1 が退行していない。
- [ ] Gate2 で前方 NPC を検出できる。
- [ ] 追い越し可能な場合に lateral behavior へ入る。
- [ ] 追い越し中に接触、壁接触、コース逸脱がない。
- [ ] 追い越し候補ラインが `min_wall_clearance_m` を満たす場合だけ許可される。
- [ ] 追い越し後に通常参照ラインへ復帰できる。
- [ ] 追い越し不可時は停止・追走へフォールバックできる。
- [ ] Gate2 の判断根拠が log / rosbag で説明できる。
- [x] 既存 topic / service / result JSON 契約を変更していない。
