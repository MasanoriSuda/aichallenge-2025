# Race V2X Follow And Overtake Tasklist

## Phase 0. Baseline And Evidence

- [ ] 現在の `make dev2` 起動構成を確認する。
- [ ] `make dev2` で `/v2x/vehicle_positions` が両車から見えるか確認する。
- [ ] 2 台同時走行時の `vehicle_id` と ROS_DOMAIN_ID の対応を記録する。
- [ ] `output/latest/d<N>/autoware.log` と rosbag の保存場所を確認する。
- [ ] Gate1 / Gate2 の最新合格状態を記録する。

## Phase 1. Requirements Freeze

- [x] race 用に `use_v2x_race_behavior` を追加するか、既存 `use_v2x_overtake` を mode 付きで使うか決める。
- [x] Gate2 専用 param と race 用 param の境界を決める。
- [ ] 2 台試走の初期条件を決める。
- [x] 先行側が yield する条件と速度 cap を決める。
- [ ] 個別車両 config を使えるか確認する。
- [x] `make race2` を追加するか、`make dev2` + launch arg で始めるか決める。

## Phase 2. Pure Python Planner

- [x] `V2XRaceBehaviorConfig` を定義する。
- [x] `V2XRaceBehaviorResult` を定義する。
- [x] `V2XRaceBehaviorPlanner` を追加する。
- [x] `/v2x/vehicle_positions` の snapshot 更新を実装する。
- [x] polyline projection で `signed_gap_m` と `lateral_offset_m` を求める。
- [x] `front_target`、`side_blocker`、`return_blocker` を分類する。
- [x] `yield_target` を分類する。
- [x] target lock と stale hold を実装する。
- [x] follow speed cap を実装する。
- [x] yield speed cap を実装する。
- [x] yield release / timeout / cooldown を実装する。
- [x] 先行車が大きく離れた場合の catch-up wait speed cap を実装する。
- [x] 横並び・斜め前の front conflict guard を実装する。
- [x] 遠方追走から追い越し開始距離に入る前の approach speed cap を実装する。
- [x] race2 ログに合わせて近距離追い越し開始と短い壁 horizon の設定へ調整する。
- [x] 狭所での壁接触ログに合わせて wall clearance を戻し、反対側への即時 side switch を抑止する。
- [x] overtake permission を実装する。
- [x] return permission を実装する。
- [x] cooldown を実装する。

## Phase 3. MPC Integration

- [x] `mpc_controller.py` に race behavior planner を接続する。
- [x] race behavior の速度 cap を `v_max` / `v_ref` に合成する。
- [x] race behavior の lateral offset を MPC path constraints に合成する。
- [x] yield 中は lateral offset を 0 または現ライン維持へ倒す。
- [x] Gate1 stop fallback の優先順位を整理する。
- [x] race が front target を処理中は Gate1 stop stale hold を bypass する。
- [x] Gate2 overtake と race behavior が同時 true の場合の扱いを実装する。
- [x] MPC infeasible / control fault 時の abort を race behavior に接続する。
- [x] throttled log を追加する。
- [x] `V2X stop` active result の nullable gap / lateral を安全に log する。

## Phase 4. Config And Launch

- [x] `config.yaml` に `v2x_race_behavior` section を追加する。
- [x] `sim_config.yaml` に `v2x_race_behavior` section を追加する。
- [x] `control/mpc.launch.xml` に `use_v2x_race_behavior` arg / param を追加する。
- [x] `multi_purpose_mpc_ros/launch/mpc_controller.launch.py` に launch argument を追加する。
- [x] `run_autoware.bash` で race2 用の有効化方法を追加する。
- [x] 必要なら `Makefile` に `race2` を追加する。
- [x] 既存 `make gate1` / `make gate2` の起動引数が変わらないことを確認する。

## Phase 5. Unit Tests

- [x] 前方 target を follow 対象にするテストを追加する。
- [x] yaw-based front conflict fallback のテストを追加する。
- [x] yaw-based rear target を front follow に誤分類しないテストを追加する。
- [x] front conflict lateral guard のテストを追加する。
- [x] approach speed cap のテストを追加する。
- [ ] 後方 target を無視するテストを追加する。
- [ ] 横方向に遠い target を無視するテストを追加する。
- [x] target speed が遅い場合に follow speed cap が下がるテストを追加する。
- [x] gap が十分あり side が safe の場合に overtake permission が true になるテストを追加する。
- [x] side blocker がいる場合に overtake permission が false になるテストを追加する。
- [ ] wall clearance 不足で overtake permission が false になるテストを追加する。
- [x] return blocker がいる場合に復帰しないテストを追加する。
- [x] 後方接近 target がいる場合に yield speed cap へ入るテストを追加する。
- [x] 後方 target が追い越し side へ入った場合に先行側が同じ side へ寄らないテストを追加する。
- [x] yield release gap を超えたら yield を解除するテストを追加する。
- [ ] yield timeout で通常状態へ戻るテストを追加する。
- [x] cooldown 中に再追い越ししないテストを追加する。
- [x] 後続が大きく離れた場合に catch-up wait speed cap へ入るテストを追加する。
- [x] 後続が catchup release gap まで詰めたら解除するテストを追加する。
- [ ] stale target hold と timeout release のテストを追加する。

## Phase 6. Validation

- [x] `python3 -m py_compile` を実行する。
- [x] 対象 package の unit test を実行する。
- [x] YAML load check を実行する。
- [x] launch XML parse check を実行する。
- [x] `git diff --check` を実行する。
- [x] `make autoware-build` を実行する。
- [ ] `make gate1` を実行し、停止挙動が退行していないことを確認する。
- [ ] `make gate2` を実行し、NPC 追い越しが退行していないことを確認する。
- [ ] `make dev2` または `make race2` を実行する。
- [ ] Trial A: 片方が先行車を追走できることを確認する。
- [ ] Trial A: 片方が先行車を安全に追い越せることを確認する。
- [ ] Trial B: 先行側が後方接近を見て yield speed cap へ入ることを確認する。
- [ ] Trial B: yield 中の先行側が追い越し side へ寄らないことを確認する。
- [ ] Trial C: 両車 race behavior 有効時に同時横移動しないことを確認する。
- [ ] Trial D: 交互追い越しで接触、壁接触、制御スパイクがないことを確認する。
- [ ] `/control/command/control_cmd` の速度、加速度、操舵角がスパイクしないことを確認する。
- [ ] `/mpc/ref_path` または `/mpc/prediction` で横移動と復帰を確認する。
- [ ] result JSON で collision、wall contact、timeout の状態を確認する。

## Phase 7. Documentation

- [x] `docs/spec/v2x-multivehicle.md` に race behavior 方針を追記する。
- [x] `docs/spec/mpc-integration.md` に `use_v2x_race_behavior` と config を追記する。
- [x] `docs/spec/safety-gates.md` に Gate2 と race behavior の境界を追記する。
- [ ] `docs/interface/participant-interface.md` の契約変更が不要か確認する。

## Definition Of Done

- [ ] Gate1 が退行していない。
- [ ] Gate2 が退行していない。
- [ ] 2 台同時走行で V2X target を認識できる。
- [ ] 追い越し不可時に追走できる。
- [ ] 先行側が後方接近を見て yield できる。
- [ ] 追い越し可能時に lateral behavior へ入る。
- [ ] 追い越し後に通常ラインへ復帰できる。
- [ ] 交互追い越しで同時横移動、接触、壁接触がない。
- [ ] race behavior の判断根拠が log / rosbag で説明できる。
- [ ] 既存 topic / service / result JSON 契約を変更していない。
