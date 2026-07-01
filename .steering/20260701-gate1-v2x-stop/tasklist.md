# Gate1 V2X Stop Tasklist

## Phase 0. Baseline

- [ ] 既存変更を `git status --short` で確認し、今回の変更範囲を分ける。
- [ ] `make gate1` の現在の失敗ログを保存する。
- [ ] `/v2x/vehicle_positions` が Gate1 中に publish されるか確認する。
- [ ] message 内に自車 `vehicle_id` が含まれるか確認する。

## Phase 1. Pure Logic

- [x] `v2x_stop_planner.py` を追加する。
- [x] config dataclass または軽量設定オブジェクトを定義する。
- [x] 前方対象選択を実装する。
- [x] self echo 除外を実装する。
- [x] speed cap 計算を実装する。
- [x] hold/release hysteresis を実装する。
- [x] stale timeout を実装する。

## Phase 2. Unit Tests

- [x] 前方対象を検出するテストを追加する。
- [x] 後方対象を無視するテストを追加する。
- [x] 横方向に遠い対象を無視するテストを追加する。
- [x] self echo を無視するテストを追加する。
- [x] gap に応じて speed cap が下がるテストを追加する。
- [x] target stop gap 以下で speed cap が 0 になるテストを追加する。
- [x] stale timeout と hold/release のテストを追加する。

## Phase 3. MPC Integration

- [x] `mpc_controller.py` に `use_v2x_stop` parameter を追加する。
- [x] `use_obstacle_avoidance=false` でも `/v2x/vehicle_positions` を購読できるようにする。
- [x] 既存 `V2XVehicleTracker` と `V2XStopPlanner` を接続する。
- [x] `ref_vel_configulator` の速度上限と V2X speed cap を `min()` で合成する。
- [x] 停止時に急な NaN / Inf / 負速度が出ないよう clamp する。
- [x] throttled log を追加する。

## Phase 4. Config And Launch

- [x] `config.yaml` に `v2x_stop` section を追加する。
- [x] `sim_config.yaml` に `v2x_stop` section を追加する。
- [x] `control/mpc.launch.xml` に `use_v2x_stop` arg / param を追加する。
- [x] `use_obstacle_avoidance` と `use_v2x_stop` の役割差をコメントまたは docs に残す。

## Phase 5. Verification

- [x] `python3 -m py_compile` で追加 Python を確認する。
- [x] 対象 package の unit test を実行する。
- [x] `make autoware-build` を実行する。
- [ ] `make gate1` を実行する。
- [ ] `output/latest/d<N>/autoware.log` から selected target / gap / speed cap を確認する。
- [ ] `/control/command/control_cmd` が停止へ落ちることを確認する。
- [ ] result JSON で接触、wall、timeout の状態を確認する。

## Phase 6. Documentation

- [x] `docs/spec/safety-gates.md` に Gate1 実装方針と確認 topic を追記する。
- [x] `docs/spec/mpc-integration.md` に `use_v2x_stop` と `v2x_stop` config を追記する。
- [x] `docs/interface/participant-interface.md` に契約変更が不要か確認する。

## Definition Of Done

- [ ] Gate1 で前方対象に対して停止できる。
- [ ] 停止後に不要な再加速をしない。
- [ ] Gate1 の停止根拠が log / rosbag で説明できる。
- [ ] 通常走行時に V2X 未受信でも制御が止まらない。
- [ ] 既存 topic / service / result JSON 契約を変更していない。
