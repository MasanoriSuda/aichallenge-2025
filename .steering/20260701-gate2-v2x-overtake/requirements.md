# Gate2 V2X Overtake Requirements

## Background

Automotive AI Challenge 2026 の Gate2 は NPC 追い越しを対象にする。2026-07-01 時点の運営回答により、障害物・他車両情報の正入力は `/v2x/vehicle_positions` のみを使用する。

Gate1 では同じ V2X 入力から前方対象を検出し、追い越しせずに停止する縦方向レイヤを実装した。Gate2 では、その検出結果を再利用し、追い越し可能な場合だけ横方向回避へ分岐する。

## Goal

MPC 制御で走行中、V2X 上の前方 NPC または低速車両に対して、追い越し可能なら安全な横方向オフセットを生成して追い越し、追い越し後に通常走行ラインへ復帰する。

Gate2 の最小ゴール:

1. `/v2x/vehicle_positions` から前方の追い越し対象を検出する。
2. 対象が停止対象なのか、低速先行車なのか、横方向に関係ない車両なのかを分類する。
3. 追い越し可能な距離、相対速度、横方向余裕がある場合だけ overtake mode に入る。
4. 追い越し不可または危険距離では Gate1 の停止・減速へフォールバックする。
5. 追い越し後は通常参照ラインへ復帰する。
6. 判断理由を log / rosbag から追える。

## Scope

変更してよい主対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`
- `aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/control/mpc.launch.xml`
- `docs/spec/` と `docs/interface/` の追記

変更しない対象:

- `aichallenge/workspace/src/aichallenge_system/`
- `/v2x/vehicle_positions` の topic 名、message 型、fanout 設計
- `/control/command/control_cmd`
- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- result JSON schema
- Gate1 の停止安全性

## Functional Requirements

### R1. V2X input

- Gate2 の追い越し判断は `/v2x/vehicle_positions` のみを入力にする。
- message は現行ローカルの `v2x_msgs/V2XVehiclePositionArray` を前提にする。
- `vehicle_id`、`position`、`header.stamp` を使用する。
- 相対速度は既存 `V2XVehicleTracker` の有限差分推定を使う。
- 自車 echo、NaN / Inf、座標ジャンプ、stale sample は追い越し判断から除外する。

### R2. Target classification

- 自車参照経路上で前方にいる対象だけを追い越し候補にする。
- 横方向コリドー外の車両は追い越し対象にしない。
- 複数候補がある場合は、進行方向距離が最短の対象を優先する。
- 対象の forward gap、lateral offset、relative speed、estimated target speed を記録する。
- 対象が動き出した場合は、停止障害物ではなく先行車追走または追い越し候補として扱えるようにする。

### R3. Overtake permission

以下をすべて満たす場合だけ追い越し許可を出す。

- `gap_m` が `min_overtake_start_gap_m` 以上、`max_overtake_start_gap_m` 以下。
- `time_to_collision_sec` または距離余裕が危険閾値を下回っていない。
- 横方向に `lateral_clearance_m` 以上の回避余裕がある。
- 追い越し側の候補ラインが参照パス幅 `reference_path.max_width` の内側に収まる。
- 追い越し側の候補ラインから壁・コース境界まで `min_wall_clearance_m` 以上の余裕がある。
- 追い越し中に別の V2X 対象へ近づきすぎない。
- MPC の速度、操舵角、操舵レート制約内で横移動できる。

追い越し不可の場合:

- 先行車が近い場合は Gate1 の `v2x_stop` による停止・減速へ戻す。
- 先行車が動いている場合は、暫定的に距離ベースの速度 cap で追走する。

### R4. Overtake behavior

- 追い越しは状態機械で管理する。
- 初期の side selection は、左右のうち wall margin が大きい安全側を基本にする。
- wall margin 差が `side_margin_tie_threshold_m` 以下の場合だけ `preferred_side` を tie-breaker にする。
- 追い越し開始後は選択済み side を維持し、走行中に左右を切り替えない。
- 選択済み side が unsafe になった場合は反対側へ切り替えず abort / follow に倒す。
- 両側 unsafe の場合は追い越しせず停止・追走に倒す。
- 横方向オフセットは急に切り替えず、時間または進行方向距離に対して滑らかに増減させる。
- 追い越し中の速度は通常の `ref_vel.yaml` / `v_max` より低い専用 cap を使う。

### R5. Return behavior

- 追い越し対象より `return_clearance_m` 以上前方へ出たら復帰候補にする。
- 復帰先の前方に別車両がいる場合は復帰しない。
- 復帰時も lateral offset を滑らかに 0 へ戻す。
- 復帰完了後は Gate1 の通常 V2X stop / follow 監視だけを残し、overtake state を clear する。

### R6. Fail-safe

- V2X 欠損や stale が起きた場合、追い越し中でも急操舵で戻さない。
- 追い越し中に対象位置が不明になった場合は、短時間は現在の lateral offset を維持し、速度 cap を下げる。
- 危険距離、壁接触リスク、MPC infeasible、制御出力異常を検出したら abort し、停止・減速へ倒す。
- 壁距離が `min_wall_clearance_m` を下回る候補ラインは追い越し許可しない。
- Gate2 実装によって Gate1 の停止挙動を弱めない。

### R7. Observability

最低限、以下を throttled log で出す。

- overtake state
- selected `vehicle_id`
- `gap_m`
- `lateral_offset_m`
- `relative_speed_mps`
- selected side
- target lateral offset
- wall clearance
- speed cap
- permit / deny / abort reason

Gate2 解析では以下 topic を rosbag / log で確認できること。

- `/v2x/vehicle_positions`
- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- `/control/command/control_cmd`
- `/mpc/ref_path`
- `/mpc/prediction`

## Non-Goals

- Gate2 では学習ベースの画像 / LiDAR 認識は扱わない。
- Gate2 初期実装では最速ライン最適化まで狙わない。
- `domain_bridge` や独自クロスドメイントピックは追加しない。
- 評価基盤の result JSON schema は変更しない。
- 実車向けの高速度追い越しは、シミュレータ Gate2 通過後に別ステアリングで扱う。

## Definition Of Done

- `make autoware-build` が通る。
- 追加した pure Python 判断ロジックに単体テストがある。
- `make gate1` が退行しない。
- `make gate2` で前方 NPC を認識し、接触・壁接触・コース逸脱なしに追い越す。
- 追い越し後に通常参照ラインへ復帰する。
- 追い越し不可時は停止・追走へフォールバックする。
- `output/latest/d<N>/autoware.log` から追い越し判断の根拠を追える。
- 仕様変更があれば `docs/spec/safety-gates.md` または `docs/spec/mpc-integration.md` に反映されている。
