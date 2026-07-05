# multi_purpose_mpc_ros

このパッケージはリポジトリ内（`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`）に直接収録されています。別途 `git clone` は不要です。

## build

autoware コンテナ内で実行します（`make autoware-bash` または `make autoware-build`）：

```bash
cd /aichallenge/workspace
colcon build --symlink-install --allow-overriding gyro_odometer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 通常の MPC 実行系は C++ executable `mpc_controller_cpp` です。
- Python 補助スクリプト用に、ビルド時に仮想環境が `${ROS_WS}/install/multi_purpose_mpc_ros/.venv` に作成されます。

## run

### MPC コントローラー
```bash
ros2 run multi_purpose_mpc_ros mpc_controller_cpp \
  --config_path $(ros2 pkg prefix --share multi_purpose_mpc_ros)/config/config.yaml \
  --ref_vel_path $(ros2 pkg prefix --share multi_purpose_mpc_ros)/config/ref_vel.yaml
```

Python 版は比較・検証用に残しています。

```bash
ros2 run multi_purpose_mpc_ros run_mpc_controller.bash
```

#### Domain-specific v_max

`mpc.domain_v_max` を設定すると、`ROS_DOMAIN_ID` ごとに最高車速を km/h で上書きできます。`domain_v_max` に該当 Domain がない場合は `v_max` がそのまま使われます。

```yaml
mpc:
  v_max: 40.0
  domain_v_max:
    1: 40.0
    2: 38.0
    3: 36.0
```

#### Wall-edge trajectory tuning

`config/config.yaml` の `mpc.wp_id_low_offset` / `mpc.wp_id_low_speed` / `mpc.wp_id_offset` で、低速時だけ参照 waypoint の先読み量を変えられます。`wp_id_low_speed` は km/h で指定します。

```yaml
mpc:
  wp_id_low_offset: 1
  wp_id_low_speed: 5.0
  wp_id_offset: 2
```

`config/config.yaml` の `mpc.center_bias` と `mpc.safety_margin_scale` で、編集済み trajectory への追従と壁際の余白を調整できます。

```yaml
mpc:
  center_bias: 0.0          # 0.0 = trajectory追従, 1.0 = 制約中央寄せ
  safety_margin_scale: 1.0 # 0.0 = 追加marginなし, 1.0 = 現行margin
```

まず `center_bias: 0.0`, `safety_margin_scale: 1.0` で中央寄せだけを消し、必要に応じて `safety_margin_scale` を `0.7`, `0.5`, `0.3` の順に下げて確認します。

#### V2X gap planner

C++ MPC には `/v2x/vehicle_positions` から他車位置を受け取り、reference path 上の横方向 gap を選んで `lb/ub` と `xr` に反映する rule-based planner があります。既定では無効です。

```yaml
mpc:
  use_v2x_gap_planner: false
  v2x_vehicle_radius: 1.6
  v2x_vehicle_length: 2.0
  v2x_prediction_margin: 0.2
  v2x_timeout_sec: 1.0
  gap_min_width: 1.8
  gap_target_bias: 1.0
  no_gap_target_velocity: 0.0
  v2x_wall_clearance_margin: 0.0
  v2x_vehicle_side_target_margin: 0.0
  v2x_wall_avoidance_bias: 0.0
  v2x_vehicle_vehicle_gap_enabled: true
  v2x_vehicle_vehicle_gap_min_distance: 0.0
  v2x_vehicle_vehicle_gap_min_width: 0.0
  v2x_multi_front_gap_enabled: true
  v2x_multi_front_gap_distance: 0.0
  v2x_low_speed_pass_side: auto      # auto, left, right
  v2x_low_speed_pass_ramp_ratio: 1.0
```

`V2XVehiclePositionArray` は位置と covariance のみを持つため、C++ 側では vehicle_id ごとの直近2点から速度を推定します。初回、異常ジャンプ、過大速度時は静止障害物として扱います。`v2x_vehicle_radius` は V2X 車両単体の半幅ではなく、自車中心が入ってはいけない横方向禁止幅です。V2X 車両幅 1.45m の場合、相手半幅 0.725m + 自車半幅 0.725m として 1.45m 程度を基準にし、必要に応じて余白を足します。前後方向の占有判定には `v2x_vehicle_length` と自車長を使います。gap がない場合は `no_gap_target_velocity` を速度上限として使います。

free gap が壁と他車に挟まれている場合、`v2x_wall_clearance_margin` で壁側の制約を内側へ削り、`v2x_wall_avoidance_bias` で target を gap 中央から車側へ寄せられます。壁ペナルティを避けたい場合は `v2x_wall_clearance_margin: 0.4`, `v2x_vehicle_side_target_margin: 0.2`, `v2x_wall_avoidance_bias: 0.8` から確認します。

左右が両方とも V2X 車両の free gap は、`v2x_vehicle_vehicle_gap_enabled: false` で候補から外せます。3台同時走行のスタート直後など、前方2台の間をすり抜けようとして操舵が不安定になる場合は、車-車 gap を禁止し、壁-車 gap または no-gap 低速追走へ倒します。

さらに、前方近距離に2台以上いる状況そのものを追い抜き禁止にしたい場合は、`v2x_multi_front_gap_enabled: false` と `v2x_multi_front_gap_distance` を設定します。この条件に入ると gap planner は横目標を作らず、`no_gap_target_velocity` による低速追走へ倒します。

#### V2X behavior FSM

`use_v2x_behavior_fsm: true` にすると、`Cruise` / `Follow` / `Overtake` / `LowSpeedAvoidance` / `SafetyBrake` の最小状態で `V2XGapPlanner` の使用可否を制御します。`Follow` と `SafetyBrake` では gap planner を使わず、速度上限を下げます。`Follow` 中は `v2x_follow_velocity` と前方距離から計算した停止可能速度の小さい方を使います。`Overtake` と `LowSpeedAvoidance` のときだけ gap planner が `lb/ub` と `xr` に反映されます。

```yaml
mpc:
  use_v2x_behavior_fsm: false
  v2x_follow_distance: 8.0
  v2x_safety_brake_distance: 3.0
  v2x_safety_brake_margin: 2.0
  v2x_follow_velocity: 5.0
  v2x_safety_brake_velocity: 0.0
  v2x_overtake_min_gap_width: 2.0
  v2x_overtake_max_curvature: 0.05
  v2x_require_gap_for_overtake: true
  v2x_low_speed_avoidance_enabled: false
  v2x_low_speed_avoidance_distance: 10.0
  v2x_low_speed_avoidance_lookahead_distance: 18.0
  v2x_low_speed_avoidance_velocity: 1.5
  v2x_low_speed_avoidance_max_front_speed: 1.0  # 予備設定。開始条件の hard gate にはしない。
  v2x_low_speed_avoidance_min_gap_width: 0.5
  v2x_low_speed_avoidance_min_gap_points: 2
  v2x_low_speed_avoidance_clear_distance: 8.0
  v2x_low_speed_pass_side: auto      # auto, left, right
  v2x_low_speed_pass_ramp_ratio: 1.0
  v2x_overtake_forbidden_wp_ranges: []
  v2x_state_hold_time: 0.5
```

既定では無効です。`SafetyBrake` は `v2x_safety_brake_distance` と `v^2 / (2 * abs(a_min)) + v2x_safety_brake_margin` の大きい方で判定します。`SafetyBrake` の横方向判定は corridor 全体ではなく、`v2x_vehicle_radius + v2x_prediction_margin` の衝突幅に重なった場合だけ使います。`v2x_low_speed_avoidance_enabled: true` の場合、近距離の前方車両に対して連続した十分な gap があるときは、SafetyBrake より先に `LowSpeedAvoidance` へ倒し、`v2x_low_speed_avoidance_velocity` に速度を制限して徐行回避します。連続点数は `v2x_low_speed_avoidance_min_gap_points` で指定します。V2X メッセージは速度を直接持たないため、差分速度推定は開始条件の hard gate にしません。低速回避中は停止車列を先読みするため、gap planner が見る V2X 車両を `v2x_low_speed_avoidance_lookahead_distance` 程度まで広げ、通常走行では無効にしている車両間 gap も一時的に候補へ戻します。低速回避では `v2x_low_speed_pass_side` で通過側を `auto` / `left` / `right` から選べます。`right` は reference path 座標系の負の lateral 側、`left` は正の lateral 側です。`auto` の場合は最初に選んだ側を低速回避中の side lock として使います。実制御時だけ `v2x_low_speed_pass_ramp_ratio` に従って手前の horizon 点にも side-pass target を入れ、候補判定時は実際に障害物と重なる horizon 点だけで gap を評価します。すでに `LowSpeedAvoidance` 中の場合は、曲率による追い越し禁止区間に入っても gap がある限り低速回避を継続し、さらに `v2x_low_speed_avoidance_clear_distance` 以内に V2X 車両が残る間は通常速度へ戻らず徐行を維持します。`v2x_require_gap_for_overtake: false` にすると、追い越し禁止条件に入っていない前方車は gap 幅の事前判定を必須にせず `Overtake` へ倒します。ヘアピン入口などで追い抜きを禁止したい場合は `v2x_overtake_forbidden_wp_ranges` に wp_id 範囲を追加します。

### MPC シミュレーション
```bash
ros2 run multi_purpose_mpc_ros run_mpc_simulation.bash
```

### Trajectory editor
MPC の `env/final_ver3/traj_mincurv.csv` を Lanelet2 map 上で編集します。

```bash
ros2 run multi_purpose_mpc_ros trajectory_editor
```

Pure Pursuit 用の `simple_trajectory_generator/data/raceline_awsim_30km_from_garage.csv` を開く場合:

```bash
ros2 run multi_purpose_mpc_ros pure_pursuit_trajectory_editor
```

### V2X position editor

デバッグ用に仮想車両を地図上へ置き、`/v2x/vehicle_positions` を publish します。C++ MPC の `use_v2x_gap_planner: true` と組み合わせると、配置した車両を避ける方向へ制約が変わるか確認できます。

```bash
ros2 run multi_purpose_mpc_ros v2x_position_editor
```

左クリックで車両追加/選択、ドラッグで移動、右ドラッグまたは中ドラッグで pan、ホイールで zoom、Delete で削除します。配置は JSON で保存/読込できます。

### まとめて起動（コントローラー + シミュレーション）
```bash
ros2 launch multi_purpose_mpc_ros test.launch.xml
```

### Attribution
This repository includes code derived from:

Multi-Purpose-MPC  
Author: Mats Steinweg  
Original repository: https://github.com/matssteinweg/Multi-Purpose-MPC

Used with permission from the author.
