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

#### Wall-edge trajectory tuning

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
  v2x_vehicle_radius: 1.25
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
```

`V2XVehiclePositionArray` は位置と covariance のみを持つため、C++ 側では vehicle_id ごとの直近2点から速度を推定します。初回、異常ジャンプ、過大速度時は静止障害物として扱います。gap がない場合は `no_gap_target_velocity` を速度上限として使います。

free gap が壁と他車に挟まれている場合、`v2x_wall_clearance_margin` で壁側の制約を内側へ削り、`v2x_wall_avoidance_bias` で target を gap 中央から車側へ寄せられます。壁ペナルティを避けたい場合は `v2x_wall_clearance_margin: 0.4`, `v2x_vehicle_side_target_margin: 0.2`, `v2x_wall_avoidance_bias: 0.8` から確認します。

左右が両方とも V2X 車両の free gap は、`v2x_vehicle_vehicle_gap_enabled: false` で候補から外せます。3台同時走行のスタート直後など、前方2台の間をすり抜けようとして操舵が不安定になる場合は、車-車 gap を禁止し、壁-車 gap または no-gap 低速追走へ倒します。

さらに、前方近距離に2台以上いる状況そのものを追い抜き禁止にしたい場合は、`v2x_multi_front_gap_enabled: false` と `v2x_multi_front_gap_distance` を設定します。この条件に入ると gap planner は横目標を作らず、`no_gap_target_velocity` による低速追走へ倒します。

#### V2X behavior FSM

`use_v2x_behavior_fsm: true` にすると、`Cruise` / `Follow` / `Overtake` / `SafetyBrake` の最小状態で `V2XGapPlanner` の使用可否を制御します。`Follow` と `SafetyBrake` では gap planner を使わず、速度上限を下げます。`Overtake` のときだけ gap planner が `lb/ub` と `xr` に反映されます。

```yaml
mpc:
  use_v2x_behavior_fsm: false
  v2x_follow_distance: 8.0
  v2x_safety_brake_distance: 3.0
  v2x_follow_velocity: 5.0
  v2x_safety_brake_velocity: 0.0
  v2x_overtake_min_gap_width: 2.0
  v2x_overtake_max_curvature: 0.05
  v2x_overtake_forbidden_wp_ranges: []
  v2x_state_hold_time: 0.5
```

既定では無効です。ヘアピン入口などで追い抜きを禁止したい場合は `v2x_overtake_forbidden_wp_ranges` に wp_id 範囲を追加します。

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
