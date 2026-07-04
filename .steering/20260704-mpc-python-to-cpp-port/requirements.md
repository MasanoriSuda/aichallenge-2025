# MPC Python to C++ Faithful Port Requirements

作成日: 2026-07-04
状態: Draft

## 目的

現行の `multi_purpose_mpc_ros` の Python 実行系を C++ 実行系へ置き換える。目的は高速化や設計刷新ではなく、Python コードの有効挙動を正本として忠実に移植すること。

この作業では「Codex らしい改善」や独自判断のリファクタリングを入れない。変数名、処理順、境界条件、既存の不自然な挙動も、評価や安全に明確な問題が確認されるまでは Python 実装に合わせる。

## 背景

- `control_method=mpc` は現行の既定制御方式。
- `aichallenge_submit_launch/launch/control/mpc.launch.xml` は `multi_purpose_mpc_ros` の `run_mpc_controller.bash` を起動する。
- `run_mpc_controller.bash` は package install prefix 配下の Python venv を有効化し、Python の `mpc_controller` script を実行する。
- `multi_purpose_mpc_ros` には既に C++ の `boost_commander` があるが、MPC 本体は Python 実装。
- 2026 公式仕様が未確定の項目は、2025 由来の現行仕様として扱う。

## 対象範囲

対象:

- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`
- Python MPC 実行時に使う主要モジュール:
  - `multi_purpose_mpc_ros/mpc_controller.py`
  - `multi_purpose_mpc_ros/core/MPC.py`
  - `multi_purpose_mpc_ros/core/reference_path.py`
  - `multi_purpose_mpc_ros/core/spatial_bicycle_models.py`
  - `multi_purpose_mpc_ros/core/map.py`
  - `multi_purpose_mpc_ros/core/utils.py`
  - `common.py`
  - `obstacle_manager.py`
  - `v2x_vehicle_tracker.py`
  - `simulation_logger.py`
  - `exexution_stats.py`
  - `tools/reference_velocity_configulator.py`
- MPC launch と package build/install 設定。

対象外:

- `simple_pure_pursuit` の制御ロジック変更。
- `simple_trajectory_generator` の経路生成変更。
- `tiny_lidar_net` / `pilot_net` の挙動変更。
- `aichallenge_system/` の評価 FSM、result JSON、AWSIM 管理の変更。
- Domain、V2X topic 契約、提出 tar.gz レイアウトの変更。
- MPC パラメータチューニング、軌道 CSV の最適化、速度プロファイルの改善。

## 守る契約

次の契約は C++ 化後も変えない。

- `control_method` の有効値は `mpc`、`pure_pursuit`、`tiny_lidar_net`、`pilot_net`、`joycon`。
- 既定の `control_method` は `mpc`。
- `aichallenge_submit_launch` の `aichallenge_submit.launch.xml` と `reference.launch.xml` の起動階層を維持する。
- MPC の ROS node name は `mpc_controller`。
- launch 引数・実行引数の意味を維持する:
  - `--config_path`
  - `--ref_vel_path`
  - `use_sim_time`
  - `use_boost_acceleration`
  - `use_obstacle_avoidance`
  - `use_stats`
- 入力 topic と型を維持する:
  - `/localization/kinematic_state`: `nav_msgs/Odometry`
  - `planning/scenario_planning/trajectory`: `autoware_auto_planning_msgs/Trajectory`
  - `control/control_mode_request_topic`: `std_msgs/Bool`
  - `/control/mpc/stop_request`: `std_msgs/Empty`
  - `/awsim/status`: `std_msgs/Float32MultiArray`
  - `/aichallenge/pitstop/condition`: `std_msgs/Int32`
  - `/path_constraints_provider/path_constraints`: `multi_purpose_mpc_ros_msgs/PathConstraints`
  - `/path_constraints_provider/border_cells`: `multi_purpose_mpc_ros_msgs/BorderCells`
  - `/v2x/vehicle_positions`: `v2x_msgs/V2XVehiclePositionArray`
- 出力 topic と型を維持する:
  - `/control/command/control_cmd`: `autoware_auto_control_msgs/AckermannControlCommand`
  - `/control/command/control_cmd_raw`: `autoware_auto_control_msgs/AckermannControlCommand`
  - `/boost_commander/command`: `multi_purpose_mpc_ros_msgs/AckermannControlBoostCommand`
  - `/mpc/prediction`: `visualization_msgs/MarkerArray`
  - `/mpc/ref_path`: `visualization_msgs/MarkerArray`
  - 既存の可視化用 dummy topic 2 本。
- `planning/scenario_planning/trajectory` の subscription は Python 実装と同じく BEST_EFFORT / KEEP_LAST / depth 1 にする。
- `/control/command/control_cmd` を最終制御出力として維持する。
- `/localization/kinematic_state` と `/planning/scenario_planning/trajectory` の連結を壊さない。

## 忠実移植の要求

Python コードを仕様として扱い、次を維持する。

- YAML key、既定値、単位、パラメータ更新時の副作用。
- `yaw_from_quaternion` の計算式と特異点分岐。
- map の `w2m` / `m2w` 変換、occupied threshold、small holes 除去の実効挙動。
- reference path の補間、平滑化、circular path の先頭追加、曲率計算。
- path width と border cell の探索方法。
- speed profile の OSQP 問題、制約、cost、終端 waypoint の扱い。
- spatial bicycle model の `t2s`、`s2t`、`drive`、`linearize`。
- MPC の OSQP 問題生成、制約行列、cost、steering rate constraint、infeasible 時の fallback。
- `use_max_kappa_pred` による速度上限制約の切り替え。
- `steering_tire_angle_gain_var` の適用順序。
- `accel_low_pass_gain` と `steer_low_pass_gain` の適用順序。
- `KP = 100.0` による速度追従 acceleration 計算。
- stop request 時の減速処理。
- V2X 障害物の予測、corridor filtering、obstacle map 更新。
- lap / collision / marker publish / stats / logger の実行時影響。

既存 Python の typo や不自然な名前も、外部挙動に影響しない限り勝手に直さない。修正が必要な場合は、別タスクとして理由と差分を明記する。

## 禁止事項

- 移植と同時に MPC アルゴリズムを改善しない。
- solver を OSQP 以外に置き換えない。
- パラメータ値、速度制限、steering gain、low-pass gain を調整しない。
- topic 名、message 型、QoS、node name を独自判断で変えない。
- launch の `control_method` 契約を変えない。
- Python 実装を削除してから C++ 実装の一致確認を始めない。
- C++ らしさを理由に処理順、境界条件、丸め、fallback を変えない。

## 受け入れ条件

最小受け入れ条件:

- `make autoware-build` が成功する。
- `control_method=mpc` で C++ MPC node が起動する。
- `/control/command/control_cmd` が publish される。
- 既存の `mpc.launch.xml` 利用者から見た起動引数、topic、param の意味が変わらない。
- Python 実装と C++ 実装の golden test を用意し、主要な中間値と最終制御出力を比較できる。
- Python 実装との差分がある場合は、許容理由と影響範囲を記録する。

推奨受け入れ条件:

- map load、reference path、speed profile、bicycle model、MPC first control の単体比較が通る。
- 同一 odometry sequence に対して C++ と Python の `/control/command/control_cmd` が許容誤差内に収まる。
- `make dev` または `make gate*` で起動し、MPC の制御出力、prediction marker、ref path marker が確認できる。
- Python venv / pip install が MPC 実行時の必須経路から外れる。ただし比較用 Python 実装は parity 確認完了まで残す。
- 実装変更に応じて `docs/spec/mpc-integration.md` と `docs/interface/participant-interface.md` を更新する。

## 未確定事項

- C++ 側 OSQP 連携に使う package / library の可用性。
- Python の `scipy.sparse` と同じ sparse matrix 構築を C++ でどの粒度まで一致させるか。
- Python の `skimage.draw.line_aa` と `remove_small_holes` の C++ 等価実装方針。
- PGM/YAML/CSV 読み込みに追加依存を入れるか、狭い独自実装にするか。
- `ReferenceVelocityConfigulator` を初期移植に含めるか、`--ref_vel_path` 指定時のみ有効な段階移植にするか。
