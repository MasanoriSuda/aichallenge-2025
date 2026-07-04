# MPC Python to C++ Faithful Port Design

作成日: 2026-07-04
状態: Draft

## 方針

Python 実装を唯一の正本として、C++ 実装は Python の処理を写経する。設計改善、責務分割の再設計、パラメータ調整はしない。

移植は段階的に行う。Python 実装と C++ 実装を一時的に共存させ、fixture と実行ログで一致確認してから launch の実行先を C++ に切り替える。

## 現行実行経路

```text
aichallenge_submit_launch/launch/control/mpc.launch.xml
  -> multi_purpose_mpc_ros exec=run_mpc_controller.bash
    -> install prefix の .venv を activate
      -> python3 lib/multi_purpose_mpc_ros/mpc_controller
        -> MPCController(args.config_path, args.ref_vel_path)
```

C++ 化後も、外側から見える `control_method=mpc`、node name、topic、param は維持する。

## 実装配置案

既存 package `multi_purpose_mpc_ros` の中に C++ 実装を追加する。新しい package は作らない。

```text
multi_purpose_mpc_ros/
  include/multi_purpose_mpc_ros/
    mpc_controller.hpp
    mpc_config.hpp
    core/
      map.hpp
      reference_path.hpp
      spatial_bicycle_models.hpp
      mpc.hpp
      utils.hpp
    obstacle_manager.hpp
    v2x_vehicle_tracker.hpp
    simulation_logger.hpp
    execution_stats.hpp
    reference_velocity_configulator.hpp
  src/
    mpc_controller.cpp
    mpc_controller_node.cpp
    core/
      map.cpp
      reference_path.cpp
      spatial_bicycle_models.cpp
      mpc.cpp
      utils.cpp
    obstacle_manager.cpp
    v2x_vehicle_tracker.cpp
    simulation_logger.cpp
    execution_stats.cpp
    reference_velocity_configulator.cpp
```

`execution_stats` は Python 側のファイル名が `exexution_stats.py` だが、C++ 側のファイル名を直す場合は外部挙動に影響しない範囲に限定する。

## Python to C++ 対応表

| Python | C++ | 方針 |
|---|---|---|
| `mpc_controller.py` | `MPCController` | ROS I/O、loop、param callback、marker publish を移植 |
| `core/MPC.py` | `core::MPC` | OSQP 問題生成と fallback を移植 |
| `core/reference_path.py` | `core::ReferencePath` / `Waypoint` / `BorderCells` | 経路補間、曲率、width、speed profile を移植 |
| `core/spatial_bicycle_models.py` | `core::BicycleModel` / state structs | `t2s`、`s2t`、`drive`、`linearize` を移植 |
| `core/map.py` | `core::Map` / `Obstacle` | occupancy grid 読み込み、座標変換、障害物追加を移植 |
| `core/utils.py` | `core::utils` | CSV load、単位変換を移植 |
| `common.py` | config / arg helper | YAML 読み込みと引数 parse を移植 |
| `v2x_vehicle_tracker.py` | `V2XVehicleTracker` | obstacle avoidance 有効時の V2X 予測を移植 |
| `obstacle_manager.py` | `ObstacleManager` | 既存の実行時影響がある範囲を移植 |
| `simulation_logger.py` | `SimulationLogger` | stop 判定とログ副作用を移植 |
| `tools/reference_velocity_configulator.py` | `ReferenceVelocityConfigulator` | `--ref_vel_path` 利用時の速度上書きを移植 |

## 数値表現

- Python の `float` / numpy default に合わせ、C++ は原則 `double` を使う。
- 配列と行列は Python の shape と index order を優先する。
- 変数名は Python に寄せる。
- 角度正規化、丸め、clip、mod は Python の実効挙動と比較して決める。
- `np.inf` 相当の制約は OSQP に渡す値として Python と一致することを確認する。

## OSQP 方針

- solver は OSQP を維持する。
- Python の `osqp.OSQP().setup(..., verbose=False)` と同等の設定を C++ で使う。
- warm start や solver tuning は、Python 実装が明示していない限り追加しない。
- `P`、`q`、`A`、`l`、`u` の構築結果を fixture として比較する。
- infeasible 時の fallback は Python の有効挙動を優先する。Python の `except TypeError or ValueError` のような不自然な構文も、実際に捕捉される例外範囲を確認してから C++ に写す。

## Map / Image 方針

Python 実装は YAML と PGM を読み込み、occupied threshold と small holes 除去を行う。

C++ では次を満たす。

- `occupied_thresh`、`resolution`、`origin`、`image` の読み方を維持する。
- `w2m` の `+ 0.5` 丸めと clipping を維持する。
- `m2w` の `int(dx + 0.5)` / `int(dy + 0.5)` 相当を維持する。
- `remove_small_holes(area_threshold=5, connectivity=8)` の等価性を fixture で確認する。
- `line_aa` 相当の rasterization は path width と obstacle 判定への影響が大きいため、Python と cell sequence を比較する。

## ROS Node 方針

C++ node は `rclcpp::Node` として実装する。

維持するもの:

- node name: `mpc_controller`
- publisher / subscriber topic
- message type
- QoS
- parameter name
- parameter update callback の副作用
- `use_sim_time` 有効時の clock 待ち
- odometry / trajectory / path constraints 待ち
- 40 Hz control loop
- stop 時の 5 秒停止試行と zero command publish

Python の `MultiThreadedExecutor(num_threads=2)` と同じく、subscription callback と control loop が詰まらない構成にする。

## Launch / Build 方針

段階 1:

- C++ executable を追加する。
- Python executable は残す。
- launch の既定は Python のままにし、C++ は開発用に明示起動できるようにする。

段階 2:

- golden test とローカル起動で parity を確認する。
- `mpc.launch.xml` の実行先を C++ に切り替える。
- `control_method=mpc` の外部契約は変えない。

段階 3:

- Python venv と pip install を runtime 必須経路から外す。
- 比較用 Python 実装を削除する場合は、削除理由と parity 結果を残す。

## テスト設計

Python から fixture を生成し、C++ と比較する。

比較対象:

- `yaw_from_quaternion`
- `Map.w2m` / `Map.m2w`
- map binarization 後の occupied/free cell count
- reference path の waypoint 数、代表 waypoint の `x`、`y`、`psi`、`kappa`
- path width の `ub`、`lb`、border cell
- speed profile の `v_ref`
- bicycle model の `t2s`、`s2t`、`linearize`
- MPC の `P`、`q`、`A`、`l`、`u`
- first control の `v`、`delta`、`max_delta`
- control loop の `AckermannControlCommand`

許容誤差は fixture ごとに明示する。最初から広い誤差を置かず、Python と C++ の差が出た箇所を特定してから判断する。

## 移行時の安全策

- C++ 実装へ切り替える PR では、MPC 以外の制御方式を変更しない。
- `pure_pursuit` を比較・退避用に維持する。
- `use_obstacle_avoidance=false` の基本経路を先に通す。
- obstacle avoidance / V2X は基本経路の parity 後に有効化する。
- `make autoware-build` を最小検証とし、制御挙動変更時は `make dev` または `make gate*` で確認する。

## ドキュメント更新

実装切り替え時に次を更新する。

- `docs/spec/mpc-integration.md`
  - MPC が Python 実行系ではなく C++ 実行系であること。
  - Python venv / pip install が不要になった場合のビルド説明。
- `docs/interface/participant-interface.md`
  - `control_method=mpc` の起動ノード説明。
  - topic / param 契約に変更がないこと。
- `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/README.md`
  - C++ MPC の起動方法、比較方法、既知差分。
