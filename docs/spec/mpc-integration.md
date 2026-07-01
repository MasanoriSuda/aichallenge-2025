# multi_purpose_mpc_ros インテグレーション設計

> 仕様ドキュメント（現仕様の正）。最終確認: 2026-06-14。文書運用方針は [docs/README.md](../README.md) を参照。

作成日: 2026-02-10

## 概要

`multi_purpose_mpc_ros` は `aichallenge_submit` に統合済み。`reference.launch.xml` の `control_method` 引数で `mpc` / `pure_pursuit` / `tiny_lidar_net` / `pilot_net` / `joycon` を切り替えられる。デフォルトは `mpc`。

## 現在のアーキテクチャ

### ノード構成（Planning + Control）

```
reference.launch.xml (aichallenge_submit_launch)

  [Planning]
  ┌─────────────────────────────────┐
  │ simple_trajectory_generator     │
  │   CSV → Trajectory を 1Hz で Pub│
  │   出力: /planning/scenario_     │
  │         planning/trajectory     │
  └───────────┬─────────────────────┘
              │ Trajectory
              ▼
  [Control] (control_method == "pure_pursuit" の場合)
  ┌─────────────────────────────────┐
  │ simple_pure_pursuit (100Hz)     │
  │   入力:                         │
  │     /localization/kinematic_    │
  │       state (Odometry)          │
  │     /planning/scenario_         │
  │       planning/trajectory       │
  │   出力:                         │
  │     /control/command/control_cmd│
  │     (AckermannControlCommand)   │
  └─────────────────────────────────┘
```

### simple_pure_pursuit のパラメータ

- `wheel_base`: 2.14m
- `lookahead_gain`: 0.5
- `lookahead_min_distance`: 3.5m
- `speed_proportional_gain`: 1.0
- `steering_tire_angle_gain`: 1.50（sim）/ 1.639（実機）
- `use_external_target_vel`: false

### simple_trajectory_generator の役割

- CSV ファイルからウェイポイント（x, y, z, orientation, velocity）を読み込み
- `Trajectory` メッセージとして 1Hz で Publish
- ネームスペース `planning/scenario_planning` 配下で起動

## MPC コントローラの特徴

### トピック構成

**入力:**
| トピック名 | 型 | 備考 |
|-----------|-----|------|
| `/localization/kinematic_state` | Odometry | **既存と同一** |
| `planning/scenario_planning/trajectory` | Trajectory | 相対パス。`update_by_topic: true` 時のみ使用 |
| `control/control_mode_request_topic` | Bool | 制御有効/無効 |
| `/control/mpc/stop_request` | Empty | 停止要求 |

**出力:**
| トピック名 | 型 | 備考 |
|-----------|-----|------|
| `/control/command/control_cmd` | AckermannControlCommand | **既存と同一** |
| `/control/command/control_cmd_raw` | AckermannControlCommand | ゲイン適用前 |
| `/mpc/prediction` | MarkerArray | 予測軌跡（可視化） |
| `/mpc/ref_path` | MarkerArray | 参照パス（可視化） |

### 経路参照の方式

MPC コントローラは参照パスの取得方法を2種類持つ。

1. **CSV ファイルから直接読み込み**（`reference_path.update_by_topic: false`、デフォルト）
   - `config.yaml` の `reference_path.csv_path` で指定
   - 独自の occupancy grid map と最適化済み経路ファイルを使用
   - `simple_trajectory_generator` は不要

2. **Trajectory トピック経由**（`reference_path.update_by_topic: true`）
   - `simple_trajectory_generator` と同じ経路を動的に受け取り、内部で参照パスを再構成する

### V2X behavior modes

MPC controller は `/v2x/vehicle_positions` を使う V2X behavior を複数持つ。いずれも V2X は認識入力であり、最終制御出力 `/control/command/control_cmd` は維持する。

| launch param | 用途 | 有効化入口 | 主な出力 |
|---|---|---|---|
| `use_v2x_stop` | Gate1 / レース中の安全停止 fallback | 既定 `true` | 前方 target への速度 cap / stop hold |
| `use_v2x_overtake` | Gate2 専用 NPC 追い越し | `make gate2` | Gate2 用の低速 overtake lateral offset |
| `use_v2x_race_behavior` | 2 台以上同時走行の追走・yield・追い越し | `make race2` | follow / catchup_wait / yield / overtake / return / cooldown |

`use_v2x_race_behavior` と `use_v2x_overtake` が同時に true の場合、race behavior を優先し、Gate2 専用 overtake は無効化する。Gate2 は安全ゲート通過用の強めの補助を含むため、レース用挙動とは分離する。

race behavior は `v2x_race_behavior` config を使用し、前方車両に対しては距離ベースの follow speed cap、後方が大きく離れた場合は catch-up wait speed cap、後方接近車両に対しては yield speed cap、追い越し可能時は MPC path constraints への lateral offset を出す。横並び・斜め前の接触リスクは `front_conflict_lateral_window_m` / `front_conflict_gap_m` で follow 対象に含めるが、yaw-based relation が明確に後方を示す target は path projection が前方に見えても front target にしない。追い越し開始距離へ入る手前では `approach_start_gap_m` / `approach_speed_cap_kmph` で段階的に減速する。race behavior が front target を処理している間は race 側の emergency / follow / overtake を優先し、Gate1 stop fallback の stale hold で後続側を停止させない。race target がない場合や race が front target を処理していない場合は `use_v2x_stop` による安全停止 fallback を残す。

race2 の試走では前走車との距離が 3m 台で張り付きやすく、Gate2 相当の長い壁 horizon では `no_safe_side` で追い越し開始を逃しやすい。そのため race 用の初期値は `min_overtake_start_gap_m=4.0`、`wall_check_horizon_m=16.0`、`min_wall_clearance_m=0.5` とし、直近から少し先まで抜ける空間がある場合だけ `prepare_overtake` に入る。追い越し側は yield 側より速くするため、`overtake_speed_cap_kmph=12.0`、`prepare_speed_cap_kmph=9.0` を使う。車体が片側へ寄っている間に反対側の追い越しへ切り替えると、狭い corridor 内で壁へ寄りやすいため、`side_switch_center_threshold_m` 以上の横偏差が残っている間は反対側への追い越し指示を follow に戻す。

## 統合方針

### アーキテクチャ: control_method による切り替え

`control_method` launch 引数で `mpc` / `pure_pursuit` / `tiny_lidar_net` / `pilot_net` / `joycon` を選択できる。デフォルトは `mpc`。各コントローラは `<group if="...">` 内の `<include ... control/<name>.launch.xml>` で起動する（インライン `<node>` ではない）。

MPC コントローラは独自の参照パスと occupancy grid map を持ち、これが経路幅制限・速度プロファイルの制約計算に不可欠なため、MPC モードでは `simple_trajectory_generator` の軌跡入力を使用しない（`update_by_topic: false`）。

```
統合後:

  [Planning]
  ┌──────────────────────────────────┐
  │ simple_trajectory_generator      │ ← そのまま残す
  │   出力: /planning/scenario_      │    （pure_pursuit モードで使用）
  │         planning/trajectory      │
  └──────────────────────────────────┘

  [Control] (control_method == "mpc" の場合) ← デフォルト
  ┌──────────────────────────────────┐
  │ <include control/mpc.launch.xml> │
  │   mpc_controller (40Hz)          │
  │   独自CSV参照パス + occupancy map│
  │   入力:                          │
  │     /localization/kinematic_     │
  │       state (Odometry)           │
  │   出力:                          │
  │     /control/command/control_cmd │
  │     /mpc/prediction (可視化)     │
  │     /mpc/ref_path   (可視化)     │
  └──────────────────────────────────┘

  [Control] (control_method == "pure_pursuit" の場合)
  ┌──────────────────────────────────┐
  │ simple_pure_pursuit (100Hz)      │ ← 従来どおり残す
  │   入力:                          │
  │     /localization/kinematic_state│
  │     /planning/scenario_planning/ │
  │       trajectory                 │
  │   出力:                          │
  │     /control/command/control_cmd │
  └──────────────────────────────────┘

  [Control] (control_method == "tiny_lidar_net" / "pilot_net" の場合)
  ┌──────────────────────────────────┐
  │ tiny_lidar_net_controller /      │ ← LiDAR / カメラから直接制御
  │ pilot_net_controller             │
  └──────────────────────────────────┘
```

### control_method 一覧

| 値 | コントローラ | 経路ソース | 用途 |
|----|------------|-----------|------|
| `mpc` (デフォルト) | `mpc_controller` | MPC 独自 CSV | 本番走行・安全ゲート/同時走行検証 |
| `pure_pursuit` | `simple_pure_pursuit` | `simple_trajectory_generator` | デバッグ・比較検証 |
| `tiny_lidar_net` | `tiny_lidar_net_controller` | LiDAR 直接 | E2E 学習ベース走行 |
| `pilot_net` | `pilot_net_controller` | — | パイロットネット走行 |
| `joycon` | joystick teleop | — | 手動操作 |

### 各ノードの扱い

| ノード | 変更 |
|--------|------|
| `simple_pure_pursuit` | **残す**（`control_method=pure_pursuit` で使用） |
| `simple_trajectory_generator` | **残す**（`pure_pursuit` モードで使用） |
| `mpc_controller` | **新規追加**（`control_method=mpc` で使用、デフォルト） |

## 実装済み状態の確認

### パッケージ配置

`multi_purpose_mpc_ros` と `multi_purpose_mpc_ros_msgs` はすでに `aichallenge_submit/` 配下に存在する。手動でのクローンや移動は不要。

```
aichallenge/workspace/src/aichallenge_submit/
├── aichallenge_submit_launch/
├── simple_pure_pursuit/
├── simple_trajectory_generator/
├── tiny_lidar_net_controller/
├── multi_purpose_mpc_ros/          # ← 統合済み
├── multi_purpose_mpc_ros_msgs/     # ← 統合済み
└── ...（その他既存パッケージ）
```

### reference.launch.xml の現状

`aichallenge/workspace/src/aichallenge_submit/aichallenge_submit_launch/launch/reference.launch.xml`

- `control_method` arg（L20）: デフォルト `mpc`
- MPC コントローラは `<include file="$(find-pkg-share aichallenge_submit_launch)/launch/control/mpc.launch.xml">` で起動（インライン `<node>` ではなく専用 launch ファイル経由）
- `pure_pursuit` / `tiny_lidar_net` / `pilot_net` / `joycon` も同様に各 `control/<name>.launch.xml` を include する構造

### ビルド

```bash
# Docker コンテナ内でビルド
make autoware-build
```

ビルドで行われること:
1. `multi_purpose_mpc_ros_msgs` のメッセージ型生成（`AckermannControlBoostCommand.msg`, `PathConstraints.msg`, `BorderCells.msg`）
2. `multi_purpose_mpc_ros` のビルド:
   - C++ ライブラリ/ノード（`boost_commander`）のビルド
   - Python venv の作成（`/usr/bin/python3 -m venv`）
   - `requirements.txt` からの pip install（`numpy`, `pandas`, `matplotlib`, `osqp`, `scikit-image`, `PyYAML`）
   - スクリプトとデータの install

**ビルド依存関係の順序:**
```
autoware_auto_control_msgs（Autoware underlay に存在）
  → multi_purpose_mpc_ros_msgs
    → multi_purpose_mpc_ros
```

colcon が自動解決するため、特別な指定は不要。

### config.yaml の確認・調整

MPC の config ファイル: `multi_purpose_mpc_ros/config/config.yaml`

| 設定項目 | 現在の値 | 確認事項 |
|---------|---------|---------|
| `map.yaml_path` | `env/final_ver3/occupancy_grid_map.yaml` | 占有格子地図が存在するか |
| `reference_path.csv_path` | `env/final_ver3/traj_mincurv.csv` | 最適化済み経路が存在するか |
| `reference_path.update_by_topic` | `false` | CSV 直接読み込みモード（推奨） |
| `mpc.steering_tire_angle_gain_var` | `1.639` | 実機値。sim では `1.50` が必要かも |
| `mpc.v_max` | `20.0` | 速度プリセット（中速）。環境に合わせて調整 |
| `obstacles.csv_path` | `""` | 空 = トピック購読モード（障害物回避が off なので影響なし） |

**コースが変更された場合**（例: 新しい lanelet2_map.osm が配布された場合）は、「事前準備」セクションの手順に従って OGM と経路を再生成する。

### 動作確認

#### MPC モードでの起動（デフォルト）

```bash
make dev
```

起動後の確認:

```bash
# MPC ノードが起動しているか
ros2 node list | grep mpc

# 制御指令が出力されているか
ros2 topic echo /control/command/control_cmd --once

# 予測軌跡が可視化できるか
ros2 topic echo /mpc/prediction --once

# 参照パスが可視化できるか
ros2 topic echo /mpc/ref_path --once
```

#### Pure Pursuit モードでの起動確認

```bash
# reference.launch.xml を呼んでいる箇所で control_method を変更するか、
# 直接 launch コマンドで
ros2 launch aichallenge_submit_launch reference.launch.xml control_method:=pure_pursuit simulation:=true use_sim_time:=true
```

Pure Pursuit が従来通り動作することを確認。

#### 走行品質の確認

| チェック項目 | 確認方法 |
|------------|---------|
| 参照パスに追従しているか | RViz で `/mpc/ref_path` と実際の走行軌跡を比較 |
| 40Hz の制御レートで安定しているか | `ros2 topic hz /control/command/control_cmd` |
| 制御指令値が妥当か | `ros2 topic echo /control/command/control_cmd` でステア角・加速度を確認 |
| occupancy grid map が正しく読めているか | ノード起動ログでエラーがないか |

### 提出ファイルの確認

```bash
bash create_submit_file.bash
tar tf submit/aichallenge_submit.tar.gz | grep multi_purpose_mpc
```

以下のエントリが含まれていれば OK:
```
aichallenge_submit/multi_purpose_mpc_ros/
aichallenge_submit/multi_purpose_mpc_ros_msgs/
```

### 変更ファイルまとめ

| ファイル | 内容 |
|---------|------|
| `aichallenge_submit/multi_purpose_mpc_ros/` | 統合済み（in-tree） |
| `aichallenge_submit/multi_purpose_mpc_ros_msgs/` | 統合済み（in-tree） |
| `reference.launch.xml` | `control_method` 引数（デフォルト `mpc`）、各コントローラを `<include control/<name>.launch.xml>` で起動 |

### 将来の改善項目（今回はスコープ外）

- sim/実機での `steering_tire_angle_gain_var` 切り替え（config 分離 or launch param override）
- 速度プリセットの launch arg 化
- 障害物回避の有効化（`use_obstacle_avoidance=true`）
- `boost_commander` ノードの統合（`use_boost_acceleration=true` 時）
- `path_constraints_provider` ノードの統合（高度な障害物回避）

## 事前準備: MPC 用地図・経路データの生成

MPC コントローラはノード起動時にファイルを読み込むだけで、実行時に計算は行わない。**コースが変わった場合はこの手順で再生成が必要**。

現在は `env/final_ver3/` に計算済みのファイルが格納されており、同じコースであればそのまま使える。

### データ生成フロー

```
lanelet2_map.osm（コース地図）
    │
    ▼  Step 1: lanelet2_to_ogm
occupancy_grid_map.pgm + .yaml（占有格子地図）
    │
    ▼  Step 2: global_racetrajectory_optimization
traj_mincurv.csv（最適化済み経路）
```

### Step 1: Occupancy Grid Map の生成

lanelet2 形式の地図（`.osm`）から占有格子地図を生成する。

**ツール**: https://github.com/Roborovsky-Racers/lanelet2_to_ogm

**参考**: https://roborovsky-racers.github.io/RoborovskyNote/AutomotiveAIChallenge/2024/lanelet2_to_ogm.html

```bash
git clone https://github.com/Roborovsky-Racers/lanelet2_to_ogm.git
cd lanelet2_to_ogm

# lanelet2_map.osm を lanelet2/map/ に配置（デフォルトで AIC2024 マップが同梱）
make
```

**出力:**
- `occupancy_grid_map.pgm` — コースの壁・境界を表現した画像ファイル
- `occupancy_grid_map.yaml` — 解像度・原点座標の定義

### Step 2: 最適化済み経路の生成

occupancy grid map 上で最適走行ラインを計算する。

**ツール**: https://github.com/TUMFTM/global_racetrajectory_optimization

最適化基準を選択して経路を生成する。`env/preliminary/` に3種類の結果が残っている：

| ファイル | 最適化基準 |
|---------|-----------|
| `optimized_traj_mincurv.csv` | 最小曲率（カーブが緩やかなライン） |
| `optimized_traj_shortest.csv` | 最短距離 |
| `optimized_traj_mintime.csv` | 最小時間（最速ライン） |

**出力フォーマット** (`traj_mincurv.csv`):
```
s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
（距離, x座標, y座標, ヨー角, 曲率, 速度, 加速度）
```

### Step 3: env/ ディレクトリへの配置

生成したファイルを `multi_purpose_mpc_ros/env/<バージョン名>/` に配置し、`config.yaml` のパスを更新する。

```yaml
# config.yaml
map:
  yaml_path: "env/<バージョン名>/occupancy_grid_map.yaml"

reference_path:
  csv_path: "env/<バージョン名>/traj_mincurv.csv"
```

### 現在の env/ ディレクトリ構成

```
env/
├── preliminary/     # 初期版（3種類の最適化軌跡あり）
├── final/           # 決勝版 v1
├── final_ver2/      # 決勝版 v2
├── final_ver3/      # 決勝版 v3 ← 現在 config.yaml で参照中
├── final_ver4/      # 決勝版 v4
├── official/        # 公式版（軌跡なし、地図のみ）
└── others/          # 補助データ（ウェイポイント、障害物 CSV 等）
```

### ウェイポイント作成補助ツール

`env/create_waypoints.py` を使うと、occupancy grid map を GUI で表示しマウスクリックでウェイポイントを打てる。軌跡最適化ツールの入力用。

```bash
cd multi_purpose_mpc_ros/env/<バージョン名>   # 例: final_ver3（occupancy_grid_map.yaml を含む版を選ぶ）
python3 ../create_waypoints.py               # 要: matplotlib, pyyaml
```

## 注意事項

### トピック互換性

| 観点 | 互換性 | 備考 |
|------|--------|------|
| 入力: Odometry | **完全一致** | `/localization/kinematic_state` |
| 出力: 制御指令 | **完全一致** | `/control/command/control_cmd` |
| 出力: 制御指令（raw）| **完全一致** | `/control/command/control_cmd_raw` |
| Planning → Control | **不要** | MPC は独自 CSV を使う（`update_by_topic: false`） |

トピックインタフェースの互換性は高く、**リマップは不要**。

### 制御周期の差異

- simple_pure_pursuit: **100Hz**（`create_wall_timer(10ms)`）
- mpc_controller: **40Hz**（`config.yaml` の `control_rate: 40.0`）

MPC は計算負荷が高いため 40Hz は妥当。問題があれば `control_rate` を調整できる。

### Python venv

MPC パッケージは CMakeLists.txt 内で Python 仮想環境を作成する（`execute_process` で `/usr/bin/python3 -m venv` + `pip install`）。Docker ビルド内で完結するため追加設定は不要だが、ビルド時間が増加する点に注意。

### 障害物回避

MPC コントローラは障害物回避機能を内蔵しているが、**今回の統合ではデフォルトで無効**（`use_obstacle_avoidance=false`）。2026 向けに有効化する場合、障害物情報の正入力は運営回答により `/v2x/vehicle_positions` のみとする。

```yaml
# config.yaml
obstacles:
  csv_path: ""  # 2026 gate/race では空にし、V2X topic を使用する
  radius: 1.25
```

2025 由来の CSV 障害物や旧 `/aichallenge/objects`（`Float64MultiArray`）入力は、ローカル検証・旧実装互換のために残る可能性がある。ただし、2026 公式の障害物停止、追走、追い越し判断では使用しない。

MPC 側の実装方針:

- `/v2x/vehicle_positions` から前方対象の相対距離、相対速度、進行方向上の投影距離を算出する。
- Gate1 では `use_v2x_stop=true` により、追い越し不可を前提に通常走行、追走減速、停止を速度上限で切り替える。
- Gate2 では `use_v2x_overtake=true` により、同じ V2X 入力から追い越し可否と左右方向を判断し、MPC の lateral path constraints を一時的に左または右へ寄せる。
- 欠損、遅延、外れ値、急な座標ジャンプでは安全側に倒す。

`use_v2x_stop` は MPC の横方向障害物回避制約ではなく、MPC に渡す `v_max` / `v_ref` へ上から速度 cap を重ねる縦方向レイヤである。`use_obstacle_avoidance=false` でも `/v2x/vehicle_positions` を購読して動作する。

```yaml
v2x_stop:
  enabled: true
  detection_range_m: 35.0
  corridor_half_width_m: 2.0
  self_ignore_radius_m: 0.75
  target_stop_gap_m: 3.0
  stop_hold_gap_m: 3.5
  release_gap_m: 5.0
  comfortable_decel_mps2: 1.2
  stale_timeout_sec: 2.0
  max_speed_cap_kmph: 30.0
  circular_path: true
  log_throttle_sec: 1.0
```

`comfortable_decel_mps2` は `abs(mpc.a_min)` 以下に clamp する。停止判断は `V2X stop: ... target=<id> gap=<m> lat=<m> v_cap=<m/s>` の throttled log で追跡する。

**さらに高度な回避（オプション）:**

`path_constraints_provider` ノードを別途起動すると、障害物を考慮した経路の上下限制約（`PathConstraints`, `BorderCells`）を MPC に提供できる。

```yaml
# config.yaml
reference_path:
  use_path_constraints_topic: true   # PathConstraints トピックを購読
  use_border_cells_topic: true       # BorderCells トピックを購読
```

**有効化する場合の launch 変更:**
```xml
<param name="use_v2x_stop" value="true"/>
<param name="use_obstacle_avoidance" value="true"/>
```

Gate1 の停止だけなら `use_v2x_stop=true` で足りる。`use_obstacle_avoidance=true` は横方向の障害物回避制約を使う場合の追加オプションとして扱う。

Gate2 用の追い越し設定:

```yaml
v2x_overtake:
  enabled: true
  detection_range_m: 35.0
  corridor_half_width_m: 2.0
  min_overtake_start_gap_m: 6.0
  max_overtake_start_gap_m: 24.0
  abort_gap_m: 3.5
  abort_escape_lateral_threshold_m: 0.45
  return_clearance_m: 4.0
  preferred_side: "right"
  side_selection_policy: "largest_margin"
  side_margin_tie_threshold_m: 0.3
  lateral_offset_m: 1.65
  lateral_offset_rate_mps: 2.0
  constraint_half_width_m: 0.35
  constraint_transition_horizon_ratio: 0.25
  constraint_initial_progress: 0.45
  standby_lateral_offset_m: 0.35
  standby_side: "right"
  min_lateral_clearance_m: 1.55
  min_wall_clearance_m: 0.5
  wall_check_horizon_m: 24.0
  overtake_speed_cap_kmph: 5.0
  prepare_speed_cap_kmph: 3.0
  follow_speed_cap_kmph: 5.0
  overtake_steer_rate_max: 1.2
  lateral_ready_threshold_m: -0.6
  steer_override_enabled: true
  steer_override_min_abs_rad: 0.16
  steer_override_until_ey_m: -0.3
  target_lost_hold_sec: 1.2
```

`use_v2x_overtake` は既定 false。`make gate2` のときだけ `run_autoware.bash` が `use_v2x_overtake:=true` を launch へ渡す。Gate1、通常 dev、提出評価では明示的に有効化しない限り Gate2 lateral behavior は動かない。

壁距離は実行時に OSM を直接読み込むのではなく、`map.yaml_path` の occupancy grid map から `ReferencePath._compute_width()` が事前計算した waypoint ごとの `wp.ub`（左側距離）と `wp.lb`（右側距離）を使う。Gate2 planner は horizon 内の `min(wp.ub)` / `min(abs(wp.lb))` と `min_wall_clearance_m` を比較し、unsafe な側への追い越しを許可しない。両側 safe の場合は `side_selection_policy=largest_margin` により wall margin が大きい側を選び、差が `side_margin_tie_threshold_m` 以下のときだけ `preferred_side` を tie-breaker とする。追い越し開始後は選択済み side を維持し、走行中に左右を切り替えない。

追い越し判断は `V2X overtake: ... target=<id> side=<left|right> gap=<m> offset=<m> wall_l=<m> wall_r=<m>` の throttled log で追跡する。

MPC の lateral path constraints は、追い越し開始直後の offset ramp が小さい間でも反対側へ流れないように、右追い越しでは upper bound を `0.0m` 以下、左追い越しでは lower bound を `0.0m` 以上に制限する。実走ログでは右追い越しを選べていても `ey` が `+2.0m` 付近から十分に右へ戻りきれない症状が出たため、Gate2 中だけ `overtake_steer_rate_max` で MPC の操舵レート上限を一時的に上げ、`constraint_transition_horizon_ratio` と `constraint_initial_progress` で horizon 前半から追い越し側の corridor へ遷移させる。

Gate2 では V2X target を検出してから横移動を始めると間に合わない場合があるため、`standby_lateral_offset_m` で target 未検出時から右側へ軽く寄せて待機する。追い越し中に V2X target が一瞬 stale になると state が `follow` に戻り、右 offset が解除されて停止側へ落ちる可能性がある。そのため `target_lost_hold_sec` の間は選択済み side と lateral offset を保持し、低速 cap で追い越し継続を試みる。

実際の `ey` がまだ追い越し側へ入っていない間は、`prepare_speed_cap_kmph` により overtake speed よりさらに低速へ落とす。ログには `ey`、`mpc_steer_rate`、`transition_cols`、`initial_progress`、`corridor=[lower,upper]` も出し、planner の side 判定と MPC が実際に追う横制約を照合できるようにする。

横移動の進捗判定は `abs(ey)` ではなく side-aware に扱う。右追い越しでは `ey < 0` 側、左追い越しでは `ey > 0` 側へ入った場合だけ進捗とみなす。初期状態で `ey` が反対側に大きい場合に、追い越し済みと誤判定して `abort_gap` や復帰判定が早く発火するのを避けるため。

`abort_gap` で停止へ落とすかどうかの横逃げ判定は、`lateral_offset_m` の比率ではなく `abort_escape_lateral_threshold_m` を使う。2 台目の横余裕を確保するために `lateral_offset_m` を大きくすると、比率判定では 1 台目の `gap=3m` 付近で必要横移動量も増えてしまい、追い越し継続中なのに `abort_gap` へ落ちるため。

`standby_lateral_offset_m` は target 未検出時だけの待機 offset として扱う。planner が `return_to_line`、`follow`、`abort` などで `target_lateral_offset_m=0.0` を返している間は、standby offset より planner の 0 指示を優先し、次 target の手前で右側に残り続けないようにする。

`return_to_line` の完了判定は、MPC に入れている offset 指令値ではなく、実際の path lateral error `ey` を使う。offset 指令だけで clear にすると、車体がまだ追い越し側へ残っている状態で次の V2X target を新規障害物として拾い、`abort_gap` と Gate1 stop に落ちるため。

追い越し中に現在 target を抜き切る前後で別 target が前方 corridor 内に入った場合は、同じ side の安全性が維持できる限り `return_to_line` へ入らず、追い越し対象をその target へ切り替える。Gate2 のように複数台が並ぶケースでは、1 台ごとに中央へ戻るのではなく、障害物列を抜け切ってから戻る挙動を正とする。

Gate2 の低速検証では、MPC の corridor 制約だけだと初期横移動が間に合わない場合がある。その場合は `steer_override_enabled=true` により、`prepare_overtake` / `overtaking` 中かつ `ey` が `steer_override_until_ey_m` より左側に残っている間だけ、選択 side へ最低操舵角 `steer_override_min_abs_rad` を入れる。浅すぎると 1 台目へ接触し、2 台目では `lat=0.15m` の target に対して `ey=-1.5m` 付近でも横余裕が不足したため、`lateral_offset_m` は `1.65m`、`min_lateral_clearance_m` は `1.55m` としている。override は `ey=-0.3m` までの補助に留める。これは Gate2 の緊急補助であり、race behavior へ広げる場合は速度、壁距離、他車との横距離を別途検証する。

Gate2 では `bicycle_model.width` を実車幅 `1.45m` として扱う。`2.30m` のような safety 込み幅を入れると、`BicycleModel._compute_safety_margin()` と Gate2 の `min_wall_clearance_m` / `wall_safety_margin_m` が重なり、初期位置が制約外になりやすい。安全余裕は width ではなく、MPC safety margin と Gate2 専用 clearance param で持つ。

### 提出ファイルへの影響

`create_submit_file.bash` で `aichallenge_submit` 以下を tar.gz にまとめるため、`multi_purpose_mpc_ros` と `multi_purpose_mpc_ros_msgs` が `aichallenge_submit/` 配下にある必要がある。

## まとめ

| 項目 | 内容 |
|------|------|
| 統合方式 | `control_method` launch 引数で切り替え |
| デフォルト | `mpc`（MPC コントローラ） |
| 切り替え | `pure_pursuit` / `tiny_lidar_net` / `pilot_net` / `joycon` |
| トピック互換 | 入出力ともに一致、リマップ不要 |
| 経路参照 | MPC: 独自 CSV / Pure Pursuit: `simple_trajectory_generator` |
| MPC 起動方式 | `<include control/mpc.launch.xml>` 経由（インライン node ではない） |
| パッケージ配置 | `aichallenge_submit/` 配下に統合済み（追加作業不要） |
| ビルド注意 | Python venv 作成（pip install）によるビルド時間増加 |
