# multi_purpose_mpc_ros インテグレーション設計

> 仕様ドキュメント（現仕様の正）。最終確認: 2026-07-11。文書運用方針は [docs/README.md](../README.md) を参照。

作成日: 2026-02-10

## 概要

`multi_purpose_mpc_ros` は `aichallenge_submit` に統合済み。`reference.launch.xml` の `control_method` 引数で `mpc` / `pure_pursuit` / `tiny_lidar_net` / `pilot_net` / `joycon` を切り替えられる。デフォルトは `mpc`。MPC の通常実行ノードは Python 版から C++ 版の `mpc_controller_cpp` に移行済みで、Python 実装と補助スクリプトは比較・生成ツール用途として残している。

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
| `/awsim/status` | Float32MultiArray | SIM状態。Boost残数と発動中状態を含む |
| `/awsim/state` | String | 車両FSM。Startとセッション境界を検知 |

**出力:**
| トピック名 | 型 | 備考 |
|-----------|-----|------|
| `/control/command/control_cmd` | AckermannControlCommand | **既存と同一** |
| `/control/command/control_cmd_raw` | AckermannControlCommand | ゲイン適用前 |
| `/mpc/prediction` | MarkerArray | 予測軌跡（可視化） |
| `/mpc/ref_path` | MarkerArray | 参照パス（可視化） |
| `/awsim/cmd` | Float32MultiArray | 2026 SIM Boost指令。`awsim_boost.enabled`時のみ |

### 経路参照の方式

MPC コントローラは参照パスの取得方法を2種類持つ。

1. **CSV ファイルから直接読み込み**（`reference_path.update_by_topic: false`、デフォルト）
   - `config.yaml` の `reference_path.csv_path` で指定
   - `reference_path.domain_csv_path` を設定すると、`ROS_DOMAIN_ID` ごとに CSV を上書きできる
   - 独自の occupancy grid map と最適化済み経路ファイルを使用
   - `simple_trajectory_generator` は不要

2. **Trajectory トピック経由**（`reference_path.update_by_topic: true`）
   - `simple_trajectory_generator` と同じ経路を動的に受け取り、内部で参照パスを再構成する

### C++ MPC correctness hardening（2026-07-10）

C++ 本番ノードの経路入力・solver処理には、次の安全化を適用している。

- CSV直接読込では `s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2` の7列を strict loader で読み、欠落列、不正数値、NaN / Inf、非単調 `s_m` を起動時エラーにする。
- `circular: true` で先頭・終点が同一点の場合は、全列を同じレコード単位で末尾から除去してから内部 ReferencePath を作る。
- `ceil(distance / resolution)` の分割 helper と単体テストを追加した。ただし、固定 `N` の物理 horizon を変えないため、本番 ReferencePath は距離ベース horizon へ移行するまで legacy `floor` 分割を維持する。
- 角度正規化は `atan2(sin(angle), cos(angle))` 相当を使い、正負の pi 境界を扱う。
- MPC問題生成では現在位置の `base_wp_id` を変更せず、offset適用後の `planning_wp_id` をローカルに保持する。副作用を1周期内で複数回進めないため、旧 safety-margin retry は除去した。
- OSQPは `SOLVED` / `SOLVED_INACCURATE`、解の有限性、入力を含む全制約違反を確認し、直線の steering 0 を異常扱いしない。
- callback間の未保護な同時更新を避ける初期安全策として、C++ node は `SingleThreadedExecutor` で実行する。
- odometry の受信時刻と、非ゼロsource stampが最後に変化した時刻をsteady clockで監視する。既定 `odom_timeout_sec: 0.5` を超えた場合、boost を無効にして速度0・負加速度・rate limit付き操舵復帰を直接publishする。pose、速度、solver出力、gain適用後commandの NaN / Inf も同じ fail-safe 経路で拒否する。
- `min_linearization_speed_mps: 0.5` 未満では `1/v`、`1/v^2` を含む時間状態の線形化を停止する。両値はローカル設定であり、2026公式値ではない。
- Python `path_constraints_provider` も circular CSV の重複終点を同じ許容差で除去し、C++側は受信したrows/colsが内部ReferencePathと一致しない制約を拒否する。
- solver fallback と `/control/mpc/stop_request` はlegacy boost arbitrationより優先し、boostを必ず無効化する。low-pass gainは `[0,1]` に限定し、filter後にも加速度、操舵角、操舵変化量を制限する。

### AWSIM 2026 Start Dash Boost（2026-07-11）

2026公式Boostは通常の`AckermannControlCommand`加速度とは独立したAWSIM item commandとして扱う。

- `awsim_boost.enabled: true`、`mode: start_once`でシミュレーション時だけ有効化する。
- `/awsim/state=Start`、自動制御有効、正常odometry、solver非fallback、freshな7要素`/awsim/status`、`boostRemaining >= 1`、`isBoosting < 0.5`をすべて満たした最初の正常control cycleで発動する。
- `/awsim/cmd`へ`[1.0]`、続けて`[0.0]`をReliable QoSで各1回publishする。
- high/lowの1ペア送信時点でそのセッションを使用済みにし、確認timeoutやstatus欠落でも再送しない。
- `isBoosting`または残数減少で確認するが、確認結果は再送判断に使わない。
- `Start`重複、`Ready`、control disable、fail-safe回復では再armしない。`Finish`後の新しい`Spawned`だけ次セッションへrearmする。
- `use_sim_time=false`またはlegacy `use_boost_acceleration=true`では公式Boost I/Oを無効化する。
- 使用可能回数は環境設定で変わるため、5や2をコードへ固定しない。

```yaml
awsim_boost:
  enabled: true
  domain_enabled:
    1: true
    2: true
    3: true
  mode: start_once
  status_timeout_sec: 0.5
  confirmation_timeout_sec: 2.0
```

`use_boost_acceleration`、`AckermannControlBoostCommand`、`/boost_commander/command`、高頻度`control_cmd`再送は2025由来のlegacy経路であり、2026公式Boostには使用しない。

trajectory の静的検証には次を使う。

```bash
ros2 run multi_purpose_mpc_ros reference_path_validator \
  $(ros2 pkg prefix --share multi_purpose_mpc_ros)/env/final_ver3/traj_mincurv.csv \
  --circular
```

### Offline Trajectory Editor safety（2026-07-11）

`trajectory_editor` には、GUI非依存のPython validator、geometry normalization、offline speed-profile生成、Before/Candidate比較、安全保存を追加している。MPCはcanonical 7列、Pure Pursuitは既存8列として別々に検証し、Validate操作はworking data、Undo、revision、入力fileを変更しない。

- 周回状態は重複終端とは別に保持し、`--circular` / `--open` またはGUIで明示する。組み込みMPC presetの周回既定はローカル設定であり、2026公式仕様ではない。
- MPCのXY変更は `s_m/psi_rad/kappa_radpm` と `vx_mps/ax_mps2` の両方をstaleとする。Pure Pursuitは既存8列とquaternionの明示再計算を維持する。保存時の暗黙再計算は行わない。
- `Normalize Geometry` は重複終端・退化点の選択的除去、open/circularのcanonical arc length、等間隔線形再サンプリング、heading/curvature再生成をdetached candidate上で行う。`vx/ax`はpreserve、周期/線形interpolate、後続speed再計算へdeferのいずれかを明示する。
- `Recompute Speed` は `min(v_max, sqrt(ay_max/max(abs(kappa), epsilon)))` の上限からforward/backward relaxationを行い、周回seamを含む制約、minimum speed競合、非収束、出力validationを確認する。open終端の`ax_mps2`は0とする。
- candidateはsource revisionと内容signatureに結び、workerで生成する。XY、spacing、psi、kappa、velocity、acceleration、lateral accelerationと補正統計をpreviewし、validation error時はApplyを無効にする。Applyは1回のUndo snapshotとしてworkingへ反映する。
- main canvasは読込時`Original`、編集中`Working`、detached `Candidate`を独立表示する。`Original`はSaveで置換せず、別trajectoryのOpen時だけ更新する。same-arc比較でpoint数差、path長差、最大・平均変位、1cm超の変更`s_m`範囲を表示し、該当Working segmentを強調する。zoom後の移動は水平・垂直scrollbarを正規操作とし、既存の右・中drag panと`Center Selection`も同じviewport stateへ同期する。
- `Validate Clearance`は選択したoccupancy-grid YAML / P2・P5 PGMを読み、rear-axle基準の向き付き車体矩形、前後左右margin、point間sweepをofflineで検証する。final_ver3のbinary PGM、origin yaw=0、negate=0では画像最大値正規化、Y反転、pixel threshold、5 cell未満のoccupied component除去を現C++ runtimeへ合わせる。一般map loader完全互換ではなく、Editorはorigin yawを扱い、unknownを既定occupied、map外をclampせずerrorとする安全側仕様を持つ。
- raw minimum clearanceは離散poseの車体矩形とunsafe cell矩形の距離である。別表示のconservative minimumはpoint側のgrid量子化下限と、segment sweepの距離場下限・回転膨張を含む値の最小である。final_ver3の0.1m gridに対する約0.071mはpoint側量子化項に限り、report全体の固定差引量ではない。Lanelet2 railは表示用であり、wall判定には使用しない。
- `Adjust Clearance`は最大shift内のpath法線方向offsetを決定的に探索し、補正後のgeometryとpoint/swept clearanceがsafeな場合だけdetached candidateを生成する。Apply直前にdocument revision、candidate content、map signature、vehicle/margin設定、clearanceを再検査する。Apply後はgeometry valid、speed metadata staleとし、`Recompute Speed`完了まで保存を止める。sweep step 0.05m、最大shift 0.50m、offset step 0.05m、最大絶対曲率0.70rad/m、margin 0mはローカル暫定値であり、`INFEASIBLE`はbounded search内で候補を発見できなかったことだけを表す。実コース規模で数十秒かかる場合があるためworkerで実行し、progress dialogから協調Cancelできる。
- Clearance stateはnot-run、running、safe、unsafe、failed、staleを区別する。失敗した再検証では過去のSAFEを無効化し、設定後のfailed/running/staleは保存不可とする。SAFE保存時もYAML/PGMを再読込してmap signatureを再確認する。
- 通常動線は処理に応じた `*_normalized.csv`、`*_speed_profiled.csv`、`*_edited.csv` への `Save As` とする。上書きはpath確認を要求し、同一directoryのtemporary fileを再検証後にatomic replaceする。symlink targetは置換しない。
- `resolution=0.25 m`、`a_max=1.0 m/s²`、`horizon_distance=16 m` は `AI Challenge 2026 Candidate - Safe` のローカル候補値であり、公式確定値ではない。resolutionと加速度条件は編集可能、horizonはruntime統合hintとしてread-only表示する。

Editorが保存する `vx_mps/ax_mps2` はoffline CSV metadataである。現在のC++ MPCは制御周期内のruntime速度上限を優先するため、この値を編集しただけでは走行速度プロファイルへ直接反映されない。

Clearanceの車体presetは現行`vehicle_info.param.yaml`から導出したrear-axle基準の暫定値であり、trajectory pose基準とAWSIM colliderの一致は未確認である。margin既定値は0 mで、2026公式安全余白を意味しない。occupancy gridも静的近似なので、EditorのSAFEはAWSIM／実車の非接触やSafety Gate通過を保証しない。採用candidateは別名保存し、C++ validator後にシミュレータで最初のヘアピンと1周を確認する。

Editorの0.25m CSV生成はoffline機能として実装済みだが、runtime側の `resolution: 0.6`、legacy補間、`smoothing_distance`、固定 `N` は変更していない。生成CSVを本番pathへ採用する場合は、C++ validator、固定Nの実距離horizon、計算時間、壁余白、走行回帰を別途確認する。特にCSV密度だけを上げてもruntime再サンプリングや速度上書きの責務は自動では変わらない。

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
| `mpc_controller` | **C++ 実装を使用**（`mpc_controller_cpp` executable、`control_method=mpc` で使用、デフォルト） |

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
   - C++ ライブラリ/ノード（`awsim_boost_start_dash`, `boost_commander`, `mpc_controller_cpp`）のビルド
   - OSQP C API（`osqp_vendor`）、Eigen、OpenCV、yaml-cpp を使った MPC 実行系のビルド
   - Python 補助スクリプト用 venv の作成（`/usr/bin/python3 -m venv`）
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
| `reference_path.domain_csv_path` | Domain 1..3 別 CSV | `ROS_DOMAIN_ID` ごとの trajectory 上書き。未設定 Domain は `csv_path` を使う |
| `reference_path.update_by_topic` | `false` | CSV 直接読み込みモード（推奨） |
| `awsim_boost.enabled` | `true` | SIMで2026公式Boostを有効化。実車では無効 |
| `awsim_boost.domain_enabled` | Domain 1..3=`true` | `ROS_DOMAIN_ID`ごとの有効/無効上書き。未設定Domainは`enabled`を使う |
| `awsim_boost.mode` | `start_once` | Start後の正常制御時に1回だけ発動 |
| `mpc.steering_tire_angle_gain_var` | `1.639` | 実機値。sim では `1.50` が必要かも |
| `mpc.wp_id_low_offset` | 未設定 | 低速時の参照 waypoint offset。未設定時は `wp_id_offset` |
| `mpc.wp_id_low_speed` | `0.0` | 低速判定閾値。値は km/h、`0.0` なら無効 |
| `mpc.wp_id_offset` | `2` | 通常時の参照 waypoint offset |
| `mpc.center_bias` | `0.0` | `0.0` = CSV trajectory 追従、`1.0` = 左右制約中央寄せ |
| `mpc.safety_margin_scale` | `1.0` | `0.0` = 追加 margin なし、`1.0` = 現行 margin |
| `mpc.use_v2x_gap_planner` | `false` | `/v2x/vehicle_positions` から rule-based gap を作る暫定拡張。既定無効 |
| `mpc.v_max` | `20.0` | 速度プリセット（中速）。環境に合わせて調整 |
| `mpc.domain_v_max` | 未設定 | `ROS_DOMAIN_ID` ごとの最高車速上書き。値は km/h |
| `mpc.domain_start_v_max` | 未設定 | `ROS_DOMAIN_ID` ごとのスタート限定最高車速上書き。値は km/h |
| `mpc.domain_start_v_max_duration` | `0.0` | `domain_start_v_max` を適用する開始後秒数 |
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
- 2025由来`boost_commander` / custom message / `use_boost_acceleration`の段階的削除
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
  domain_csv_path:
    1: "env/<バージョン名>/traj_mincurv.csv"
    2: "env/<バージョン名>/traj_mincurv_p2.csv"
    3: "env/<バージョン名>/traj_mincurv_p3.csv"
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
| AWSIM SIM状態 | **2026公式** | `/awsim/status`, `/awsim/state` |
| AWSIM Boost指令 | **2026公式・SIMのみ** | `/awsim/cmd` |
| Planning → Control | **不要** | MPC は独自 CSV を使う（`update_by_topic: false`） |

トピックインタフェースの互換性は高く、**リマップは不要**。

### 制御周期の差異

- simple_pure_pursuit: **100Hz**（`create_wall_timer(10ms)`）
- mpc_controller: **40Hz**（`config.yaml` の `control_rate: 40.0`）

MPC は計算負荷が高いため 40Hz は妥当。問題があれば `control_rate` を調整できる。

### Python venv

MPC の通常実行は C++ の `mpc_controller_cpp` で行う。現時点では `mpc_simulation`、reference path / velocity visualizer、Python 版 MPC との比較用途のため、CMakeLists.txt 内で Python 仮想環境も作成している（`execute_process` で `/usr/bin/python3 -m venv` + `pip install`）。Docker ビルド内で完結するため追加設定は不要だが、ビルド時間が増加する点に注意。

### 障害物回避

MPC コントローラは障害物回避機能を内蔵しているが、**今回の統合ではデフォルトで無効**（`use_obstacle_avoidance=false`）。将来的に必要になったら有効化できる。

**障害物情報の取得方法は2つ:**

1. **CSV ファイルから静的障害物を読み込み**
   ```yaml
   # config.yaml
   obstacles:
     csv_path: "maps/occupancy_grid_map_obstacles.csv"  # 空でなければCSVモード
     radius: 1.25  # 障害物の半径 [m]
   ```
   - `create_waypoints.py --obs` で GUI 上から障害物座標を手動で作成
   - `ObstacleManager` が occupancy grid map に障害物を追加し、MPC の制約計算で回避

2. **トピック経由で動的障害物を受け取り**
   ```yaml
   # config.yaml
   obstacles:
     csv_path: ""  # 空にするとトピック購読モード
     radius: 1.25
   ```
   - `/aichallenge/objects`（`Float64MultiArray`）から障害物座標を受け取る
   - データフォーマット: `[x, y, ?, ?, x, y, ?, ?, ...]`（4要素ずつ、x/y を使用）
   - 更新のたびに occupancy grid map をリセット → 障害物再配置 → 経路制約を再計算

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
<param name="use_obstacle_avoidance" value="true"/>
```

**C++ V2X gap planner（暫定拡張）:**

C++ の `mpc_controller_cpp` には、`/v2x/vehicle_positions` を使って他車位置を横方向 occupied interval に変換し、reference path の `lb/ub` から free gap を選ぶ rule-based planner を追加している。これは 2026 公式仕様として確定した機能ではなく、現行 MPC の実験用拡張である。

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
  v2x_low_speed_pass_side: auto      # auto, left, right
  v2x_low_speed_pass_ramp_ratio: 1.0
```

- 既定 `false` なので、通常走行では従来の trajectory tracking のまま。
- 有効時は `/v2x/vehicle_positions` を subscribe し、vehicle_id ごとの直近2点から速度を推定する。
- 他車は円近似し、horizon waypoint ごとに `lb/ub` を狭める。
- gap が選べる場合は `xr` を gap 中央へ寄せる。
- gap が壁と他車に挟まれている場合は、`v2x_wall_clearance_margin` で壁側制約を内側へ削り、`v2x_wall_avoidance_bias` で target を車側へ寄せられる。
- 左右が両方とも V2X 車両の gap は `v2x_vehicle_vehicle_gap_enabled=false` で候補から外せる。3台同時走行のスタート直後など、前方2台の間を狙うと操舵が不安定になる場面では、車-車 gap を使わず no-gap 低速追走へ倒す。
- 前方近距離に2台以上いる状況そのものを追い抜き禁止にしたい場合は、`v2x_multi_front_gap_enabled=false` と `v2x_multi_front_gap_distance` を使う。この条件に入ると gap planner は feasible gap を返さず、`no_gap_target_velocity` へ倒す。
- gap がない場合は `no_gap_target_velocity` を速度上限として使う。
- 実車・遠隔環境では使わず、先にシミュレータで wall / crash / over penalty を確認する。

**C++ V2X behavior FSM（暫定拡張）:**

`use_v2x_behavior_fsm=true` の場合は、既存 gap planner を常時使わず、`Cruise` / `Follow` / `Overtake` / `LowSpeedAvoidance` / `SafetyBrake` の最小状態で使用可否を制御する。これは高度な追い抜き戦略ではなく、ヘアピン入口などで不用意な横回避が単独走行の安定ラインを壊すことを抑えつつ、近距離停止車両を徐行回避するための暫定層である。

```yaml
mpc:
  use_v2x_behavior_fsm: false
  v2x_behavior_debug_log_enabled: false
  v2x_behavior_debug_log_period_sec: 1.0
  v2x_follow_distance: 8.0
  v2x_safety_brake_distance: 3.0
  v2x_safety_brake_margin: 2.0
  v2x_follow_gap_planner_enabled: false
  v2x_follow_gap_planner_no_gap_speed_limit_enabled: false
  v2x_follow_gap_planner_respect_overtake_forbidden: true
  v2x_follow_speed_limit_enabled: false
  v2x_follow_velocity: 5.0
  v2x_front_decel_guard_enabled: true
  v2x_front_decel_guard_distance: 9.0
  v2x_front_decel_guard_ttc: 1.5
  v2x_front_decel_guard_speed_margin: 0.5
  v2x_front_decel_guard_min_closing_speed: 1.5
  v2x_front_decel_guard_curve_include_slow_front: false
  v2x_front_decel_guard_curve_lateral_margin: 0.0
  v2x_front_decel_guard_curve_lookahead_distance: 0.0
  v2x_front_decel_guard_curve_distance: 16.0
  v2x_front_decel_guard_curve_ttc: 3.0
  v2x_front_risk_arbitration_enabled: false
  v2x_front_risk_brake_prepare_limit_enabled: false
  v2x_front_risk_avoid_candidate_limit_enabled: true
  v2x_front_risk_comfort_decel: 2.0
  v2x_front_risk_hard_decel: 4.0
  v2x_front_risk_emergency_decel: 6.0
  v2x_front_risk_distance_margin: 3.0
  v2x_front_risk_min_closing_speed: 0.5
  v2x_front_risk_prepare_time: 1.5
  v2x_front_risk_curve_limit_enabled: false
  v2x_front_risk_curve_limit_required_decel: 1.2
  v2x_front_risk_curve_limit_decel: 1.4
  v2x_front_risk_curve_limit_speed_margin: 0.5
  v2x_safety_brake_velocity: 0.0
  v2x_overtake_min_gap_width: 2.0
  v2x_overtake_max_curvature: 0.05
  v2x_overtake_forbidden_curve_lookahead_distance: 0.0
  v2x_overtake_gap_lookahead_distance: 0.0
  v2x_overtake_guard_enabled: true
  v2x_overtake_guard_min_gap_width: 2.5
  v2x_overtake_guard_min_gap_points: 3
  v2x_overtake_guard_min_prepare_distance: 8.0
  v2x_overtake_guard_max_lateral_shift: 1.2
  v2x_overtake_guard_reachable_gap_enabled: false
  v2x_overtake_guard_max_lateral_accel: 2.0
  v2x_overtake_guard_min_gap_time: 0.8
  v2x_overtake_guard_min_speed_for_reachable: 1.0
  v2x_overtake_guard_min_front_distance: 3.0
  v2x_overtake_close_follow_enabled: false
  v2x_overtake_close_follow_min_front_distance: 1.5
  v2x_overtake_close_follow_max_closing_speed: 0.8
  v2x_overtake_close_follow_min_side_clearance: 2.0
  v2x_overtake_before_curve_enabled: false
  v2x_overtake_before_curve_max_front_speed: 8.0
  v2x_overtake_before_curve_min_speed_advantage: 1.0
  v2x_overtake_continue_in_forbidden_enabled: false
  v2x_overtake_front_velocity_limit_enabled: true
  v2x_overtake_target_ramp_enabled: false
  v2x_overtake_target_ramp_ratio: 0.7
  v2x_overtake_line_enabled: false
  v2x_overtake_line_shift_distance: 8.0
  v2x_overtake_line_pass_distance: 8.0
  v2x_overtake_line_return_distance: 10.0
  v2x_overtake_line_lateral_offset: 1.2
  v2x_overtake_line_target_bias: 0.8
  v2x_overtake_line_min_wall_clearance: 0.8
  v2x_overtake_line_max_lateral_accel: 2.5
  v2x_overtake_line_max_target_change: 0.25
  v2x_overtake_line_return_clear_distance: 4.0
  v2x_overtake_line_phase_hold_time: 0.3
  v2x_overtake_line_debug_log_enabled: false
  v2x_moving_front_speed_threshold: 1.0
  v2x_moving_follow_speed_margin: 2.0
  v2x_moving_safety_brake_distance: 1.5
  v2x_moving_safety_brake_margin: 1.0
  v2x_moving_safety_brake_time_headway: 0.3
  v2x_start_grid_grace_time: 0.0
  v2x_require_gap_for_overtake: true
  v2x_low_speed_avoidance_enabled: false
  use_v2x_local_path_planner: false
  v2x_low_speed_avoidance_distance: 10.0
  v2x_low_speed_avoidance_lookahead_distance: 18.0
  v2x_low_speed_avoidance_velocity: 1.5
  v2x_low_speed_avoidance_max_front_speed: 1.0
  v2x_low_speed_avoidance_min_gap_width: 0.5
  v2x_low_speed_avoidance_min_gap_points: 2
  v2x_low_speed_avoidance_clear_distance: 8.0
  v2x_local_path_pass_clearance: 3.0
  v2x_local_path_return_distance: 6.0
  v2x_local_path_invert_target: false
  v2x_low_speed_pass_side: auto      # auto, left, right
  v2x_low_speed_pass_ramp_ratio: 1.0
  v2x_overtake_forbidden_wp_ranges: []
  v2x_state_hold_time: 0.5
```

- `Cruise`: 他車なし。V2X 由来の横目標変更は入れない。
- `v2x_behavior_debug_log_enabled=true` の場合、`v2x_behavior_debug_log_period_sec` 周期で V2X FSM の詳細ログを出す。ログには desired/final state、state hold 後の結果、front distance/speed、required decel、risk、overtake forbidden、curve guard、low-speed candidate、overtake zone、gap availability、block reason を含める。追い越しに入らず `Follow` に居続ける場合は、`reason` と `block` を確認する。
- `Follow`: 前方車あり、追い抜き禁止または gap 不足。レース中に競り負けた直後の不要な失速を避けるため、既定では速度制限を入れない。`v2x_follow_speed_limit_enabled=true` の場合だけ、停止/低速車には前方距離から計算した停止可能速度を使い、`v2x_moving_front_speed_threshold` より速い前走車には前走車速度 + `v2x_moving_follow_speed_margin` を上限にする。`v2x_follow_gap_planner_enabled=true` の場合は、Follow のままでも feasible な gap planner 出力だけを横制約と target に反映する。`v2x_follow_gap_planner_no_gap_speed_limit_enabled=false` なら、gap 不成立時の `no_gap_target_velocity` は Follow では使わず、譲り減速を避ける。`v2x_follow_gap_planner_respect_overtake_forbidden=true` なら、曲率または WP 範囲で overtaking forbidden の区間では Follow の gap planner も止め、ヘアピン入口で横に張りに行って進路を失う挙動を抑える。`v2x_follow_preposition_enabled=true` の場合は、WP 禁止ではないソフトな曲率禁止区間でも、カーブ外側かつ前方距離が残っているときだけ弱い lateral target を出し、真後ろ追従から外れる準備をする。通常の overtake pass side が内側を選んだ場合でも、Follow preposition だけはカーブ外側へ差し替える。
- `v2x_front_decel_guard_enabled=true` の場合、通常 Follow の速度制限を無効にしていても、近距離の動く前走車に対しては `front speed + v2x_front_decel_guard_speed_margin` の速度上限を掛ける。これは通常追走で失速させるためではなく、前走車の減速に追従できず追突するケースの緊急ガードである。`v2x_front_decel_guard_min_closing_speed` 未満の閉じ速度では発火させず、前走車の後ろに付いているだけの後続車を不要に失速させない。追い越し禁止カーブ中は `v2x_front_decel_guard_curve_distance` と `v2x_front_decel_guard_curve_ttc` まで判定距離を広げ、速度差を付けた車両がカーブで前走車へ追いつくケースを早めに抑える。直線で譲らせずヘアピンだけ追従させる場合は、`v2x_front_decel_guard_distance=0.0` と `v2x_front_decel_guard_ttc=0.0` にし、curve 側の距離/TTC だけを使う。ヘアピンで前走車が `v2x_moving_front_speed_threshold` 以下まで落ちる場合は、`v2x_front_decel_guard_curve_include_slow_front=true` にしてカーブ中だけ低速前走車も速度上限の対象にする。前走車が曲がり込みで横から進路を塞ぐ場合は、`v2x_front_decel_guard_curve_lateral_margin` でカーブ中の前方判定横幅を広げる。`v2x_front_decel_guard_curve_lookahead_distance` は速度上限用の曲率先読み距離で、`v2x_overtake_forbidden_curve_lookahead_distance` より短くすることで、横攻め停止だけを早めて失速開始を遅らせられる。
- `v2x_front_risk_arbitration_enabled=true` の場合、前走車との相対速度と有効距離から required decel を計算し、`EmergencyBrake` では SafetyBrake へ倒す。`BrakePrepare` は既定では警戒レベルとして扱い、`v2x_front_risk_brake_prepare_limit_enabled=true` の場合だけ速度制限に使う。`AvoidCandidate` も既定では速度制限に使うが、レース中に譲りすぎる場合は `v2x_front_risk_avoid_candidate_limit_enabled=false` で警戒レベルに落とせる。Phase 1 では reachable gap との統合は行わず、ブレーキ開始の遅れを切り分けるための braking guard として使う。
- `v2x_front_risk_curve_limit_enabled=true` の場合、曲率ガード中だけ required decel ベースの速度上限を追加する。直線の競り合いでは `BrakePrepare` / `AvoidCandidate` の速度制限を使わず、ヘアピンなど先行車が急減速する区間だけ `v2x_front_risk_curve_limit_decel` を前提に追突しない相対速度へ落とす。`v2x_front_risk_curve_limit_required_decel` は発火閾値、`v2x_front_risk_curve_limit_speed_margin` は前走車速度に足す余裕速度である。
- レース中に譲りすぎる場合は、距離/TTC ベースの `v2x_front_decel_guard_enabled` を `false` にし、required decel ベースの `v2x_front_risk_arbitration_enabled=true` へ切り替える。さらに `v2x_front_risk_brake_prepare_limit_enabled=false` と `v2x_front_risk_avoid_candidate_limit_enabled=false` にすると、通常の競り合いでは速度を落とさず、EmergencyBrake だけを残す。
- `Overtake`: 低曲率かつ十分な gap がある場合だけ gap planner を許可する。`v2x_overtake_guard_enabled=true` の場合は、gap 幅だけでなく、連続した gap 点数、gap までの準備距離、前方車との距離を追加確認してから Overtake に入る。`v2x_overtake_guard_reachable_gap_enabled=true` の場合は、最初に使える gap までの距離を現在速度から時間へ変換し、`2 * abs(target_ey - current_ey) / t^2` で必要横加速度を近似する。`v2x_overtake_guard_min_gap_time` 未満で現れる gap、または `v2x_overtake_guard_max_lateral_accel` を超える gap は Overtake 候補から外す。`v2x_overtake_guard_max_lateral_shift` は絶対横移動量の追加上限で、`0.0` の場合は無効である。早めに追い越し準備へ入るほど総横移動量は大きく見えるため、通常は `0.0` にして時間込みの lateral accel guard を優先する。これにより、通れない側へ一瞬振ってから反対側へ戻るような近距離 gap 飛び込みや、高速度では横移動が間に合わない gap へ突っ込む挙動を Follow / SafetyBrake 側へ倒す。`v2x_overtake_forbidden_curve_lookahead_distance` を指定すると、MPC horizon `N` より先の曲率まで見て overtake forbidden を立てるため、ヘアピン手前から横攻めを止められる。`v2x_overtake_gap_lookahead_distance` を指定すると、追い越し可否判定と Overtake 中の gap planner だけを MPC horizon より先まで伸ばして評価する。前方検出距離が MPC horizon より長い場合でも、低速前走車の先にある通過側 gap を早めに見つけられる。`v2x_overtake_target_ramp_enabled=true` では、長い lookahead 上で見つけた最初の追い越し target へ、現MPC horizon の先頭側から `v2x_overtake_target_ramp_ratio` の比率で target_ey を立ち上げる。これにより、前方車がまだ10点 horizon内に入っていない段階でも横方向の意思をMPCへ渡せる。
- `v2x_overtake_line_enabled=true` の場合、通常 `Overtake` 中の横参照を `ShiftOut` / `Pass` / `Return` / `Recovery` の内部フェーズで生成する。pass side は追い越し開始時に lock し、`v2x_overtake_line_lateral_offset` へ `smoothstep` で横移動する。`Pass` 中は同じ側を維持し、前方・横の V2X 車両が消えたら `Return` で基準 trajectory 側へ戻る。target は `lb/ub` から `v2x_overtake_line_min_wall_clearance` だけ内側へ clip し、必要横加速度が `v2x_overtake_line_max_lateral_accel` を超える場合は target 変化を抑える。`LowSpeedAvoidance` と `SafetyBrake` が優先で、停止車列の gate2 local path には適用しない。`LowSpeedAvoidance` へ入る前でも、前方車が `v2x_low_speed_avoidance_max_front_speed` 以下かつ低速回避 lookahead 内にいる場合は OvertakeLine を抑止する。既定は `false` で、既存の overtake target ramp を維持する。
- `v2x_overtake_close_follow_enabled=true` の場合、近距離で通常 fallback guard の `min_prepare_distance` を満たせないときでも、前方距離・横余裕・相対速度が安全側の範囲内なら `Overtake` の横 target だけを許可する。さらに通常 overtake guard と同じ `v2x_overtake_guard_min_gap_time` / `v2x_overtake_guard_max_lateral_accel` で横移動の到達性を確認し、至近距離で急なU字経路が必要になる場合は Follow / SafetyBrake 側へ残す。真後ろに詰まってから永久に Follow に落ちるケースを避けるための例外で、相対速度が大きい場合や emergency front risk では使わない。既定は `false`。
- `v2x_overtake_before_curve_enabled=true` の場合、WP 明示禁止ではなく曲率先読みだけで overtake forbidden になっている区間では、前走車が `v2x_overtake_before_curve_max_front_speed` 以下で、自車が `v2x_overtake_before_curve_min_speed_advantage` 以上速く、かつ `front_decel_guard_curve_lookahead_distance` ではまだガードされていない場合だけ、新規 Overtake を許す。これは長い曲率先読みで直線中の低速前走車に張り付く問題を抑えるための例外である。`v2x_overtake_continue_in_forbidden_enabled=true` の場合は、すでに Overtake 中なら同じ soft forbidden 区間で Overtake 継続を許し、ヘアピン前に横へ出た車両が途中で Follow に戻される挙動を抑える。
- `v2x_overtake_front_velocity_limit_enabled=true` の場合、`Overtake` 中でも前走車の required decel / front decel guard 由来の速度上限を掛ける。安全寄りだが、前走車速度へ引っ張られて追い越しが成立しない場合は `false` にする。`false` でも `EmergencyBrake` と inside stopping distance は Overtake 判定より先に評価されるため、近すぎる場合の SafetyBrake は残る。
- `LowSpeedAvoidance`: 近距離の低速前方車両に対して通過可能な側がある場合、SafetyBrake より先に `v2x_low_speed_avoidance_velocity` へ速度制限して徐行回避する。開始条件では `v2x_low_speed_avoidance_max_front_speed` 以下の V2X 推定速度を低速車両として扱う。いったん `LowSpeedAvoidance` に入った後は、local path または gap がまだ feasible で、対象車両が `v2x_low_speed_avoidance_clear_distance` 以内に残る限り、停止車基準の inside stopping distance と `EmergencyBrake` より LowSpeedAvoidance を優先する。これは gate2 のような停止車列回避で、基準 trajectory 上の前方距離だけを見た SafetyBrake により通過途中で停止しないためである。
- 低速回避では `v2x_low_speed_pass_side` で通過側を `auto` / `left` / `right` から選べる。`right` は reference path 座標系の負の lateral 側、`left` は正の lateral 側である。`auto` の場合は最初に選んだ側を低速回避中の side lock として使う。configured side に通過可能 gap がない場合は逆側へ無理に振らず、Follow / SafetyBrake 側へ倒す。
- `use_v2x_local_path_planner=true` の場合、`LowSpeedAvoidance` は従来の constraint-only gap planner ではなく、停止/低速車両列を reference path の `s/d` 座標へ射影し、選んだ側の「壁と膨張済み車両の間」を通る `target_ey` 列と横制約 `lb/ub` を生成する。この target は全 horizon 点で active になり、障害物が horizon 上で重なる前から MPC の `xr[e_y]` を通過側へ向ける。
- local path planner の target は `v2x_wall_avoidance_bias` を反映する。`0.0` は通路中央、`1.0` は膨張済み車両から `v2x_vehicle_side_target_margin` だけ離れた車両側寄りで、壁側へ膨らみすぎる場合は `0.5` から `1.0` の範囲で上げる。
- local path planner は通過中の horizon 後半で基準 trajectory 側へ戻さない。`LowSpeedAvoidance` が継続している間は選んだ通過側 corridor を制約にも反映し、MPC が反対側をすり抜け候補として選ばないようにする。
- local path planner の通過側 target への入り方は `v2x_low_speed_pass_ramp_ratio` を使う。停止車両までの横移動開始距離にこの比率を掛けた距離で target へ到達させるため、近距離開始で操舵が遅い場合は `0.2` から `0.4` 程度へ下げる。
- local path planner は `v2x_low_speed_avoidance_lookahead_distance` 内の低速車両を先読みし、選んだ側の通路が車列全体で成立する場合だけ `LowSpeedAvoidance` を許可する。成立しない場合は Follow / SafetyBrake 側へ倒す。
- `v2x_local_path_pass_clearance` は最後の低速車両を抜いた後も通過側 target を保持する距離、`v2x_local_path_return_distance` は基準 trajectory の `e_y=0` へ戻すブレンド距離である。
- `v2x_local_path_invert_target` は local path planner が選んだ `target_ey` を MPC へ渡す直前に反転する切り分け用設定である。RViz 上の通過方向と `e_y` 符号が逆に見える場合の検証に使い、恒久対応では座標系と操舵符号を整理する。
- `v2x_low_speed_pass_ramp_ratio` は、低速回避で通過側 target へ入る速さである。`use_v2x_local_path_planner=true` では停止車両手前の横移動距離を短くし、`false` の旧 gap planner 系では手前の horizon 点にも side-pass target をソフト参照として入れる。
- MPC の操舵レート制約は、前回出力した実操舵から horizon 先頭の操舵にも掛ける。これにより、MPC 表示が「実際にはまだ切れていない操舵」を前提にした進行方向を描くことを避ける。近距離停止車両の横抜けで操舵が遅い場合は `steer_rate_max` を上げるが、実効値は `steering_tire_angle_gain_var` で割った値になる。
- 近距離停止車両が `LowSpeedAvoidance` の距離条件に入っているが、連続した安全 gap が確認できない場合は、通常 `Overtake` へ落とさず `Follow` に倒す。これは回避ラインが確定する前に通常追い越しで横へ振り、停止車両の前を横切って接触することを防ぐためである。
- `LowSpeedAvoidance` 中は、曲率による追い越し禁止区間に入っても gap がある限り低速回避を継続する。さらに `v2x_low_speed_avoidance_clear_distance` 以内に V2X 車両が残る間は通常速度へ戻らず、横抜け後半での早すぎる Cruise 復帰を抑える。新規の通常追い越し開始は引き続き禁止条件に従う。
- 前方車判定は、進行方向前方にあり、かつ `v2x_vehicle_radius + v2x_prediction_margin` の横方向衝突幅に重なる車両だけに限定する。混走スタートの斜め横車両を前方車として `Follow` に落とすと片方だけ加速が遅れるため、横に並ぶ車両は `side vehicle` として扱う。
- 動いている前走車に対する SafetyBrake は相対速度ベースで判定する。停止車向けの大きな停止距離をそのまま使うと、競り負けた直後に不要な強制減速が入るため、`v2x_moving_safety_brake_distance`、`v2x_moving_safety_brake_margin`、`v2x_moving_safety_brake_time_headway` を別に持つ。
- `v2x_start_grid_grace_time` は、同時走行スタート直後に横車両がいる場合だけ停止車向けの `LowSpeedAvoidance` / `SafetyBrake` 判定を猶予する。スタート前に前走車の V2X 推定速度が 0 と見えることで、最後尾車が gate2 用の停止車回避へ誤って入ることを防ぐ。
- `LowSpeedAvoidance` に入った後も、最初の `target_ey` を絶対値として固定し続けない。3台以上の停止車列では車両ごとに通れる gap が変わるため、固定するのは通過側だけに留め、horizon 上の障害物に応じて目標を再選択する。
- `SafetyBrake`: 前方車が近すぎる、または現在速度から見た停止距離内に入っている。gap planner を使わず `v2x_safety_brake_velocity` に速度制限する。
- SafetyBrake 判定距離は `max(v2x_safety_brake_distance, v^2 / (2 * abs(a_min)) + v2x_safety_brake_margin)` とする。横方向は corridor 全体ではなく、`v2x_vehicle_radius + v2x_prediction_margin` の衝突幅に重なる場合だけ危険判定する。
- `v2x_require_gap_for_overtake=false` の場合は、追い越し禁止条件に入っていなければ gap 幅の事前判定を必須にせず `Overtake` へ遷移する。
- `LowSpeedAvoidance` は `front_distance <= v2x_low_speed_avoidance_distance`、かつ `v2x_low_speed_avoidance_min_gap_width` 以上の gap が `v2x_low_speed_avoidance_min_gap_points` 以上連続する場合に使う。
- `v2x_vehicle_radius` は V2X 車両単体の半幅ではなく、自車中心が入ってはいけない横方向禁止幅として扱う。V2X 車両幅 1.45m の場合、相手半幅 0.725m + 自車半幅 0.725m として 1.45m 程度を基準にし、必要に応じて余白を足す。前後方向の占有判定には `v2x_vehicle_length` と自車長を使う。
- `v2x_overtake_forbidden_wp_ranges` は `[start, end]` の配列で指定し、ヘアピン入口など追い抜き禁止区間の抑制に使う。ただし通常はコース位置固定の禁止より `v2x_overtake_guard_*` の条件で抑制する。
- 既定 `false` のため、通常設定では既存挙動を維持する。

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
| MPC 起動方式 | `<include control/mpc.launch.xml>` 経由で `mpc_controller_cpp` を起動（インライン node ではない） |
| パッケージ配置 | `aichallenge_submit/` 配下に統合済み（追加作業不要） |
| ビルド注意 | C++ MPC は `osqp_vendor` を使用。補助 Python venv 作成（pip install）によるビルド時間増加は残る |
