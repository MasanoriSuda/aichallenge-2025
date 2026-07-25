# Localization Scope

日本語 | [English](README.md)

Localization Scopeは、Automotive AI Challenge向けのオフライン自己位置推定解析
ツールです。trajectory CSV、rosbag内のGNSS・IMU・車両状態・EKF出力を重ね、
単一走行の状態確認とBaseline/Candidateの2走行比較を行います。

## 対応環境

Kaleidoscopeと同じく、運営提供のAI Challenge Dockerイメージと、このリポジトリを
元の構成でcloneした環境を対象とします。

```text
aichallenge/workspace/src/aichallenge_submit/
  multi_purpose_mpc_ros/tools/localization_scope/
```

このツールはROS nodeではありません。MCAPのROS messageを読むために
`rosbag2_py`、`rclpy`、`rosidl_runtime_py`を使用するため、AI Challengeの
Autoware commandコンテナ内で実行してください。HTML生成部分はPlotly、pandas、
NumPyへ依存しません。

## 他の参加者リポジトリへ導入する

`tools/localization_scope/`だけを所定位置へ配置した場合は、ソースディレクトリから
直接実行できます。

```bash
cd /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/localization_scope
python3 -m localization_scope --help
```

この方法では`multi_purpose_mpc_ros`のCMake統合は不要です。ただし、MCAP読込には
運営Dockerイメージ内のROS 2 Python packageが必要です。

次のコマンドでも実行できるようにする場合、

```bash
ros2 run multi_purpose_mpc_ros localization_scope
```

`tools/localization_scope/`配下以外にも、参加者側で以下を追加してください。

### 1. `CMakeLists.txt`

Python packageをインストール対象へ追加します。

```cmake
ament_python_install_package(localization_scope
  PACKAGE_DIR tools/localization_scope/localization_scope
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)
```

CLI wrapperもインストール対象へ追加します。

```cmake
install(PROGRAMS
  scripts/localization_scope
  DESTINATION lib/${PROJECT_NAME}
)
```

既存の`install(PROGRAMS ...)`がある場合は、その一覧へ
`scripts/localization_scope`を加えてください。

### 2. `package.xml`

すでに存在する依存は重複して追加しないでください。

```xml
<depend>rclpy</depend>
<depend>rosbag2_py</depend>
<depend>rosidl_runtime_py</depend>
<depend>sensor_msgs</depend>
<exec_depend>rosbag2_storage_mcap</exec_depend>
```

解析するmessageに応じて、次のpackageも必要です。通常のAI Challenge参加者
packageには既に含まれています。

```xml
<depend>autoware_auto_control_msgs</depend>
<depend>autoware_auto_planning_msgs</depend>
<depend>autoware_auto_vehicle_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
```

### 3. CLI wrapper

このリポジトリの
`multi_purpose_mpc_ros/scripts/localization_scope`を、同じ相対位置へ追加して
実行権限を付けます。

```bash
chmod +x \
  aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/scripts/localization_scope
```

### 4. テスト（任意・推奨）

`test/test_localization_scope.py`を追加する場合は、`CMakeLists.txt`の
`BUILD_TESTING`ブロックへ追加します。

```cmake
ament_add_pytest_test(test_localization_scope
  test/test_localization_scope.py
)
```

ステアリング文書は開発記録であり、ツール実行には不要です。

統合後はビルドしてCLIを確認します。

```bash
make autoware-build
make autoware-bash
ros2 run multi_purpose_mpc_ros localization_scope --help
```

## 重要な解析上の前提

- trajectoryは目標経路であり、車両位置の真値ではありません。
- trajectoryとEKFの差には、追従誤差と自己位置推定誤差の両方が含まれます。
- GNSS poseも真値ではありません。
- GNSS、EKF、trajectoryを同時に比較して原因を切り分けます。
- 追い越し等で経路が変化する場合は、静的CSVとは別に実行時trajectory topicを
  記録してください。

## 最初に記録するtopic

最低限、次のtopicをrosbagへ追加することを推奨します。

```yaml
- /clock
- /sensing/gnss/nav_sat_fix
- /sensing/gnss/pose_with_covariance
- /sensing/imu/imu_raw
- /sensing/imu/imu_data
- /vehicle/status/velocity_status
- /vehicle/status/steering_status
- /localization/imu_gnss_poser/pose_with_covariance
- /localization/twist_estimator/twist_with_covariance_raw
- /localization/twist_estimator/twist_with_covariance
- /localization/kinematic_state
- /planning/scenario_planning/trajectory
- /control/command/control_cmd
```

不足topicがあるbagも読み込めます。取得できない指標は`N/A`になり、レポートへ
警告が表示されます。

## `make dev`で自己位置推定を試験する

このツールは公式評価結果を確認するためだけのものではなく、参加者が
`make dev`で自分の自己位置推定を試走・デバッグすることを主用途とします。
リアルタイム監視ツールではなく、走行後のrosbagを解析するオフラインツールです。

```text
自己位置推定または設定を変更
        ↓
make devで試走
        ↓
rosbagを取得
        ↓
単一走行レポートで異常を確認
        ↓
Baseline/Candidateの2走行で変更効果を比較
```

### rosbagを有効にする

`make dev`を実行するだけでは、利用環境の設定によってはrosbagが生成されないか、
自己位置推定に必要なtopicが記録されません。

このリポジトリでは、次の設定が記録を管理しています。

```text
aichallenge/workspace/src/aichallenge_system/
  autostart_orchestrator_py/config/autostart_orchestrator.param.yaml
```

試験時は`enable_rosbag`を有効にし、前節の推奨topicを`rosbag_topics`へ追加します。

```yaml
enable_rosbag: true
rosbag_topics:
  - /clock
  - /sensing/gnss/nav_sat_fix
  - /sensing/gnss/pose_with_covariance
  - /sensing/imu/imu_raw
  - /sensing/imu/imu_data
  - /vehicle/status/velocity_status
  - /vehicle/status/steering_status
  - /localization/imu_gnss_poser/pose_with_covariance
  - /localization/twist_estimator/twist_with_covariance_raw
  - /localization/twist_estimator/twist_with_covariance
  - /localization/kinematic_state
  - /planning/scenario_planning/trajectory
  - /control/command/control_cmd
```

参加者リポジトリで別のrecorderを使っている場合は、その設定で同等のtopicを記録して
ください。トピック名を変更している場合は`run-metadata.json`の`topics`で対応付けます。

このリポジトリの標準成果物では、最新bagを次から確認できます。

```text
output/latest/d1/rosbag2_autoware.mcap
```

### 推奨する試験

いきなりレース走行だけを比較すると複数の原因が混ざるため、次の順序を推奨します。

| `test_type` | 試験 | 主に確認するもの |
|---|---|---|
| `stationary` | 30～60秒停止 | 車速バイアス、IMU yaw-rateノイズ、GNSS散布 |
| `straight` | 直線一定速 | 速度スケール、時刻ずれ、直進時yawドリフト |
| `constant_turn` | 左右の定常旋回 | yaw rate、IMUの符号、TF、左右差 |
| `accel_brake` | 加減速 | 車輪速の追従、スリップ、センサ周期 |
| `lap` | 単独周回 | trajectory偏差、コース区間差、周回再現性 |
| `race` | 複数車両 | 実行時trajectoryを含む最終的な挙動 |

最初は設定を変えないBaseline走行を記録し、その後は一度に1項目だけ変更した
Candidate走行を記録します。

比較時には、意図して変更する項目以外の次の条件を揃えてください。

- AWSIM version
- trajectory
- MPCなどの制御設定
- 目標速度と速度profile
- 試験種類と周回数
- 車両IDと開始条件

条件が複数同時に変わった場合もレポートは生成できますが、改善・悪化を
自己位置推定の変更だけに帰属できません。

## Metadataを作る

コンテナ内でソース版を直接実行する例です。

```bash
cd /aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/tools/localization_scope
python3 -m localization_scope init /path/to/run/run-metadata.json
```

利用者が最低限記入するのは、AWSIM version、表示名、試験種類、目標速度です。

```json
{
  "schema_version": "1.0",
  "run": {
    "display_name": "AWSIM 2026.2 medium baseline",
    "test_type": "lap"
  },
  "awsim": {
    "version": "2026.2"
  },
  "experiment": {
    "speed_profile": "medium",
    "target_speed_mps": 5.0
  }
}
```

省略項目には標準topic、標準trajectory、標準configが補われます。全項目は
`examples/run-metadata.example.json`、構造は
`schemas/run-metadata.schema.json`を参照してください。

試験種類は次から選びます。

```text
stationary
straight
constant_turn
accel_brake
lap
race
custom
```

## 単一走行レポート

```bash
python3 -m localization_scope report \
  /path/to/run/rosbag2_autoware.mcap \
  --metadata /path/to/run/run-metadata.json \
  --output-dir /path/to/run/localization-report
```

bagを1個だけ含むrunディレクトリを指定することもできます。

```bash
python3 -m localization_scope report /path/to/run/
```

出力:

```text
localization-report/
├── run-manifest.json
├── summary.json
└── report.html
```

## Baseline/Candidate比較

暫定版は2走行だけを比較します。

```bash
python3 -m localization_scope compare \
  /path/to/baseline-run/ \
  /path/to/candidate-run/ \
  --output-dir /path/to/comparison
```

出力:

```text
comparison/
├── comparison-summary.json
└── comparison.html
```

次の判定をmetricごとに表示します。

```text
改善
悪化
実質差なし
判定不能
```

AWSIM、trajectory、config、Git commitが複数同時に変わっている場合は、metricの
変化を単一原因へ帰属できないことを警告します。

## ブラウザで走行を選択する

複数のrunディレクトリをまとめた親ディレクトリを指定します。各runには
`run-metadata.json`とbagを1個ずつ置いてください。

```text
runs/
├── baseline/
│   ├── run-metadata.json
│   └── rosbag2_autoware.mcap
└── candidate/
    ├── run-metadata.json
    └── rosbag2_autoware.mcap
```

```bash
python3 -m localization_scope catalog /path/to/runs \
  --output-dir /path/to/catalog
```

出力された`catalog.html`では、単一走行とBaseline/Candidateの2走行比較を
ブラウザ上で切り替えられます。`runs-index.json`にはrun一覧と比較結果を保存します。

## 標準入力と上書き

未指定時は次を使用します。

- trajectory: `multi_purpose_mpc_ros/env/final_ver3/traj_mincurv.csv`
- MPC config: `multi_purpose_mpc_ros/config/config.yaml`
- localization config:
  - `imu_corrector/config/imu_corrector.param.yaml`
  - `imu_gnss_poser/config/imu_gnss_poser.param.yaml`
  - `aichallenge_submit_launch/config/vehicle_velocity_converter.param.yaml`
- launch: `aichallenge_submit_launch/launch/reference.launch.xml`

参加者が変更したパスやtopicは`run-metadata.json`で上書きできます。解析時に
解決したファイルパスとSHA-256は`run-manifest.json`へ保存されます。
標準trajectoryは周回経路として扱います。非周回試験では
`trajectory.circular`を`false`にしてください。

## 現段階の制限

- simulator ground truthとの比較は行いません。
- EKF innovation/NISは現行topicに存在しないため解析しません。
- 実行時trajectoryは最新snapshotをXY表示します。
- 3走行以上を同時に扱うtrend比較は未実装です。
- `.mcap.zstd`は事前に展開してください。

## License

Apache License 2.0
