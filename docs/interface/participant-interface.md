# 参加者インターフェース契約

> 参加者（提出者）が**変えてはいけない約束**を列挙します。「[守るべき約束（一覧）](#守るべき約束一覧)」で 7 項目を簡潔にまとめたあと、各節で技術的な詳細を補足します。評価システムが依存する安定面（契約）の集約であり、手順書ではありません。

関連ドキュメント:

- 2026 SW 部門ルール要約: [../spec/competition-rules.md](../spec/competition-rules.md)
- 2026 未確定事項・運営確認リスト: [../spec/open-questions.md](../spec/open-questions.md)
- 評価システム側契約（FSM・結果 JSON など）: [evaluation-interface.md](evaluation-interface.md)
- Compose オーバーレイ設計: [../spec/compose-overlays.md](../spec/compose-overlays.md)
- V2X / 多車両方針: [../spec/v2x-multivehicle.md](../spec/v2x-multivehicle.md)
- MPC 統合仕様: [../spec/mpc-integration.md](../spec/mpc-integration.md)
- リポジトリルート README: [../../README.md](../../README.md)
- ドキュメント命名・分類規約: [../README.md](../README.md)

---

## 1. 概要

この文書は現行リポジトリの参加者インターフェース契約を記述する。Automotive AI Challenge 2026 のAWSIM管理topicとgear topicは2026-07-12に[公式インターフェース仕様](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/specifications/interface.html)と照合した。V2X、評価、提出、自律スタック復帰など確認中の仕様を変更する場合は、先に [../spec/open-questions.md](../spec/open-questions.md) の該当項目を解消してから本契約を更新する。

評価環境は `docker_build.sh eval --submit <tar>` で封入イメージを生成し、そのイメージ内で `evaluation.launch.xml` を起動します。提出する tar.gz はイメージのビルドとランタイムに直接影響するため、以下の契約を守ることが評価の前提条件です。

契約は 4 つの軸に分かれます。

1. **提出パッケージの形式**（tar.gz レイアウト）
2. **制御方式の選択**（`control_method` 引数）
3. **ROS 2 トピック I/O**（subscribe/publish すべきトピック）
4. **ビルド・実行環境の前提**（eval イメージで固定・差し替えられる事項）

---

## 守るべき約束（一覧）

1. tar.gz 内の最上位ディレクトリ名は **必ず `aichallenge_submit/`** にする。異なる名前にすると eval ビルド時に展開後の `aichallenge_submit/` が空になり、参加者パッケージが一切ビルドされない。
2. tar.gz は**リポジトリ直下（Docker ビルドコンテキスト内）**に置く。リポジトリルート外のパスを指定すると `docker build` の `COPY` が解決できず eval イメージのビルドが失敗する。
3. エントリ launch ファイルは **`aichallenge_submit_launch` パッケージ内の `aichallenge_submit.launch.xml`** として提供する。このファイルを欠くと評価の launch ツリーが起動できない。
4. `control_method` に渡せる値は **`mpc`・`pure_pursuit`・`tiny_lidar_net`・`pilot_net`・`joycon` の 5 つのみ**（既定: `mpc`）。それ以外の値を渡すとどの制御ノードも起動せず車両が動かない。既定値を変更すると `control_method` を明示しない既存の起動経路の挙動が変わる。
5. 提出パッケージは最小インターフェース（AWSIM センサトピックの subscribe、`/localization/kinematic_state` と `/planning/scenario_planning/trajectory` の produce、`/control/command/control_cmd` の publish、`/set_initial_pose` サービスの advertise）をすべて満たす。いずれかのトピック名・型を変更すると localization / planning / control の連結が切れ、車両の起動・走行・評価ができなくなる。
6. 2026 SIMでBoostを使う場合は、車両Domain内の `/awsim/status` と `/awsim/cmd` を `std_msgs/msg/Float32MultiArray` で扱う。`/awsim/cmd.data[0]` の `0.0`→`1.0`以上の立ち上がりが発動条件であり、`/awsim/boost_cmd`、`Bool`、Domain 0の`/admin/awsim/*`を代用しない。
7. gear変更が必要な場合は、車両Domain内の `/control/command/gear_cmd` と `/vehicle/status/gear_status` を使う。スタック復帰を理由にDomain 0の`/admin/awsim/reset`、クロスドメイン転送、非公開のteleport / respawn手段で代用しない。

---

## 2. 提出パッケージの形式

### tar.gz の最上位ディレクトリ

`create_submit_file.bash` は次のコマンドで tar を生成します。

```bash
tar zcvf submit/aichallenge_submit.tar.gz -C ./aichallenge/workspace/src aichallenge_submit
```

tar 内の最上位エントリは `aichallenge_submit/` のみです。独自に tar を生成する場合も、内側の最上位ディレクトリ名は **必ず `aichallenge_submit/`** にしてください。

eval イメージの `Dockerfile` eval ステージは `/aichallenge/workspace/src/aichallenge_submit` を `rm -rf` したうえで `tar zxf /tmp/s.tgz -C /aichallenge/workspace/src` で展開します。ディレクトリ名が異なると展開後に `aichallenge_submit/` が空のままになり、参加者パッケージが一切ビルドされません。

### tar.gz の配置場所

`Dockerfile` eval ステージは `COPY ${SUBMIT_TAR} /tmp/s.tgz` でビルドコンテキスト（リポジトリルート）を基準にパスを解決します。既定パスは `submit/aichallenge_submit.tar.gz`（`ARG SUBMIT_TAR=submit/aichallenge_submit.tar.gz`）。別パスを指定する場合は `--submit <リポジトリ直下の相対パス>` を使います。

リポジトリルート外の tar を指定すると `docker build` の `COPY` が解決できず、eval イメージのビルドが失敗します。

### エントリ launch ファイル

`aichallenge_system.launch.xml` は `aichallenge_submit_launch` パッケージの `aichallenge_submit.launch.xml` を固定で `<include>` します。この launch ファイルは `reference.launch.xml` に委譲し、センシング・自己位置・プランニング・制御の全スタックを起動します。

```
aichallenge_submit.launch.xml
└── reference.launch.xml    ← 参加者が変更してよいエントリ
    ├── sensing（imu_corrector, racing_kart_gnss_poser, gyro_odometer）
    ├── localization（imu_gnss_poser, ekf_localizer, twist2accel）
    ├── planning（simple_trajectory_generator）
    ├── control（control_method で選択）
    └── map（lanelet2_map_loader）
```

`aichallenge_submit_launch` パッケージを提出物に含めない、または `aichallenge_submit.launch.xml` を削除すると、評価の launch ツリーが起動できません。

---

## 3. 制御方式の選択

### `control_method` の有効値

`reference.launch.xml` は以下の引数を持ちます。

```xml
<arg name="control_method" default="mpc"
     description="Select control: mpc, pure_pursuit, tiny_lidar_net, pilot_net, joycon"/>
```

各値が起動するノードと消費するセンサ:

| `control_method` | 起動するノード（パッケージ） | 主な入力トピック |
|---|---|---|
| `mpc`（既定） | `multi_purpose_mpc_ros`（C++、`mpc_controller_cpp`） | `/localization/kinematic_state`、`/planning/scenario_planning/trajectory`、下記の速度・IMU・操舵入力 |
| `pure_pursuit` | `simple_pure_pursuit`（C++） | `/localization/kinematic_state`、`/planning/scenario_planning/trajectory` |
| `tiny_lidar_net` | `tiny_lidar_net_controller`（Python） | `/scan`（`sensor_msgs/LaserScan`） |
| `pilot_net` | `pilot_net_controller`（Python） | `/image_raw`（`sensor_msgs/Image`） |
| `joycon` | `teleop_manager`（`teleop_manager` パッケージ） | （手動制御） |

各値は `control/<name>.launch.xml` を `<include>` する `<group if=...>` で実装されており、いずれも `/control/command/control_cmd`（`autoware_auto_control_msgs/AckermannControlCommand`）を publish します。

上記 5 値以外を渡すと、どの `<group if=...>` にも一致せず制御ノードが起動せず車両が動きません。既定値 `mpc` を変更すると、`control_method` を明示しない既存の起動経路の挙動が変わります。

---

## 4. トピック I/O 契約

評価可能な提出物が守るべき ROS 2 トピック契約です。型・方向はソースで確認済みです。2026 AWSIMのシミュレーション管理topicは公式仕様で確認済みで、AWSIMが各車両Domain 1〜4へ直接接続する。Domain bridgeを追加せず、Domain 0の管理面と車両DomainのI/Oを分離する。

> **アーキテクチャ補足**: `aichallenge_awsim_adapter` の launch（`aichallenge_awsim_adapter.launch.xml`）は現状**全行がコメントアウト**されており、`actuation_cmd_converter` ノードは起動しません。AWSIM は Autoware 標準のトピック名を直接 publish/subscribe します。

### (A) AWSIM → Autoware（参加者が subscribe してよい入力）

AWSIM が publish し参加者ノードが subscribe するトピックです（Autoware 側の購読実装から確認済み。AWSIM 側の正確な publisher 名は要確認）。

| トピック | 型 | 確認元 |
|---|---|---|
| `/sensing/imu/imu_raw` | `sensor_msgs/Imu` | `reference.launch.xml`（imu_corrector 入力）、MPCCの実yaw rate入力 |
| `/sensing/gnss/nav_sat_fix` | `sensor_msgs/NavSatFix` | `reference.launch.xml`（racing_kart_gnss_poser 入力） |
| `/vehicle/status/velocity_status` | `autoware_auto_vehicle_msgs/VelocityReport` | `reference.launch.xml`（vehicle_velocity_converter 入力）、MPCCのCOM前後・横速度入力 |
| `/vehicle/status/steering_status` | `autoware_auto_vehicle_msgs/SteeringReport` | MPCCの実タイヤ角入力、実車のraw_vehicle_cmd_converter入力 |
| `/vehicle/status/gear_status` | `autoware_auto_vehicle_msgs/GearReport` | 2026公式gear状態。gear変更またはスタック復帰を行う場合の任意入力 |
| `/clock` | `rosgraph_msgs/Clock` | シミュレーション時間（`use_sim_time=true`） |
| `/awsim/status` | `std_msgs/Float32MultiArray` | 2026公式AWSIM状態。index 5=`boostRemaining`、6=`isBoosting` |
| `/awsim/state` | `std_msgs/String` | 車両FSM。`Spawned, Grounded, Ready, Start, Finish` |

MPCCの9状態モデル移行（2026-09-10、2025由来のローカル暫定）では、上記の
`VelocityReport`、`Imu`、`SteeringReport`を同じ車両Domainで直接購読する。
既存topic名・型・管理面の責務は変更しない。位置は引き続き
`/localization/kinematic_state`のbase_link poseを使う。速度はCOMの前後・横速度、
yaw rateはIMUのangular_velocity.z、操舵は実タイヤ角であり、
`VelocityReport.heading_rate`から操舵角を逆算しない。

poseの元時刻以前に受信済みの各センサ値を選び、元時刻と受信時刻の鮮度を確認する。
送信済み指令履歴とともに共通モデルで現在・制御開始時刻へ予測する。
必要な公開入力や履歴が欠けた場合は通常解へ進まず、既存の停止出力を使う。
実車での同じ座標・信号意味と2026公式plant値は未検証。再生bagはこれらの
topicと元stampを含める必要があり、旧bagの欠測を暗黙のゼロ入力で補わない。

`tiny_lidar_net` 使用時の追加入力（要確認: AWSIM 側の `/scan` publisher 名は本リポジトリ外）:

| トピック | 型 | 確認元 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | `tiny_lidar_net_controller_node.py` の `create_subscription` |

`pilot_net` 使用時の追加入力（要確認: AWSIM 側のカメラトピック名は本リポジトリ外）:

| トピック | 型 | 確認元 |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | `pilot_net_controller_node.py` の `create_subscription` |

### (B) Autoware → AWSIM（参加者が publish すべき出力）

評価のために参加者パッケージが最終的に publish しなければならないトピックです。

| トピック | 型 | 確認元 |
|---|---|---|
| `/control/command/control_cmd` | `autoware_auto_control_msgs/AckermannControlCommand` | `pure_pursuit.launch.xml`（remap）、`mpc_controller_cpp.cpp`、`mpc_controller.py`（比較用 Python 版）、`tiny_lidar_net_controller_node.py`、`pilot_net_controller_node.py` |
| `/control/command/gear_cmd` | `autoware_auto_vehicle_msgs/GearCommand` | 2026公式gear指令。gear変更またはスタック復帰を行う場合の任意出力 |
| `/awsim/cmd` | `std_msgs/Float32MultiArray` | 2026 SIM Boostを使う場合の任意出力。index 0=`boostCommand` |

AWSIM はこのトピックを受けてカートを動かします。全制御方式（mpc / pure_pursuit / tiny_lidar_net / pilot_net）がこのトピックに収束します。

### SIM control mode要求の信頼性

`/awsim/control_mode_request_topic`は`std_msgs/msg/Bool`の`true`でAUTONOMOUS engageを
要求する車両Domain内topicであり、複数publisherを許容する。評価側
`autostart_orchestrator`のsuccessログはpublish APIの成功を示すだけで、AWSIMからの
acknowledgementではない。

現行MPC提出物は、DDS discoveryまたはAWSIM初期化が一発送信より遅い場合に備え、
SIMの`Ready`で即時要求して周期再送を開始し、`Start`でも再送windowを開始し直す。
各windowは実速度0.1 m/sを確認するまで最大5秒間、0.2秒周期で`true`を再送する。
この有限期間だけEvidence-free Stuck Recoveryの新規開始を
抑止する。発進確認後またはtimeout後は再送と抑止を終了し、通常のRecovery判定へ戻る。
この参加者側補強は`use_sim_time=true`かつ`simulation_mode=true`でのみ有効であり、
実車経路、Domain 0、`/admin/awsim/*`の責務を変更しない。

### 2026 Gear の任意契約

gear変更を行わない提出物に `/control/command/gear_cmd` と
`/vehicle/status/gear_status` は必須ではない。使用する場合は次を守る。

- 指令は `autoware_auto_vehicle_msgs/msg/GearCommand`、状態は
  `autoware_auto_vehicle_msgs/msg/GearReport` を使う。
- 公式のgear値は `1=NEUTRAL`、`2=DRIVE`、`20=REVERSE` である。
- AWSIMは `AckermannControlCommand.longitudinal.speed` を使用せず、
  `longitudinal.acceleration` を加速度指令として使用する。したがって負の
  `longitudinal.speed` だけを送っても後退契約にはならない。
- これらはAWSIMと車両Domain N内で直接通信する。自律スタック復帰の
  許可、REVERSE中の加速度符号、gear status acknowledgementの待機条件は
  [../spec/open-questions.md](../spec/open-questions.md) の未確定事項であり、
  topicが公開されていること自体は自律復帰の競技上の許可を意味しない。
- `/admin/awsim/reset` はDomain 0のシミュレーション全体管理用であり、
  参加者車両のgear変更やスタック復帰に使用しない。

### 2026 SIM Boost の任意契約

Boostを使用しない提出物に `/awsim/cmd` は必須ではない。使用する場合は次を守る。

- `/awsim/status` は `Float32MultiArray` 7要素で、index 5が残り回数、index 6がBoost中フラグ。
- `/awsim/cmd` は `Float32MultiArray` で、index 0を`0.0`から`1.0`以上へ立ち上げると発動する。
- 再発動可能な入力へ戻すにはindex 0を一度`0.0`へ戻す。
- 残り回数は起動引数で変わるため固定値にせず、`/awsim/status.data[5]`を正とする。
- `/awsim/status.data[6] >= 0.5`の間は新しい発動edgeを送らない。
- このI/Oは車両Domain内でAWSIMと直接通信する。`/admin/awsim/*`やクロスドメイン転送を使わない。

実車経路（`simulation=false`）のみ使用する追加出力:

| トピック | 型 | 用途 |
|---|---|---|
| `/control/command/actuation_cmd` | `tier4_vehicle_msgs/ActuationCommandStamped` | `raw_vehicle_cmd_converter` 経由で実車アクチュエータへ（シミュレーションでは未使用） |

### (C) 評価起動ハンドシェイク（参加者が提供すべきサービス）

| エンドポイント | 型 | 確認元 |
|---|---|---|
| `/set_initial_pose` | `std_srvs/srv/Trigger` | `imu_gnss_poser_node.cpp`（`initial_pose_service` パラメータで名前設定） |

オーケストレータ（`autostart_orchestrator_node`）は起動時にこのサービスを呼び出して初期自己位置を設定します。未提供の場合は `initial_pose_service_timeout_sec` 経過後にスキップされます。

サービス呼び出しが最初のGNSS受信より早く失敗した場合も、`imu_gnss_poser` は最初の有効な位置・方位の受信時に、サービスと同じ初期化処理で `/localization/initial_pose3d` をpublishし、その後にEKFを起動します。初期姿勢は観測のsource stampを保持します。

ローカルAWSIMでは、GNSS位置と同じsource stampのIMU絶対姿勢を、校正済みTFでGNSSアンテナ座標へ変換してからレバーアームを戻し、その`map`上の`base_link`姿勢で初期化します。未着・時刻不一致・無効な姿勢・TF欠落の組を公開せず、生のIMU quaternionを初期姿勢へ直接代入しません。これは2025由来のローカルセンサー契約であり、2026公式/実車の絶対方位供給を仮定しません。実車側の既定は明示された`raceline`初期化を維持します。詳細は[自己位置入力の校正](../spec/localization-calibration.md)。

### 評価可能な提出物が満たす最小インターフェース

提出パッケージは以下をすべて満たす必要があります。

1. AWSIM センサ入力を subscribe: `/sensing/imu/imu_raw`（Imu）、`/sensing/gnss/nav_sat_fix`（NavSatFix）、`/vehicle/status/velocity_status`（VelocityReport）
2. `/localization/kinematic_state`（`nav_msgs/Odometry`）を produce する（ノード名`ekf_localizer`が publish）
3. `/planning/scenario_planning/trajectory`（`autoware_auto_planning_msgs/Trajectory`）を produce する
4. `/control/command/control_cmd`（`autoware_auto_control_msgs/AckermannControlCommand`）を publish する
5. `/set_initial_pose`（`std_srvs/srv/Trigger`）を advertise する（`imu_gnss_poser` が実装）

いずれかのトピック名・型を変更すると、対応する localization / planning / control の連結が切れ、車両の起動・走行・評価ができなくなります。

自己位置の時刻は、状態を予測・更新した時点を表す。EKFの1回の処理では、予測の時間差、
観測遅延、pose／twist／Odometryのstampに共通の時刻を使う。TFも保持しているposeのstampを使う。
処理終了時の時計で古い状態を付け替えない。2026-09-09の移行では参加者内の
`aichallenge_ekf_localizer`へ起動元とpackage依存を変更し、ノード名・topic・service・QoS・引数は維持する。
提出物には同packageを含め、旧underlay版との同時起動をしない。基底イメージの手編集は不要。
[時刻不整合の再現と移行設計](../../.steering/20260909-mpcc-ego-viability-transition/design.md)を参照。

---

### MPCC予測表示の内部契約

`/mpc/prediction` と
`/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/virtual_wall`
は、現行MPC launchではMPCCが専有する `visualization_msgs/msg/MarkerArray` 表示トピック。
制御指令として使用しない。C++実装の予測表示は各メッセージ内の `DELETEALL` に続く
`SPHERE_LIST`（namespace `mpc_pred`、ID 0）の `points` に全予測点を順番に格納する。
`map` 座標、球径0.5m、車両別RGBAと既存の約4Hzの送信頻度を維持する。
毎回の置換で、旧実装の個別 `SPHERE` IDや短縮前の点も残さず消す。

標準RVizのMarkerArray表示はそのまま使用できる。個別Markerの `pose.position` を
読む独自subscriberは、単一リストの `points` を読むよう移行する。
他の表示producerを同じトピックへ追加する場合は、`DELETEALL` の影響を避けるため
専用トピックへ分離する。参照経路表示と最終制御指令の契約は別に維持する。

## 5. ビルド・実行環境の前提

### eval イメージが行うこと（固定事項）

`Dockerfile` eval ステージは以下を実行します。この部分は参加者が変更できません。

1. アップストリームをクローンしてクリーンな `/aichallenge` ツリーを得る
2. `/aichallenge/simulator` と `/aichallenge/workspace/src/aichallenge_submit` を削除
3. 提出 tar.gz を `/aichallenge/workspace/src` に展開（→ `/aichallenge/workspace/src/aichallenge_submit/`）
4. ローカルの `aichallenge/simulator/`（AWSIM バイナリ + データ）をイメージに戻す
5. `rosdep install` + `colcon build --symlink-install --allow-overriding gyro_odometer --cmake-args -DCMAKE_BUILD_TYPE=Release` を実行

この結果、**`aichallenge_system/` 以下（`autostart_orchestrator_py`、`aichallenge_awsim_adapter` 等）はアップストリームのものが使われます**。参加者提出物はステップ 3 の `aichallenge_submit/` 展開のみです。

### eval イメージで変更できないもの

| 項目 | 固定値 | 理由 |
|---|---|---|
| RMW 実装 | `rmw_cyclonedds_cpp` | イメージに bake 済み |
| `CYCLONEDDS_URI` | `file:///opt/autoware/cyclonedds.xml` | イメージに bake 済み |
| アップストリーム `aichallenge_system/` | クローン時の HEAD | eval ステージでクローン |
| AWSIM バイナリ | ローカルの `aichallenge/simulator/` | ステップ 4 でコピー |
| `colcon build` オプション | `--allow-overriding gyro_odometer` 固定 | `Dockerfile` に記述 |

### .env・make は評価環境側の設定

`.env`（`COMPOSE_FILE`、`HOST_UID`、`ROS_DOMAIN_ID` 等）および `Makefile` のターゲットは評価環境（運営）側が管理します。参加者は `.env` や `Makefile` を直接変更して評価を制御することはできません。

提出 tar.gz 内に `.env` や `Makefile` を含めても、eval イメージはホストから切り離して動作するため効果がありません。評価環境の挙動を変えたい場合は、`aichallenge_submit/` パッケージの launch 引数・パラメータ YAML の変更で対応してください。

---

## 6. 関連ドキュメント

| ドキュメント | 内容 |
|---|---|
| [evaluation-interface.md](evaluation-interface.md) | 評価 FSM（`autostart_orchestrator` / `awsim_state_manager`）、結果 JSON スキーマ、AWS 評価パイプライン |
| [../spec/compose-overlays.md](../spec/compose-overlays.md) | `COMPOSE_FILE` の GPU/CPU/headless 選択肢 |
| [../spec/mpc-integration.md](../spec/mpc-integration.md) | MPC 制御器（`multi_purpose_mpc_ros`）の統合仕様 |
| [../spec/makefile-target-naming.md](../spec/makefile-target-naming.md) | make ターゲット命名規約 |
| [../README.md](../README.md) | ドキュメント命名・分類規約 |
| [../../README.md](../../README.md) | リポジトリルート README |

### MPCCの共通入力認証と保存データ（2026-09-11、候補）

通常のMPCC指令は、既存の解/trajectory/commandの一致に加え、元観測と公開履歴から
一つの共通指令列・全応答のモデル内静止までを検査した追加certificateを要求する。
Stop artifactの `applied_stop_program` はsource IDs、元観測、入力profile、実公開予定列、
静止時刻を保持する。`problem_context.applied_program_fingerprint` がそのデータを結合し、
async互換性・artifact検証・最終publish前照合へ引き継ぐ。profileはlive requestで必須。
旧保存データのfield省略はfingerprint0として旧ハッシュで読めるが、新live実行の省略許可ではない。
旧入力を新しいfloat32変換へ手修正して再生成功にしない。ROS topic/型、launch、評価JSONの
既存契約は同じであり、変更はMPCC内部の保存/認証データに限る。
受信age250msは2025 AWSIM実測から選んだ経験的profileで、2026公式/実車の保証ではない。
詳細と未完了の統合受入れは[統合仕様](../spec/mpc-integration.md)を参照。

元の解の時間範囲から生成する停止候補では、元sourceの残り状態の最小速度上限を
`applied_stop_program.forward_velocity_ceiling_mps`に保存し、全応答と後続Stopで再検査する。
この項目を持つ保存形式は`applied-stop-provenance-v2`。項目の欠落・非正値・非有限値と
schemaの不一致を拒否し、fingerprintへ結合する。項目を持たない従来形式はv1と元の
fingerprintを維持する。v1だけを理解する旧readerはv2を拒否するため、再生readerも
同時更新する。通常指令の公開1周期の権限と、別途全停止まで認証する停止列は区別する。
