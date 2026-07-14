# MPC Baseline Manifest

- 記録日: 2026-07-14
- 状態: **Baseline candidate v0（未凍結）**
- 用途: 疎結合化前の静的 snapshot と、runtime baseline 取得計画

この文書に hash が載っているだけでは挙動凍結は完了していない。runtime evidence、既知差分の判断、比較 tolerance が揃った時点で `Baseline v1` に昇格する。

## 1. Repository identity

| 項目 | 値 |
|---|---|
| branch | `develop_july` |
| commit | `50e0de5dcd3861565c78c501a6f3db0ca8e9489d` |
| tracked working tree | 記録時点で clean |
| untracked | `.steering/20260714-mpc-refactor/` |
| submodule | なし |

再取得コマンド:

```bash
git branch --show-current
git rev-parse HEAD
git status --short
git submodule status --recursive
```

`point-out-by-chatgpt-pro.md` は設計上の入力資料であり、現行契約や実挙動の正本ではない。

## 2. Canonical launch route

### 2.1 評価

```text
aichallenge/run_evaluation.bash
  -> aichallenge_system_launch/evaluation.launch.xml
     +-> Domain 0: aichallenge/run_simulator.bash eval
     |               -> simulator_scripts/eval.sh -> AWSIM
     +-> Domain 0: mode/awsim_state_manager.launch.xml
     +-> Domain N: aichallenge_system.launch.xml
                    -> aichallenge_submit.launch.xml
                    -> reference.launch.xml (default: control_method=mpc)
                    -> control/mpc.launch.xml
                    -> multi_purpose_mpc_ros/mpc_controller_cpp
```

### 2.2 開発

```text
Makefile
  +-> simulator service
  |     -> aichallenge/run_simulator.bash dev<N>/gate<N>
  |     -> simulator_scripts/dev.sh または gate.sh -> AWSIM
  +-> Autoware service
        -> aichallenge/run_autoware.bash
        -> aichallenge_system.launch.xml
        -> participant 側は評価と同じ経路
```

標準経路の主要引数は `use_sim_time=true`、`use_obstacle_avoidance=false`、legacy `use_boost_acceleration=false`、`use_stats=false`。`aichallenge_submit.launch.xml` は現状 `control_method` を宣言・転送していないため、標準経路は `reference.launch.xml` の default `mpc` に依存する。

package 内の `multi_purpose_mpc_ros/launch/mpc_controller.launch.py` は別の Python/legacy/helper 構成であり、上記 canonical production route には含めない。

## 3. Static hashes

### 3.1 Runtime source/package

| ファイル | SHA-256 |
|---|---|
| `multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp` | `f84b8659caad84d3e8f2ade3bed69ac1cde2e986f0f0fb43a20bab1a90228ff9` |
| `multi_purpose_mpc_ros/CMakeLists.txt` | `1d4a56f1452a5153e99d325e71b2146bffa6d84634917905ec024197e72078b7` |
| `multi_purpose_mpc_ros/package.xml` | `c7adee0685a67604ba147089b5aac310fc73a43b4b4afd94bc0e837efdd63058` |

### 3.2 Launch entry

| ファイル | SHA-256 |
|---|---|
| `aichallenge/run_evaluation.bash` | `9556f37412ba5847b20f35d03b8a19f81e353fddb9f62d66cad1628e8db207a7` |
| `aichallenge/run_autoware.bash` | `cdd6bc821082302edb331034d1a99a55b67699617fccb0ae5603181a7dd3af79` |
| `aichallenge/run_simulator.bash` | `940687cc9a72667ced05b8e7c92bb832d016488b97ee3068a221730986115a43` |
| `aichallenge/simulator_scripts/dev.sh` | `9103e318bce713e40fdf4da4867adf1845793d84e34ea127282fdd89cdc430fc` |
| `aichallenge/simulator_scripts/gate.sh` | `7c3985e284b1c3deb5ef06f0ea34180d703dc96f34621787ee2852fccb5d5903` |
| `aichallenge/simulator_scripts/eval.sh` | `f6c6b3d1b85210f700b84b13c5577b737190cd2a46acb1b4da78fb43d0481cad` |
| `aichallenge_system_launch/launch/evaluation.launch.xml` | `098292c32c6700b7484f9a299b95c17fce6a162f92c36179099bfa415f28dced` |
| `aichallenge_system_launch/launch/aichallenge_system.launch.xml` | `09e3a3ff5e604bbb26dfc591d0f036edbf257ec6396bcd9a738beeb5b61f9433` |
| `aichallenge_system_launch/launch/mode/awsim_state_manager.launch.xml` | `c4b84c2fc7d22a672b71d9403b108c34b0aae85fbaddd0fa91882b276e14cf6e` |
| `aichallenge_submit_launch/launch/aichallenge_submit.launch.xml` | `0a82a035882bef28d3c35856f6d8b82a6207711eac5b7ecc94928309523e4537` |
| `aichallenge_submit_launch/launch/reference.launch.xml` | `5609915eb9a993313ba83deee6c6bd6c7fb841e0427d48560b279625b44fe55c` |
| `aichallenge_submit_launch/launch/control/mpc.launch.xml` | `ddb1bd98a76c5777d042531f04b22fcadddc546044a2794d72d61f345de94bba` |

### 3.3 Config/resources

| ファイル | SHA-256 | 備考 |
|---|---|---|
| `multi_purpose_mpc_ros/config/config.yaml` | `312af3289241cf28ae1d36df701dd10f1461a0d5c13d64ce8e38a9a5c30b8c61` | canonical flat config |
| `multi_purpose_mpc_ros/config/ref_vel.yaml` | `282948a20c90ea7ca2ff6f91e19e5a43ef602ebe8ef23cd3e3fda5aea1625509` | reference velocity sections |
| `env/final_ver3/traj_mincurv.csv` | `c6a92f64936fbdead8976e6fbbdfbc9fe87928e1db2a60d671e69883369a5c60` | Domain 1/4/default、header + 459 records |
| `env/final_ver3/traj_mincurv_org.csv` | `9d15efde0b777c624ad0318d153112caa21b7c2296a29d8eae281a1ad4862a48` | Domain 2/3、header + 350 records |
| `env/final_ver3/occupancy_grid_map.yaml` | `39d5aba44234c1e09fc57421d467d64c1769259cd3c3656032f008d2db4e6a79` | map metadata |
| `env/final_ver3/occupancy_grid_map.pgm` | `c24af2130a8df96047a49d5b7759af7a0b29645c023e7f3bc6d4ba436275725a` | occupancy grid |
| `aichallenge_submit_launch/map/lanelet2_map.osm` | `126a62f2ef21560ace0eb39a0f52a6a5e86ef6b9951b0ead1511e46e4b418dc4` | participant map |
| `simple_trajectory_generator/data/raceline_awsim_30km_from_garage.csv` | `4c52d7ac0d987d9a098c0cf0abe7c322c35591243528ea66ae17fdec54f49607` | planning trajectory、header + 131 records |

両 MPC CSV の header は次で一致する。

```text
s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
```

### 3.4 Canonical AWSIM snapshot

| ファイル | SHA-256 |
|---|---|
| `aichallenge/simulator/AWSIM/AWSIM.x86_64` | `0bfe51325720c950b4ad75ce7c3b65595fd7e939908520ec33600919919327a8` |
| `aichallenge/simulator/AWSIM/UnityPlayer.so` | `609f28118ecf1a6e2cc545c87a5115d87607e47737d3ea143f167501722b2a13` |
| `aichallenge/simulator/AWSIM/AWSIM_Data/globalgamemanagers` | `ed3648e21d62227b390d18591292eccc4f40451eda12ac4e04323e8c208663f0` |
| `aichallenge/simulator/AWSIM/AWSIM_Data/app.info` | `0a881daecf13af664ab653cc7b319730e4028034ab9fe70d5f474ee672322bd2` |

canonical AWSIM directory は 1,088 files、688,493,822 bytes。相対 path 順に各 regular file の SHA-256 を並べ、その出力を再度 SHA-256 化した tree hash は `7ccdb298c5ff624447dea601554c808e1ded9189810badf97c68528000a6273e`。

`AWSIM.x86_64` の記録時 ELF Build ID は `cda4a1f924e8db7fb62ef2d8a3379337e4eb5e62`。canonical dev/dev4/gate/eval はいずれもこの単一 path を使い、複数車両は `--vehicles N` で起動する。`AWSIM_1` などの別コピーを使う parallel/multiplay は今回の baseline 対象外とする。

### 3.5 Existing test surface

現行 `CMakeLists.txt` には、5 GTest target、2 process-level CTest、9 pytest target が定義されている。path、Boost、V2X overtake core、Recovery core/footprint、trajectory tool の基礎回帰としてすべて維持する。

一方、単一 executable 内にある現行 `MPC`、QP 生成、OSQP 呼出、制御 1 周期全体を直接固定する test target はない。既存 test の成功だけを挙動凍結完了とはせず、Phase 0 で characterization seam と fixture を追加する。

## 4. Active configuration snapshot

標準 eval 経路から読み取った candidate 値。`ros2 param dump` で確認できるのは宣言済み ROS parameter だけであり、YAML 内部設定の大半は含まれない。runtime では config hash、startup log、Domain 別 resolved-config fixture、宣言済み parameter dump を組み合わせて確定する。

| 項目 | Candidate value |
|---|---|
| controller | `control_method=mpc` / `mpc_controller_cpp` |
| execution | `SingleThreadedExecutor` / 40 Hz |
| `simulation`, `use_sim_time` | `true`, `true` |
| legacy boost launch arg | `false` |
| official AWSIM Boost | global enabled、Domain 1..3 explicit enabled、Domain 4 fallback enabled、start-once |
| stuck recovery | global disabled、Domain 1..3 explicit disabled、Domain 4 fallback disabled |
| reference update by topic | disabled |
| path constraint/border topic | disabled |
| Domain 1/4/default CSV | `traj_mincurv.csv` |
| Domain 2/3 CSV | `traj_mincurv_org.csv` |
| horizon | `N=30` |
| `v_max` | 40 km/h |
| Domain 2 start override | 37 km/h、15 s |
| acceleration limits | `a_min=-1.35 m/s^2`、`a_max=1.0 m/s^2` |
| raw/internal steering angle limit | 32 deg |
| final static angle bound | gain 適用後の再 clamp なし。理論上 48 deg (`32 * 1.5`) |
| configured steering-angle slew limit | 1.2 rad/s |
| steering gain | 1.5 |
| raw/internal initialized slew limit | 0.8 rad/s (`1.2 / 1.5`) |
| final nominal angle slew limit after gain | 1.2 rad/s |
| raw/final message `steering_tire_rotation_rate` field | fixed 2.0 rad/s（slew limiter と別物） |
| odometry timeout | 0.5 s |
| reference resolution | 0.6 m |
| occupancy map resolution | 0.1 m |
| V2X gap planner | enabled |
| V2X behavior FSM | enabled |
| overtake line | enabled |
| low-speed avoidance | enabled |
| local path planner | enabled |
| launch obstacle avoidance | disabled |

Domain 別の静的 effective 値と override source は次のとおり。Domain 4 の競技挙動が 2026 公式評価対象かは未確定だが、現行 `make dev4` と Domain 1..N 契約の interface baseline には含める。

| Domain | path | `v_max` | `a_max` | start override | Boost | Recovery |
|---|---|---:|---:|---|---|---|
| 1 | explicit `traj_mincurv.csv` | explicit 40 km/h | explicit 1.0 m/s² | なし | explicit true | global false + explicit false |
| 2 | explicit `traj_mincurv_org.csv` | explicit 40 km/h | explicit 1.0 m/s² | 37 km/h / 15 s | explicit true | global false + explicit false |
| 3 | explicit `traj_mincurv_org.csv` | explicit 40 km/h | explicit 1.0 m/s² | なし | explicit true | global false + explicit false |
| 4 | global fallback `traj_mincurv.csv` | global fallback 40 km/h | global fallback 1.0 m/s² | なし | global fallback true | global fallback false |

## 5. ROS contract oracle

runtime 観測をそのまま正しい契約として golden 化しない。次の正本文書上の期待値を oracle とし、実 graph を PASS/FAIL で比較する。

| Domain | endpoint | type | contract direction / owner |
|---|---|---|---|
| Vehicle N | `/sensing/imu/imu_raw` | `sensor_msgs/msg/Imu` | AWSIM publish、participant localization subscribe |
| Vehicle N | `/sensing/gnss/nav_sat_fix` | `sensor_msgs/msg/NavSatFix` | AWSIM publish、participant localization subscribe |
| Vehicle N | `/vehicle/status/velocity_status` | `autoware_auto_vehicle_msgs/msg/VelocityReport` | AWSIM publish、participant stack subscribe |
| Vehicle N | `/clock` | `rosgraph_msgs/msg/Clock` | simulation clock |
| Vehicle N | `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | participant stack が produce、MPC が consume |
| Vehicle N | `/planning/scenario_planning/trajectory` | `autoware_auto_planning_msgs/msg/Trajectory` | participant planning が produce、controller が consume |
| Vehicle N | `/control/command/control_cmd` | `autoware_auto_control_msgs/msg/AckermannControlCommand` | participant の選択 controller が publish、最終 owner は一つ |
| Vehicle N | `/set_initial_pose` | `std_srvs/srv/Trigger` | participant stack が advertise |
| Vehicle N | `/v2x/vehicle_positions` | `v2x_msgs/msg/V2XVehiclePositionArray` | V2X system が publish、participant が subscribe |
| Vehicle N | `/awsim/state` | `std_msgs/msg/String` | AWSIM publish、vehicle orchestrator/participant subscribe |
| Vehicle N | `/awsim/status` | `std_msgs/msg/Float32MultiArray` | AWSIM publish、participant subscribe |
| Vehicle N | `/awsim/cmd` | `std_msgs/msg/Float32MultiArray` | participant publish、AWSIM subscribe（任意 Boost） |
| Vehicle N | `/control/command/gear_cmd` | `autoware_auto_vehicle_msgs/msg/GearCommand` | participant publish（gear 使用時のみ） |
| Vehicle N | `/vehicle/status/gear_status` | `autoware_auto_vehicle_msgs/msg/GearReport` | AWSIM publish、participant subscribe（gear 使用時のみ） |
| Vehicle N | `/awsim/control_mode_request_topic` | `std_msgs/msg/Bool` | orchestrator/operation path publish、AWSIM subscribe。MPC façadeは新規 pub/sub しない |
| Domain 0 | `/admin/awsim/start` | `std_msgs/msg/Bool` | `awsim_state_manager` 所有の管理経路。participant は触れない |
| Domain 0 | `/admin/awsim/reset` | `std_msgs/msg/Empty` | 管理/operation path publish。participant は触れない |
| Domain 0 | `/admin/awsim/state` | `std_msgs/msg/String` | AWSIM publish、`awsim_state_manager` subscribe |

現行 `docs/interface/` は上表 endpoint の QoS 値を一般契約として固定していない。このため QoS は runtime 観測値を `Current QoS Compatibility Oracle` として別保存し、リファクタ前後で維持する。不一致は説明が付くまで compatibility blocker だが、正本文書に値がないものを external-contract RED とは呼ばない。`docs/spec/mpc-integration.md` に明記済みの `/awsim/cmd` Reliable と gear の Reliable / KeepLast(1) / Volatile は current component expectation として併記する。

契約済み `control_method` は `mpc`、`pure_pursuit`、`tiny_lidar_net`、`pilot_net`、`joycon` の 5 値で、既定は `mpc`。実装に存在する `rl_train` は現時点では非契約の開発用 option であり、本リファクタ内で契約値へ昇格させない。

### 5.1 MPC node candidate surface

これはコード/launch から得た MPC node の静的候補であり、型/owner は contract oracle、QoS/active endpoint 数は current compatibility oracle と runtime graph で照合する。標準 `/rosout` と `/parameter_events` は一覧から除外する。

#### Publishers

- `/control/command/control_cmd` — 最終 command、通常構成で owner は `/mpc_controller` 一つ
- `/control/command/control_cmd_raw`
- `/mpc/prediction`
- `/mpc/ref_path`
- `/ref_vel_marker`
- `/section_marker`
- `/awsim/cmd`
- `/planning/scenario_planning/lane_driving/motion_planning/obstacle_stop_planner/virtual_wall`
- `/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/bound`

#### Subscribers

- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- `/control/mpc/stop_request`
- `/awsim/status`
- `/awsim/state`
- `/aichallenge/pitstop/condition`
- `/v2x/vehicle_positions`
- 実装上の `/control/control_mode_request_topic`（repository 内 producer なし）

Recovery と path constraint は canonical config で無効なため、それらの optional endpoint が非 active であることも baseline の一部とする。

## 6. 既知差分・判断台帳

`Baseline v1` の前に、各項目を「維持」「既存契約への適合修正」「外部契約を変えない安全修正」「機能性能上の waiver」のいずれかに分類する。hard-safety RED と external-contract RED は waiver や「対象外」で通過させず、Baseline v1 承認を block する。

| ID | 観測事項 | Candidate v0 の扱い | Baseline v1 までの判断 |
|---|---|---|---|
| D-01 | interface contract の `control_method` は 5 値だが `reference.launch.xml` に `rl_train` がある | `rl_train` は非契約開発 option | 非契約のまま隔離/削除を判断。契約値への昇格は本計画外 |
| D-02 | `mpc-integration.md` の V2X 無効記述と config の V2X enabled が不一致 | runtime は enabled 候補 | 文書を現状に合わせるか、意図を確認する |
| D-03 | 文書は Recovery takeover を説明するが config は全 Domain disabled | dormant として記録 | runtime 等価性と pure-rule test の境界を確定する |
| D-04 | 公式 AWSIM engage `/awsim/control_mode_request_topic` と MPC 内部 subscription `/control/control_mode_request_topic` が異なる | repository 内で内部側 publisher は見つからない | legacy subscription の維持/削除を判断。公式側への統合・新規 pub/sub はしない |
| D-05 | `use_sim_time` が Boost/Recovery の simulation safety gate を兼ねる | 現状を記録 | 安全問題として別修正が必要か判断する |
| D-06 | canonical C++ launch と package 内 Python/legacy launch が併存する | canonical route のみ凍結 | tool/legacy 整理は Phase 5 以降 |
| D-07 | 一つの flat YAML に Boost/Recovery/map/MPC/V2X/domain 設定が混在する | key/value を維持 | typed config への変換は互換 loader の内側で行う |
| D-08 | empty V2X ID を同じ `__unknown__` として扱う経路がある | 境界 fixture を取る | behavior defect なら Phase 0b で独立修正する |
| D-09 | raw 32 deg clamp 後に gain 1.5 を適用し、final の再 clamp がない | static 上の final 上限は 48 deg | 実測と安全限界を照合し、違反なら Phase 0b で独立修正する |
| D-10 | 各周期の速度 profile 上書きが base path と dynamic reference を混在させる | 値列を fixture 化 | 意図的挙動か副作用かを判定する |
| D-11 | solver failure が overtake recovery state を直接変更する | state sequence を凍結 | 明示 feedback 化しても周期 semantics を維持する |

`point-out-by-chatgpt-pro.md` 内の数値や所見は、この台帳と現行コードで再検証してから採用する。例として現行 `a_min` candidate は `-1.35 m/s^2` であり、同資料の `-1.6` は現在値ではない。

機能性能上の known RED を例外承認する場合だけ、owner、issue、期限、適用 scenario、承認者を waiver に残す。D-04 は責務を確定するまで、D-09 が安全違反と判定された場合は解消するまで Baseline v1 に進めない。

## 7. Hard-safety oracle candidate

Phase 0 の走行取得前に、各行の authoritative limit と観測方法を確定する。`UNRESOLVED` は PASS ではなく Baseline v1 blocker である。

| ID | signal / unit | Candidate criterion | 根拠と現在の状態 |
|---|---|---|---|
| H-01 | final command 全 numeric field | NaN/Inf が 0 件 | fail-safe invariant。閾値は確定、runtime evidence 未取得 |
| H-02 | final steering angle / rad | authoritative final angle limit 内 | raw config は 32 deg、現行 final 理論値は 48 deg。車両側 final limit が **UNRESOLVED** |
| H-03 | final steering angle の実差分 / rad/s | authoritative slew limit 内 | 現行 nominal intent は 1.2 rad/s、message の rotation-rate 2.0 とは別。hard limit は **UNRESOLVED** |
| H-04 | final longitudinal acceleration / m/s² | resolved normal limit（candidate D1〜4: `[-1.35, 1.0]`）内 | current config。fault/Recovery の別規則と runtime evidence は未確定 |
| H-05 | stale odometry から safe source への遷移 / s | candidate `odom_timeout_sec + 1 observed control interval` 以内、以後 normal source なし | current timeout 0.5 s。観測点と scheduling allowance は **UNRESOLVED** |
| H-06 | collision / penalty | required safety scenario で 0 | result penalty/log/condition を分離。gate の local proxy だけでは公式 PASS にしない。oracle mapping は **UNRESOLVED** |
| H-07 | fault/forced-stop/Recovery 中の Boost rising edge / count | 0 | current safety requirement。deterministic/live evidence 未取得 |
| H-08 | Recovery disabled 時の gear/reverse action / count | 0 | canonical config。runtime/pure-rule evidence 未取得 |

H-02/H-03 は「現行値と同じ」だけでは安全 PASS にしない。根拠文書、車両/AWSIM interface、または承認済み工学上限を先に確定する。H-06 は公式 result を得られない local gate では proxy 判定として残し、公式安全性を断定しない。

## 8. Runtime baseline matrix

| ID | 実行 | 主目的 | 最低限保存する evidence |
|---|---|---|---|
| B-01 | 対象 package unit test | pure rule と既存 regression | test result、環境、binary hash |
| B-02 | `make dev ROS_DOMAIN_ID=1` | 単車・canonical path・40 Hz | graph、resolved config、hz、cycle trace、log |
| B-03 | `make gate1 ROS_DOMAIN_ID=1` | `awsim_safety_gate_1` local regression | log、command/state trace、run ID |
| B-04 | `make gate2 ROS_DOMAIN_ID=1` | `awsim_safety_gate_2` local regression | 同上 |
| B-05 | `make gate3 ROS_DOMAIN_ID=1` | `awsim_safety_gate_3` local regression | 同上 |
| B-06 | `make dev4` | Domain 1..4、V2X、Domain/fallback config | 各 Domain graph/config/trace、V2X completeness |
| B-07 | fault replay | stale/non-finite/solver failure/stop | normalized cycle fixture と期待結果 |
| B-08 | submit -> eval image -> `make eval` | 対象 commit の sealed evaluation route | tar/image/binary hash、result JSON、log、run ID |
| B-09 | noncanonical trajectory-topic component test | optional path update の互換性 | 一時 config、valid/invalid update event、採用 path hash |

gate 番号と 2026 公式評価項目の対応は未確定であり、ここではローカル回帰 scenario として使う。現行 gate 起動経路は AWSIM に gate ID を渡すが、baseline に必要な rosbag や result JSON の生成を保証していない。このため `awsim.log` / `autoware.log` だけに依存せず、Phase 0 専用 recorder を併用する。

B-08 は次の順で、同じ提出 tar と image/binary identity を結び付ける。

```text
./create_submit_file.bash
  -> tar 最上位と tar SHA-256 を確認
  -> ./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz
  -> eval image ID/digest と install binary SHA-256 を確認
  -> make eval
```

B-09 は canonical config の `reference_path.update_by_topic=false` を変更しない。一時 config/test launch で valid update 採用、invalid update 拒否、最後の valid path 保持を確認する component compatibility test とし、production runtime 等価性は主張しない。

Scenario status の許可範囲:

- B-01、B-02、B-07、B-08、B-09 は `PASS` 必須
- B-03〜B-06 は `PASS`、または hard-safety/external-contract に関係しない機能性能だけの承認済み `KNOWN_RED`
- infrastructure 不足、起動失敗、artifact 不完全は `N/A` ではなく `BLOCKED` または `INVALID`
- `N/A` は Recovery/gear disabled の canonical-runtime cell など、この文書で optional と明示した欄だけに許可

### 8.1 共通 run protocol

各 live scenario は実行前に次を固定し、3 回とも同じ区間を比較する。

1. `make down` と process/topic 残留確認
2. `docker compose config`、Domain、image、config/resource hash の保存
3. recorder 起動、QoS 適用、必須 topic の ready barrier
4. `/set_initial_pose` 成功、期待 node 生存、odometry source stamp の進行/freshness、reference path 取得、sole command publisher、期待 Domain/vehicle 数、startup fatal なしを valid-run barrier にする
5. `/awsim/state=Start`、最初の対象 event、または明示 waypoint を観測開始 anchor にする
6. 固定 unwrapped path distance、終了 event、または scenario 定義済み timeout を終了条件にする
7. 最大 timeout で必ず停止し、recorder を flush して graceful stop する
8. 必須 topic/message count、artifact hash、log completeness を検査する
9. `make down` と残留確認を行う

開始/終了 anchor、timeout、必要 message 数は scenario manifest に数値で固定してから取得する。detached 起動の終了を人手判断にしない。valid-run barrier を満たさない run は `INVALID` とし、3 回反復の母数に数えない。

各 runtime scenario では、少なくとも次を別々に判定する。

- hard safety: NaN/Inf、collision、publisher 重複、危険時の Boost、limit 違反
- compatibility: topic/type/QoS、config/path hash、状態/event 順、result/schema
- behavior envelope: path progress に対する横偏差、command、最接近、fallback 回数
- timing envelope: sidecar steady-clock の output inter-arrival。timer start/end、deadline miss、OSQP iteration/solve time は observation seam がある場合だけ別指標として保存
- known RED: 現行で既に失敗する項目。golden により正当化せず、修正判断まで明示的に残す

## 9. 取得方法

### 9.1 Host-side identity

Compose の resolved config は host 側で取得する。`dev4` は実 run と同じ `LOG_DIR` / `SIM_MODE` / `ROS_DOMAIN_ID`、project 名を使って各 service を別々に保存する。次は `RUN_LOG_DIR` を実際の run path に置き換える例である。

```bash
RUN_LOG_DIR=/output/actual-run-id
LOG_DIR="${RUN_LOG_DIR}" SIM_MODE=dev4 ROS_DOMAIN_ID=0 docker compose config
LOG_DIR="${RUN_LOG_DIR}" ROS_DOMAIN_ID=1 docker compose -p 1 config
LOG_DIR="${RUN_LOG_DIR}" ROS_DOMAIN_ID=2 docker compose -p 2 config
LOG_DIR="${RUN_LOG_DIR}" ROS_DOMAIN_ID=3 docker compose -p 3 config
LOG_DIR="${RUN_LOG_DIR}" ROS_DOMAIN_ID=4 docker compose -p 4 config
docker inspect "$(docker compose ps -q simulator)"
docker inspect "$(docker compose -p 1 ps -q autoware)"
```

Domain 2〜4 の Autoware container も同様に inspect し、実際の image、environment、mount、command を pre-launch config と照合する。

### 9.2 In-container ROS evidence

起動中の各 Domain コンテナ内で、少なくとも次を取得する。

```bash
ros2 node info /mpc_controller
ros2 topic list -t
ros2 param dump /mpc_controller
ros2 topic info -v /control/command/control_cmd
ros2 topic info -v /localization/kinematic_state
ros2 topic info -v /planning/scenario_planning/trajectory
ros2 topic info -v /v2x/vehicle_positions
ros2 topic info -v /awsim/status
ros2 topic info -v /awsim/cmd
ros2 service list -t
timeout 15 ros2 topic hz /control/command/control_cmd
timeout 15 ros2 topic hz /localization/kinematic_state
timeout 15 ros2 topic hz /planning/scenario_planning/trajectory
```

`ros2 param dump` は dynamic ROS parameter layer の evidence として保存する。全 YAML の effective config は別 parser で再実装せず、現行 C++ config resolver と同じコード経路を呼ぶ test-only `effective-config.json` 出力で保存する。

環境 identity と実行物も保存する。

```bash
ros2 pkg prefix multi_purpose_mpc_ros
sha256sum "$(ros2 pkg prefix multi_purpose_mpc_ros)/lib/multi_purpose_mpc_ros/mpc_controller_cpp"
```

container environment の `ROS_DOMAIN_ID`、`VEHICLE_ID`、`SIM_MODE` は未設定を有効な状態として `<unset>` 付きで記録する。environment 値と effective launch 値を混同せず、`sim_mode` は launch log と AWSIM command line、Domain は resolved Compose config/container inspect/launch log から確定する。

現行 Autoware Compose では `VEHICLE_ID` は通常 unset である。`reference.launch.xml` の `vehicle_id` arg は既定 `default` に解決されるが MPC へ転送・使用されていないため、これを実効 self vehicle identity とみなさない。environment と launch arg をそれぞれ観測事実として保存し、V2X message 内の vehicle ID と混同しない。

Docker image ID/digest、Compose project、compiler、OSQP version、`output/latest` の解決先も run manifest に加える。

### 9.3 Phase 0 専用 recorder

既定 recorder は、V2X、AWSIM state/status、raw command、reference など characterization に必要な観測をすべて含まない。一方、全 topic 記録は過大なため、Phase 0 の期間限定 recorder で次を記録する。

recorder は subscriber 作成と QoS 適用が完了した ready signal を出してから AWSIM を Start する。`/mpc/ref_path` の transient-local を含め、topic ごとの publisher QoS と互換な override を設定する。

入力:

- `/clock`
- `/localization/kinematic_state`
- `/localization/acceleration`
- `/v2x/vehicle_positions`
- `/awsim/state`
- `/awsim/status`
- `/aichallenge/pitstop/condition`

出力:

- `/control/command/control_cmd`
- `/control/command/control_cmd_raw`
- `/mpc/ref_path`
- `/mpc/prediction`
- `/awsim/cmd`

個別試験のときだけ trajectory、stop request、gear status/command を追加する。MCAP と raw log はコミットせず、そこから次の小型成果物を生成する。

- environment/input hash を持つ `manifest.json`
- 連続重複を除いた状態遷移と意味イベントの `events.json`
- rate、偏差、command、最接近距離などの `metrics.json`
- path progress または event 基準で間引いた小型 trace
- 数ケースに限定した QP characterization fixture

終了時に必須 topic ごとの message count、最初/最後の source stamp、QoS、bag metadata を検査する。`/aichallenge/pitstop/condition` は local proxy の一つであり、単独で公式 collision oracle としない。log、result penalty、condition を別 field として記録し、gate で result がない場合は local proxy と明記する。

## 10. 比較規則

比較対象を混ぜず、3 種類の oracle に分ける。

### 10.1 Contract oracle

endpoint、type、direction、publisher ownership、launch entry、Domain、tar/result schema を正本文書と exact 比較する。QoS は `docs/interface/` に明示された値だけを contract oracle とし、それ以外は current compatibility oracle で比較する。runtime が正本と違う場合は external-contract RED であり、現状値を golden にしない。

### 10.2 Deterministic cycle fixture

固定 input/state に対して次を比較する。

- enum、flag、phase、pass side、target ID、accepted solver status は exact
- horizon/QP の dimensions と正規化 sparse index pattern は exact
- reference、QP numeric values、solver solution、raw/final command、prediction は field 別 absolute/relative tolerance
- 現行の自由文字列 reason は全文 exact にせず、安定 prefix/category と抽出した numeric field に正規化

将来 enum/reason code を導入する場合、旧文字列 category との対応表を intentional internal API change としてレビューする。

### 10.3 Live-run oracle

live AWSIM では cycle ごとの target ID、状態遷移時刻、solver status を exact にしない。hard safety invariant、許可された状態遷移、semantic event count、aggregate envelope を比較する。

同じ baseline 条件を同一 image/host で最低 3 回取得し、まず run 間差を測る。連続量の許容値は、工学的な最小許容差、solver epsilon 由来値、message/保存形式の quantization floor、反復差から得た envelope の最大値を基礎に決める。中央値/MAD が 0 でも無条件の zero tolerance にせず、根拠を fixture metadata に残す。判定は baseline envelope と hard safety limit の両方を満たす必要がある。

走行 trace は wall-clock の完全一致で比較せず、unwrapped path progress、waypoint ID、または意味イベントからの相対時刻で整列する。timestamp、PID、絶対 path、ANSI、logger の行順、DDS discovery 順は正規化する。solver の solve time や iteration 数は observation seam から得られる場合だけ記録し、環境差を調べず exact match にしない。

`/control/command/control_cmd_raw` と最終 `/control/command/control_cmd` は同一ではない。現行は最終 steering に gain が適用されるため、raw の steering limit をそのまま final の合格条件として流用せず、両段の期待値と最終安全 invariant を別々に定義する。

### 10.4 Intentional delta

既存 interface contract への適合修正または外部契約を変えない安全修正は、`Baseline candidate v0` との差分を保存し、修正 commit と test を紐づけて `Baseline v1` を作る。外部契約そのものの変更は本計画外とする。以後の構造リファクタは v1 との等価性を判定する。

## 11. 生成物の配置

- コミットする: manifest、小さな normalized fixture、比較器、test、判断記録
- コミットしない: rosbag/MCAP、raw graph dump、param dump、launch log、build/install、Docker artifact
- 既存 `output/<run-id>` / `output/latest` は評価成果物の読み取り元としてのみ使い、独自 recorder の file/link を追加しない
- raw evidence の一時出力: `/tmp/aichallenge-mpc-baseline/<run-id>/`
- Baseline v1 承認時は normalized fixture だけで再判定可能にする。raw evidence を根拠として残す場合は content hash、保存期限、承認済み外部 artifact URI を manifest に記録し、ephemeral path だけを参照しない

## 12. Freeze checklist

- [x] commit、branch、canonical launch route を記録した
- [x] core/config/launch/resource の主要 hash を記録した
- [x] Domain 1〜4 の主要静的設定と fallback source を記録した
- [x] contract oracle の endpoint/type/direction/owner を記録した
- [x] 既知差分を初期登録した
- [ ] Docker image、binary、compiler、OSQP version を記録した
- [ ] 同じ C++ resolver から Domain 1〜4 の `effective-config.json` を取得した
- [ ] Domain 0 と Domain 1〜4 の runtime graph/type/QoS/owner を記録した
- [ ] 契約済み 5 control method の include 経路を静的確認した
- [ ] runtime matrix B-01〜B-09 を実行した
- [ ] B-01/B-02/B-07/B-08/B-09 は PASS、B-03〜B-06 は許可 status と completeness evidence を持つ
- [ ] normalized fixture と比較器を作成した
- [ ] 最低 3 回の反復から tolerance を確定した
- [ ] D-01〜D-11 を分類した
- [ ] D-04 の二つの control-mode topic の責務判断を完了した
- [ ] 必要な Phase 0b 修正を独立して完了した
- [ ] external-contract RED と hard-safety RED が 0 件である
- [ ] H-01〜H-08 に FAIL/UNRESOLVED がない
- [ ] `LegacyReplayHarness` の必須 fixture が 2 回以上同じ正規化 output を返す
- [ ] observation seam を含む最終 clean commit から black-box/cycle baseline を再取得した
- [ ] 最終 commit、source/config/resource、comparison schema、installed binary、dev/eval image、submit tar の identity を再取得した
- [ ] Baseline v1 をレビュー・承認した
