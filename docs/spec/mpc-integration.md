# multi_purpose_mpc_ros インテグレーション設計

> 仕様ドキュメント（現仕様の正）。最終確認: 2026-08-18。文書運用方針は [docs/README.md](../README.md) を参照。

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
| `/awsim/control_mode_request_topic` | Bool | SIMのAUTONOMOUS engage要求。Readyで即時送信し、Start後は発進確認まで有限再送 |

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
- MPC問題生成では現在最近傍の `tracking_wp_id` を状態変換、モデル線形化、経路制約、V2X、予測軌跡に使う。`wp_id_offset` 適用後の `preview_wp_id` は入力参照候補にだけ使い、future側が低速なら早めに減速し、同方向で大曲率なら早めに切り増す。カーブ出口での早期加速・早期切り戻しには使わない。これによりヘアピンで状態座標系と参照座標系を一致させたまま、offset `0..2` の先読みを行う。
- `waypoint_local_association_enabled=true` の場合、`tracking_wp_id` は前回IDを中心とする経路距離窓から選ぶ。候補scoreには位置距離、車両yawと経路headingの差、後退量、現在速度と制御周期から到達しにくい前方jumpを含める。初期化、経路更新、または局所候補までの距離が `waypoint_local_lost_distance_m` を超えた場合だけ全経路探索へ戻す。circular pathでは周回端をwrapし、ヘアピンで空間的に近い別枝へIDが飛ぶことを抑える。現行の距離窓8 m後方／30 m前方、lost 4 mは2025 AWSIM向け暫定値である。
- OSQPは `SOLVED` / `SOLVED_INACCURATE`、解の有限性、入力を含む全制約違反を確認し、直線の steering 0 を異常扱いしない。
- callback間の未保護な同時更新を避ける初期安全策として、C++ node は `SingleThreadedExecutor` で実行する。
- odometry の受信時刻と、非ゼロsource stampが最後に変化した時刻をsteady clockで監視する。既定 `odom_timeout_sec: 0.5` を超えた場合、boost を無効にして速度0・負加速度・rate limit付き操舵復帰を直接publishする。pose、速度、solver出力、gain適用後commandの NaN / Inf も同じ fail-safe 経路で拒否する。
- `min_linearization_speed_mps: 0.5` 未満では `1/v`、`1/v^2` を含む時間状態の線形化を停止する。両値はローカル設定であり、2026公式値ではない。
- Python `path_constraints_provider` も circular CSV の重複終点を同じ許容差で除去し、C++側は受信したrows/colsが内部ReferencePathと一致しない制約を拒否する。
- solver fallback と `/control/mpc/stop_request` はlegacy boost arbitrationより優先し、boostを必ず無効化する。low-pass gainは `[0,1]` に限定し、filter後にも加速度、操舵角、操舵変化量を制限する。

### Contouring-progress MPCC 第1段階（2026-08-17、2025由来の暫定）

追い越しの`ShiftOut`、`Pass`、`Return`では、従来の空間領域
`[e_y, e_psi, t]`モデルから、時間領域Frenet
`[e_y, e_psi, s]`モデルへ切り替えられる。入力次元は従来どおり
`[v, kappa]`であり、QPの3状態・2入力の固定疎構造、Persistent OSQP、
primal/dual shift warm-startを維持する。

第3状態`s`は速度入力とFrenet kinematic modelで結合され、stage progressの
lag cost、前進reward、asymmetric trust regionを持つ。既存Frenet DPの
stage corridorは`e_y`のhard boundとしてそのまま使う。これによりDPが
左右topologyを決め、MPCCが回廊内の横位置、姿勢、速度、進捗を同時に解く。

legacyとprogress modeでは第3状態の意味が異なるため、mode遷移時はOSQP
workspaceとwarm-startを一度resetする。設定
`progress_contouring_mpcc_enabled`で従来MPCへ戻せる。

循環経路の内部horizonに有限な0 m stageが現れた場合は、progress用コピーだけを
`minimum_reference_speed_mps * minimum_stage_dt_sec`へ正規化する。負値や
非有限値は修復しない。progress reference、trust region、全stage線形化、costは
一度`ProgressContouringMpcPreparation`へ構築し、全項目が成立した場合だけQPへ
適用する。不成立周期は制御全体のdeceleration fallbackへ送らず、legacy MPCへ
縮退する。また、周回境界でcourse progress originがwrapした場合は、同じprogress
mode中でもOSQP warm-startをresetする。

追い越しhorizonのstatic-wall検証は、横位置`d(s)`だけでなくその勾配が作る
Frenet path headingも車体矩形のyawへ反映する。横位置の平行移動はbase pathの
法線を維持し、車体yawだけを`d(s)`のheading offsetで補正する。stage 0は現在横位置
から最初のtarget、以降は直前stageからのbackward differenceを使い、実際の
`target_epsi`生成と同じ規約にする。wall側で不成立な場合は単一stageだけを動かさず、
全profileをcurrent-side holdまたは滑らかなbase-line復帰へ同じ比率で収縮し、各候補を
static mapと横加速度で再検証する。全速度・hard wall reserve候補を試した後も新解が
成立しない一周期では、同じhard wall / target boundで再検証済みのlast-feasibleまたは
baseline profileだけを保持し、即座にRecoveryへ落として速度を失うことを避ける。
solved MPCC trajectoryのauthority判定も現在位置から始まる同じbackward heading規約を
使うため、計画時には通過可能だった軌道が実行時だけ`solution hard wall contact`となる
不整合を避ける。周期debugの`profile_keep`は採用profileの元経路保持率で、1.0が無補正、
0.0が安全側fallbackまでの収縮を示す。
occupied / unknown / out-of-map、物理footprint、設定したhard wall marginは緩和しない。

これは本格MPCCへの段階実装であり、最大3回のRTI-SQP枠は持つが、通常設定は
first feasible QPと条件付き1回refinementである。terminal velocity cost、
dynamic bicycle / tire modelは未導入である。2026公式制御仕様ではなく、
2025 AWSIM由来のシミュレーション競技向け暫定実装として扱う。

### MPCC runtime budget Stage 1（2026-08-18、2025由来の暫定）

`control_rate=40 Hz`のcommand生成、現在車体のwall/contact確認、EmergencyBrake、
odometry/NaN fail-safeは維持したまま、追い越し中の重複計算だけを間引く。

- receding-horizon lateral evaluationは、同一reference waypoint、同一target/Mission
  generation/phase/side、同一goalと制約、target位置変化が縦0.25 m・横0.15 m以内、
  continuity lease成立、hard faultなしの場合だけ、最大
  `v2x_overtake_receding_horizon_refresh_interval_sec`再利用できる。WP更新、接触、
  predicted overlap、target jump、course-progress reject、corridor block、禁止WP、
  EmergencyBrake、wall fault、solver recoveryでは即時fresh評価へ戻る。
- RTI-SQPはfirst QP solutionを必ず保持し、lateral bound reserve、linearizationからの
  横位置/姿勢差、horizon最大曲率のいずれかが閾値を超え、かつfirst solve後が
  `progress_contouring_refinement_start_deadline_ms`未満の場合だけ2回目を開始する。
  2回目の失敗時は従来どおりfirst feasible solutionを使う。
- static wall envelope cacheはheadingを0.025 rad、追加clearanceを保守的な1 cm上向き
  bucketへ量子化する。bucket内heading差はfootprint marginへ加え、物理安全余裕を
  緩和しない。capacityは16384件、退避はLRUとし、周回中に再利用するentryを保持する。

周期ログ`Overtake horizon schedule`、`MPCC RTI-SQP`、既存wall cache telemetryで、
fresh/reuse、condition/deadline skip、cache hit率を実走比較する。core solverの別thread化、
command 40 Hzとsolver 20 Hzの完全分離は本Stageには含めない。

### MPCC execution authority hardening（2026-08-19、2025由来の暫定）

Frenet DPが生成した追い越しprefixをMPC/MPCCへ渡す前に、計画の採用と実行を
別の権限として扱う。`v2x_overtake_mpcc_frenet_dp_last_path_max_age_sec`はoptimizer
sourceの絶対実行期限であり、実行周期のwall/target再検証で期限を延長しない。
`runtime_validation_lease_sec`は、sourceが期限内である間のtarget予測揺れだけを橋渡しする。

実行権限は、同一target/side、target continuity、現在・予測車体分離、wall、Emergency、
禁止waypoint、残りprefix距離に加え、次をすべて満たす場合だけ有効とする。

- 実測`e_y`とactive prefixの差が
  `v2x_overtake_mpcc_frenet_dp_max_tracking_lateral_error_m`以下。
- 実測`e_psi`とprefix勾配・course曲率から求めたheadingの差が
  `v2x_overtake_mpcc_frenet_dp_max_tracking_heading_error_rad`以下。

権限が有効でも、DPまたは直近のphysically validated solved trajectoryの各stage横目標は、
採用済みMission profileから`v2x_overtake_mpcc_lite_same_side_max_lateral_adjustment`以内へ
制限する。これを越えるtopology変更は実行時上書きとして通さず、新しいMissionとして
再採用する必要がある。信頼幅を適用した後も従来のwall、target、横加速度horizon検証を
最終hard guardとして維持する。

source期限切れ、追従乖離、または信頼幅入力不正時はDP実行権限を解放する。
その周期も同じcanonical MPCC formulationのfresh/retained証明だけを採用し、証明が
なければtyped Emergencyとする。legacy Mission profileやlegacy 3-state solverへの
formulation handoffは行わない。診断は`DP execution authority retained/released`の
`tracking`と、`DP execution`の`authority`、`trust_adjusted`、`age`で確認する。

この処理は2025 AWSIM競技シミュレーションで観測したMPC→MPCC切替時の逸走対策であり、
2026公式仕様ではない。

### Unified Race MPCC foundation（2026-08-21、2025由来の暫定）

通常MPCと追い越しMPCCの接続不整合を減らし、将来の常時Race MPCCへ段階移行するため、
horizonの物理座標を`StageGeometry`へ一本化する。stage 0は現在tracking waypointから最初の
予測state waypointへの遷移であり、dynamics、進捗距離、実行trajectory、wall検証、physical
certificateは同じtransition/cumulative distanceを使用する。consumerごとに一stage先を再計算
してはならない。

非同期の左右extended-MPCC評価は、snapshotごとにOSQP workspaceを作り直さず、Left/Right別の
persistent solver contextを共有する。context keyはasync epoch、target ID、side、horizon sizeで、
key変更またはcourse-progress discontinuity時だけcold resetする。周期ログの`warm`、`reset`で
cold-start反復を判別する。

physical execution certificateにはwall合格trajectoryだけでなく、生成元targetのID、source/receipt
時刻、course progress/lateral、V2X observation generationを保持する。async採用およびEntry commit時に
現観測との差を再検証し、同一IDでも別時刻・別コース枝・許容外の横移動ならfail-closedとする。

診断にはLeft / Right / Hold / Returnを同一schemaで表す`Race MPCC shadow`を追加する。この段階で
solver接続済みなのはLeft/Rightだけで、Hold/Returnは未評価理由を明示する。`authority=shadow`の間は
現行MPC/OvertakeLine/DynamicEscapeの実行権限を変更しない。shadowの動的検証後にHold/Returnを同一
定式化へ接続し、最後にMPC↔MPCCの切替廃止を別ステアリングで判断する。

5-state QPは横位置・姿勢・速度・進捗という異なる単位を同じ制約行列に含む。OSQPのglobal
infinity-normだけを安全判定へ流用してはならない。solver adapterは各rowの違反量と
`eps_abs + eps_rel * row_scale`に基づく許容値を保持し、consumerは安全意味ごとに対象rowを検証する。
Track/Cruise shadowではstage 1..Nの横位置box rowをメートル単位で独立判定し、予測pose抽出時は
観測された違反量ではなく許容値だけを境界toleranceに使う。大きなcourse progress値が横制約の
許容誤差を広げたり、違反量自身が抽出許容を広げるself-relaxing contractは禁止する。

2026-08-23以降、Overtake/DynamicEscapeのlive extended solver、左右tactical branch、
Follow canonical solverは`RowToleranceNormalized`で各rowを物理許容差へ正規化してから解き、
同じrow単位で成功判定する。横境界を後段証明する一方で、global scaleでは成功扱いする
二重契約を残さない。Track/Cruise productionへ同policyを一括適用するとdynamics rowの
不成立とEmergency Stopが増えることをreplayで確認したため、Track/Cruiseは別の
failure-first sliceで定式化・scalingを監査するまで従来policyを維持する。

2026-08-24のfailure-first監査により、5-state QPはTrack/Cruiseを含む全contextで同じ
数値契約へ統一した。各変数は有限box boundから導いた物理scaleで座標変換し、各制約rowは
上下限のうち厳しい側の物理許容差をOSQPのabsolute toleranceへ写像する。物理relative
toleranceはrow scaleへ一度だけ埋め込み、`RowToleranceNormalized`内部ではglobalな
`eps_rel`を0として二重適用を禁止する。返却後は元の物理`A/l/u`ですべてのrowを再証明し、
primal/dual warm startも同一scaleで往復変換する。これは車両制約値の緩和ではなく、solverと
certificateが異なる許容差を使っていた契約不良の修正である。

2026-08-25以降、常時MPCC移行用のsix-state Track/Cruise shadowは、solver requestと同時に
現在pose、course-frame window、immutable wall grid、車体footprintを封印する。既存の
latest-only solver workerがsolve、exact trajectory adapter、endpoint/swept-footprint wall proofを
直列実行し、solver artifactとwall certificateを同じsequence/decision/intent/stage geometryへ
結び付ける。wall certificate mailboxはworld identityまで一致しないcompletionと、新しいsubmissionに
追い越されたcompletionを拒否する。別の40 Hz wall workerはCPU scheduling contentionを増やしたため
採用せず、control callbackはsnapshot構築、non-blocking submit/consume、shadow telemetryだけを担う。
この証明は引き続き`authority=shadow, selected=0`であり、単独ではproduction commandへ昇格しない。

2026-08-22のshadow A/Bでは、exact headingによるpost-solve physical certificateをhard oracleとして
維持した。post-solve再solve、reference-heading固定の横box、単一勾配による横位置・姿勢結合rowは、
いずれも40 Hz超過またはQP不成立を増やし、非線形の向き付き車体footprintを保守的に証明できなかった
ため削除した。これらをfeature flagとして残さない。非線形footprintをQP内部へ移す場合は、別sliceで
保守的包絡と計算時間のfailure-first証拠を用意する。

### Wall / Contact Stuck Recovery（Implementation Complete / dev3 Enabled）

前進専用の現行MPCは、正面が壁に押し付けられると後退できない。
この復帰は通常走行MPC / MPCCの評価関数とは分離し、次のpure C++ coreと
`mpc_controller_cpp`内のROS adapterとして実装した。

- `stuck_recovery_core`: 前進意図、signed speed、pose / path進捗、補助証拠、
  意図的停止の除外、Recovery FSM、gear timeout、距離・時間・速度・試行上限を扱う。
- `recovery_footprint`: 向き付き車体矩形、swept interpolation、map外 / unknownの
  occupied扱い、wall方向分類、初期接触を増やさず離脱するReverse Straight / Left / Rightと
  Forward Straight / Left / Right rolloutを扱う。
- `recovery_mpc`: 符号付き距離のFrenet bicycle modelでForward / Reverseを予測する
  離散・有限ホライズンplanner。2026-07-19のdev3 A/Bで改善しなかったため既定無効。

最終command arbitrationは既存C++ nodeの単一thread内で行う。Recoveryが制御権を
持つ周期はNormal MPC commandを破棄し、Recovery / SafeStopのどちらか一方だけを
既存 `/control/command/control_cmd` publisherからpublishする。別のcontrol publisherや
Domain 0のreset / teleportは追加しない。gear実制御時だけ
`/control/command/gear_cmd`をpublishし、`/vehicle/status/gear_status`の
freshな一致reportを駆動前に必須とする。

Forward escapeから`LowSpeedRejoin -> Normal`を完了した場合に限り、simulationでは
新しいStuck Recoveryの再アームを時間・コース前進距離の小さい方まで抑制できる。
これは通常のFollow / SafetyBrakeや制御出力を無効化するものではない。ガード開始後の
新規collision、現在のwall evidence、MPC solver fallbackを検出した場合は即時解除し、
Reverse経由のrejoinではアームしない。現行既定値は最大3.0秒または前進3.0 mである。

stepwise Forward復帰が`collision_worsening`または`forward_duration_limit`で失敗した場合は、
その事実を後続の`StopAndReassess` / `SafeStop`とは別のtrackerへ保持する。同一aggressive retry
cycle内の複数stepは1失敗として数え、`aggressive_forward_retry_limit_before_reverse`へ達した
次cycleは既存のswept-footprint / V2X安全判定を維持したままReverse候補だけを評価する。
Reverse maneuver終了、Forward escape成功、Recovery終了、または新規episode開始で連続数をresetする。
最後のFSM reasonや再評価中のcandidate directionから過去のForward失敗を逆算しない。

実行modeは次の順に段階化している。

1. `enabled: false`: Recovery coreをcontrol cycleで評価せず、通常MPCを維持する。
2. `enabled: true`, `shadow_mode: true`: detectorと安全判定のログだけを出し、
   control / gear commandを変更しない。
3. `shadow_mode: false`: SIMに限ってFSMがcommand ownerになり得るが、後述の
   reverse actuationラッチが閉じている間はgear駆動しない。現在のdev3ではP1 / P2 / P3を
   このmodeとし、未列挙Domainと実車は無効である。Follow / SafetyBrakeなど前方車に対する
   意図的停止は検出対象外なので、停止列は前方が空いた先頭車から解除する。

```yaml
stuck_recovery:
  enabled: true
  domain_enabled:
    1: true
    2: true
    3: true
  shadow_mode: false
  simulation_only: true
  reverse_actuation_enabled: true
  reverse_acceleration_sign: 1.0
  reverse_stop_acceleration_mps2: -0.8
  verified_reverse_stop_deceleration_mps2: 0.4
  reverse_control_latency_sec: 0.2
  boost_status_timeout_sec: 0.5
  detector:
    solver_fallback_recovery_enabled: true
    solver_fallback_duration_sec: 2.0
    solver_evidence_free_recovery_enabled: true
    solver_evidence_free_duration_sec: 3.0
    evidence_free_recovery_enabled: true
    evidence_free_duration_sec: 1.5
    coordinated_stop_recovery_enabled: true
    coordinated_stop_duration_sec: 0.25
    coordinated_stop_front_speed_mps: 0.28
    max_observation_gap_sec: 0.2
    stopped_speed_mps: 0.28
    moving_speed_mps: 0.35
    forward_intent_speed_mps: 1.0
    forward_intent_acceleration_mps2: 0.1
    stationary_duration_sec: 0.25
    max_pose_displacement_m: 0.15
    max_progress_delta_m: 0.20
    awsim_recovery_settle_sec: 0.30
  gear:
    report_timeout_sec: 0.5
    stop_speed_mps: 0.28
    stop_confirm_sec: 0.10
    command_resend_interval_sec: 0.2
    max_command_requests: 1
  maneuver:
    clearance_wait_timeout_sec: 1.0
    clearance_safe_stop_recovery_enabled: true
    safe_stop_clear_confirm_sec: 0.5
    aggressive_sim_recovery_enabled: true
    aggressive_retry_delay_sec: 0.5
    race_relaxed_fault_retry_enabled: true
    fault_retry_clear_confirm_sec: 0.5
    fault_retry_max_observation_gap_sec: 0.2
    tracked_v2x_completeness_enabled: true
    max_reverse_distance_m: 3.0
    max_reverse_duration_sec: 4.0
    max_reverse_speed_mps: 0.8
    reverse_acceleration_magnitude_mps2: 0.5
    max_forward_distance_m: 0.6
    max_forward_duration_sec: 2.0
    max_forward_speed_mps: 1.0
    forward_acceleration_magnitude_mps2: 1.0
    reverse_escape_distance_m: 2.0
    forward_escape_distance_m: 0.30
    max_reverse_pose_step_m: 0.05
    reverse_steering_angle_rad: 0.25
    side_escape_steering_samples: 5
    solver_reverse_only_heading_error_rad: 1.0
    wall_direction_search_margin_m: 0.50
    wall_direction_ambiguity_m: 0.02
    side_escape_enabled: true
    escape_step_distance_m: 0.40
    max_escape_steps: 10
    side_escape_min_contact_reduction_ratio: 0.05
    max_attempts: 3
  footprint:
    front_extent_m: 1.49
    rear_extent_m: 0.51
    left_extent_m: 0.725
    right_extent_m: 0.725
    margin_m: 0.05
    sweep_interpolation_step_m: 0.05
  rear_safety:
    # 正常なV2X messageからrace session単位で車両IDを自動学習する。
    self_filter_mode: excluded
    self_vehicle_id: ""
    vehicle_radius_m: 1.45
    prediction_margin_sec: 0.1
  recovery_mpc:
    enabled: false
    horizon_steps: 10
    travel_step_m: 0.20
    steering_sample_count: 9
    beam_width: 48
    maximum_steering_tire_angle_rad: 0.35
    maximum_steering_change_rad: 0.12
    lateral_error_weight: 6.0
    heading_error_weight: 12.0
    steering_weight: 0.15
    steering_change_weight: 1.0
    terminal_lateral_error_weight: 30.0
    terminal_heading_error_weight: 40.0
  rejoin:
    speed_limit_mps: 1.0
    feedback_steering_enabled: true
    lateral_error_gain_rad_per_m: 0.60
    heading_error_gain: 1.20
    max_steering_tire_angle_rad: 0.35
    static_lookahead_m: 0.8
    aggressive_force_after_retries: 0
    retry_on_blocked_path: true
    retry_on_timeout: true
    max_lateral_error_m: 0.5
    max_heading_error_rad: 0.35
    confirm_sec: 0.3
    timeout_sec: 5.0
    solver_recovery_timeout_sec: 1.0
    cooldown_sec: 1.0
    forward_rearm_guard_enabled: true
    forward_rearm_guard_duration_sec: 3.0
    forward_rearm_guard_distance_m: 3.0
```

`race_relaxed_fault_retry_enabled` は `simulation_only: true` の場合だけ使用できる。Recovery fault後も
Drive gear、finite odometry/command、boost inactive、V2X completeness、短距離の静的map/V2X回廊が
`fault_retry_clear_confirm_sec` 連続して正常なら、fault latchを解除して通常のstuck detectorから再評価する。
一条件でも欠けるか観測間隔が上限を超えた場合は確認時間を0へ戻す。`tracked_v2x_completeness_enabled` は
単一メッセージに全車が揃わない瞬間でも、vehicle IDごとのfreshで有限な最新sampleが期待台数ぶん揃えば
completenessを成立させる。実車相当では従来どおり自動解除しない。

`reverse_actuation_enabled` と `reverse_acceleration_sign` は別々のhard latchである。
sign 0や駆動・停止commandの同符号、停止減速度0は起動時に拒否する。2026-07-12の
ローカルAWSIM校正ではREVERSE中の正加速度を後退駆動、負加速度を停止としている。
運営`main`のteleopに合わせた逆符号A/Bは`output/20260718-225500`で後退量が小さく、
後続runでも全車`rejoin_complete`へ到達しなかったため不採用とした。現dev3は成功実績のある
`+0.5 m/s^2`を駆動、`-0.8 m/s^2`を停止とする。gear report遅延、停止距離予約用の
`0.4 m/s^2`と`0.2 s`は従来の暫定計測値を維持する。

通常のsolver fallbackはRecoveryから除外する。例外はfallbackが連続2.0秒以上で、
solverとは独立したpath前進要求、低実速度、pose / path無進捗、現在の
footprint-to-wall証拠が全て継続した場合だけである。collision hint単独では成立しない。
detector更新間隔が0.2秒を超えた場合は停止時間とfallback時間をresetし、callback / odometry
途絶時間を連続観測へ加算しない。

2025 AWSIM向け暫定設定では、wall証拠がないsolver fallbackも、同じ前進要求、低実速度、
pose / path無進捗が3.0秒継続した場合だけ`solver_evidence_free_qualified`とする。solver failure由来の
episodeは、wall証拠なしまたは`abs(e_psi) >= solver_reverse_only_heading_error_rad`の場合だけ
Reverse-onlyとする。wall証拠があり姿勢誤差が閾値未満ならwall方向が選んだ短距離候補を
static / V2X gateで検証する。いずれもswept footprintが安全でなければギアを要求しない。
Follow / SafetyBrakeで前方停止車が0.28 m/s以下のまま0.25秒停止した後続車は
`coordinated_stop_qualified`となり、同じReverse-only gateで最後尾から後方空間を作る。
SIMレースの停止entryは1 km/h相当の0.28 m/s、release hysteresisは0.35 m/sとする。

solver正常時には、物理壁とoccupancy map / legacy collision通知の不一致へ限定対応する。
Follow / SafetyBrake / LowSpeedAvoidance等の意図的停止ではなく、前進要求、低実速度、pose / path
無進捗が`evidence_free_duration_sec`継続した場合だけ、wall evidenceなしでもConfirmedとする。
現設定は有効、1.5秒であり、solver fallbackへは適用しない。前進intentはMPC解とreference path
速度要求の最大値を使い、停止中のtarget再構築による0 / 非0の交互変化でtimerをresetしない。
証拠なしConfirmedでは、現在map footprintと後方3.0 m ReverseStraight rolloutがclearで、
fresh / completeなV2X corridorもclearの場合だけ候補を生成し、実測2.0 m後退後に停止する。
map invalid、out-of-map、unknown、solver fallback、V2X不完全ではこのfallbackを使用しない。
Recoveryがcommand ownerになった後はfallback継続だけで途中abortしない。通常設定の
LowSpeedRejoinではsolver復帰を必須とし、solver fallback自体を起点に全hard gateを通過した
episodeだけ資格をcoreへラッチする。`aggressive_sim_recovery_enabled=true`のdev3では、
LowSpeedRejoinが専用の低速速度・操舵feedbackを直接出力することを利用し、episode起点に関係なく
solver healthを再合流条件から外す。この例外は`simulation_only: true`との組み合わせでしか起動できない。

通常V2X behaviorの`deliberate_stop`はRecovery開始前の誤検知除外に限定する。Followは実front
vehicleに対する`follow_speed_limit`、moving-front clearance cap、またはpath前進要求を下回る有限の
target/desired velocityが実際に有効な場合だけ該当する。state holdにより`Follow`表示が残っても、
`limit=inf`かつcap inactiveなら意図的停止とはせず、solver fallback/no-progressの確認時間を継続する。
side vehicleの存在だけでも意図的停止としない。Recovery開始後は
一時的なFollow / SafetyBrakeをimplicitな`control_interrupted`へ変換せず、選択方向のstatic / V2X
corridorを駆動可否の正本とする。control disableとadapterが明示するRecovery hard stopは
従来どおりSafeStopを優先する。

近傍wall cellを車体座標へ変換し、Front / Rear / Left / Right / Mixedへ分類する。
FrontではReverse Straight / Left / Rightをこの順で評価し、RearではForwardStraightを
評価する。Side / Mixedかつ実map contactありではReverse / ForwardのStraightに加え、Left / Rightを
0.05、0.10、0.15、0.20、0.25 radで0.40 m評価する。同じ`RequireImprovement`条件でcontact減少最大を
選び、Reverse / Forwardが同値ならReverse、同方向内ではStraight、Leftの小角、Rightの小角の
決定順を使う。solver起因のreverse-only episodeでは、積極シミュレーション設定でもForwardを
生成しない。coordinated-stopは初回Reverse intentだけを所有し、その候補が明示的に不成立となった
次のbounded retryではForward候補を解禁する。前進要求中に0.28 m/s以下となり、前方停止車、前方車を伴う
recent collision、またはRear以外のwall/contactがある場合もReverseを最初に評価する。この
Reverse-first候補が静的非実行可能、接触悪化、距離・step上限等で明示的にSafeStopへ到達した場合だけ、
次のaggressive retryでForward候補を解禁する。RequireImprovement候補がない接触状態では、SIMの
aggressive recoveryに限り、新規接触と接触セル増加を許さない`AllowNonWorsening`の0.40 m候補まで
探索する。実車相当とsolver reverse-onlyでは従来のfail-closedを維持する。
検索marginは方向推定専用でありcollision footprintを縮小しない。候補は`SUSPECT_STUCK`、
AWSIM補正待機、停止確認、clearance待機では毎周期再評価する。ただしAWSIM補正待機を抜けた後の
Reverseという方向は別latchで保持し、clearance確認中にForwardへ変更しない。具体的なprimitiveと
操舵値は`SHIFT_TO_REVERSE` / `WAIT_REVERSE_REPORT` / `REVERSE_MANEUVER` / `FORWARD_MANEUVER`へ
到達した時点でepisodeへ固定する。固定後は方向と操舵符号を変更しない。Reverse Left / Rightは
選択したsigned steering angleをepisodeへ固定して実commandへ渡し、
選択rolloutの横変位分だけV2X corridorを拡張する。

AWSIM補正待機中にpose / contactが変わるため、`STOP_AND_CONFIRM`後は待機前の候補を破棄し、
現在snapshotをbaselineとして候補を再選択する。待機時間は2025 AWSIM dev3向けに0.30秒、
停止確認は0.10秒とする。現在map footprintがclearならwall分類にかかわらずstatic swept rolloutを
評価する。fresh / completeなV2XでReverse corridorが後続車に塞がれた場合は停止して同じReverse方向の
clearanceを待つ。非協調episodeでReverse復帰自体が明示的に失敗した後だけ、Forward Straight /
Left / Rightのうちstatic rolloutとforward corridorがclearな候補を評価し、終端の絶対heading errorを
最も減らす候補へ最大0.6 mのForwardCreepを切り替える。同値ではStraightを優先し、選択したLeft /
Rightの操舵符号を実commandへ渡す。前進で距離が増え続ける後方車は
forward corridorの新規衝突対象から外す一方、前方、横並び、予測中に前方へ入る車両はrejectする。
候補をepisodeへ固定した後は方向を途中変更しない。

Recovery episode開始時のwall / collision / map contact証拠を保持する。証拠ありepisodeでは
AWSIM補正後のfootprint clearを復帰成功に使用できる。evidence-free episodeは開始時からmap上clear
なので、pose変化だけでは成功とせず、reference path進捗が`max_progress_delta_m`以上の場合だけ
`awsim_recovery_resolved`とする。横方向nudgeだけの場合はSTOP_AND_CONFIRMへ進む。

V2X trackerのposition jump許容距離は
`max(v2x_position_jump_threshold, v2x_v_max_safety * message_dt)`とする。これにより約1 Hz配信で
正常走行車が固定距離閾値以上進んでもcomplete判定を維持する。非有限・逆行timestamp、許容速度を
超える移動、position jump、台数・ID・frame・covariance不正は引き続きRecoveryをfail-closedとする。
受信側clockとmessage source clockはepochが異なり得るため両者を直接減算しない。受信freshnessは
受信側clock、source stampの単調性とsample ageはsource clock内でそれぞれ検証する。これにより
sim timeのsource stampをwall-clock受信時刻から引いて全messageをstale扱いする誤判定を避ける。

実制御候補は次のhard conditionをすべて満たす場合だけ実行する。

- static map上の全swept footprintが安全。Reverseの初期接触は前方wall、ForwardStraightは
  後方wallに限定する。これは非stepwise候補の方向別条件であり、Side / Mixedのstepwise接触離脱は
  前後方向とも次の`RequireImprovement`条件を適用する。
  現在cellは初期patchの固定1-cell halo内かつ直前patchと同一または8近傍の明示Occupiedだけを
  許し、接触数増加、chain migration、一度clear後の再接触、unknown、離れたpatchをrejectする。
  候補終端までに接触を解消し、rolloutと実後退中監視は共通helperを使用する。swept stepと
  runtime corner motionはmap resolution以下の設定stepに制限する。初期接触を一動作で完全解消する
  `RequireClear`のLeft / Rightは、向きが変わる場合のpenetration単調性が未実装のためfail-closedとする。
  Side / Mixedの段階離脱は前後方向とも`RequireImprovement`として各swept sampleのcontact非増加と
  局所連続を検証する。
- `/v2x/vehicle_positions`の最近messageがfreshで、position jumpのない他車の
  現在位置から選択maneuver duration分を予測した進行方向corridorがclear。車両台数は設定せず、
  race session中の正常messageから車両ID集合を自動学習する。Recovery開始後のlatest message、
  またはcurrent Recovery epoch内のfreshな追跡集合が全学習済みIDを含む場合だけcompleteとする。
  学習済みIDが0件、ID欠落、空ID、重複ID、source stamp / map frame / covariance異常ではrear
  informationをunknownとしてReverseを阻止する。自車の扱いは`self_filter_mode=excluded`または
  正確な`vehicle_id`を明示する。
- `/awsim/status`がfreshで`isBoosting=false`。Boost中またはstatus不明時は
  rear information incompleteとする。
- freshなREVERSE GearReportを確認してから駆動加速度を出す。
- Rear-wall離脱はfreshなDRIVE GearReportを確認し、最大0.6 m / 1.5 s / 0.8 m/sの
  ForwardCreepだけを出す。
- Side / Mixed候補は前後方向とも各swept sampleでcontact数がステップ初期値を超えず、previous patchと
  局所連続し、終端で5%以上減る場合だけacceptする。contact減少最大、Straight、Left、Rightの
  決定順で選ぶ。方向間の同値はReverseを優先する。実移動後にもcontact減少を確認し、0.40 mごとに
  停止・再評価する。
  episode距離は各stepをまたいで保持し、実測2.0 mまたは最大10ステップ、実改善なし、Unknownで
  SafeStopとする。V2X不完全時は即停止してReverseを維持し、情報が回復すれば同じステップを
  再開する。completeな情報でstaticまたは他車blockが継続した場合だけDriveへ戻す。
  gear要求後は`AllowNonWorsening`へ切り替え、残距離ごとの追加5%改善は要求せず、contact非増加と
  新規contactなしを監視して0.40 m終端まで進める。終点の実測改善判定は維持する。
  step中にcontact悪化または単step時間上限へ達した場合も即停止し、Driveでstatic / V2X候補を
  再評価する。1回の境界変化だけでepisodeを破棄しない一方、累積距離・最大10ステップ・改善確認の
  上限は維持する。
- signed speedの絶対値が`max_reverse_speed_mps`へ達した周期はReverseを維持して減速し、
  上限未満へ戻れば同じmaneuverを再開する。速度上限だけでescape完了やDrive復帰にしない。

通常設定では`clearance_wait_timeout_sec`到達によるSafeStopだけをrecoverableとし、static rollout、
freshでcompleteなV2X、後方corridor clearが`safe_stop_clear_confirm_sec`連続した後に再び
CHECK_CLEARANCEへ戻る。積極シミュレーション復旧ではmotion/solver/gear report/rejoin/attempt limitの
終端も`aggressive_retry_delay_sec`停止後にbudgetと候補をリセットして再評価する。奇数retryでは、
静的・V2X rolloutがclearなら前進fallbackを優先し、同じ効かなかった後退の反復を避ける。
さらに`rejoin.aggressive_force_after_retries`回以上失敗し、現在footprintが非接触かつfeedback steeringが
有効なら、短いrejoin sweepのwall交差だけをsimulation-onlyで緩和してLowSpeedRejoinへ進む。
現在footprintの接触、非有限値、gear/odometry/controlのハード異常は緩和しない。0はこの分岐を無効にする。
invalid/non-finite input、non-monotonic time、odometry unsafe、control interruptedはlatchedのままとする。

REVERSE GearReportとV2X completenessが隣接周期で到着する場合は、
`WAIT_REVERSE_REPORT`で停止commandを維持し、completeかつclearになった後だけReverseCreepへ
入る。ReverseManeuver中の一時欠落にも同じ停止待機を適用し、回復後は累積移動距離とcontact基準を
維持して再開する。情報不足の周期には駆動せず、情報欠落だけではDRIVEやLowSpeedRejoinへ
遷移しない。completeなstatic / vehicle blockageが継続した場合だけ停止後DRIVEへ戻す。
stepwise maneuverではDrive確認後にstep数とepisode距離を保持して`STOP_AND_REASSESS`へ戻し、
候補を選び直す。非stepwise Reverseのduration limitも残attemptがあれば再評価し、上限消費後だけ
SafeStopする。

stepwise Reverse完了後にDrive reportを待つ間は、次の`STOP_AND_REASSESS`またはSafeStopが
予約されているためnormal MPC solverを使用しない。したがってsolver fallback継続だけで
再判定を中断しない。通常設定では非solver起因のLowSpeedRejoin後のsolver fallbackをDriveで停止保持し、
最大`solver_recovery_timeout_sec`だけ再初期化を待つ。積極シミュレーション復旧ではこの待機を行わず、
通常MPC solverに依存しないfeedback commandを継続する。

`WAIT_FOR_CLEAR`でstatic rolloutとV2X corridorが同時にclearになった場合は、その同一snapshotで
`CHECK_CLEARANCE`を消費してgear要求へ進む。clear確認後にもう1周期待つことでV2X completenessや
contact candidateを失い、駆動前にSafeStopへ落ちる競合を避ける。

AWSIM標準wall recoveryによるpose / yaw変化でdetectorの観測windowがresetされても、現在footprintに
map contactが残る場合は`awsim_recovery_resolved`としない。待機時間終了後に
`STOP_AND_CONFIRM`へ進める。現在footprintがclearでも、横偏差またはheading誤差がrejoin許容外なら
通常制御へ直帰しない。`WAIT_AWSIM_RECOVERY`中の位置変化がdetectorのpose閾値を超えるか、yaw変化が
rejoin heading閾値を超えた場合は外部pose handoffとして扱い、現在poseでwaypointをglobal再対応する。
待機解除時には補正前の観測anchor、maneuver距離、contact baseline、選択済みprimitive、direction latch、
Forward失敗履歴を破棄し、現在snapshotから方向を再評価する。これによりAWSIM補正で車体が反転した場合に、
補正前の通常rejoinまたは同一方向retryを継続しない。footprint clearかつ横・heading誤差がともに許容内の
場合だけ`awsim_recovery_resolved`として通常制御へ戻す。

Recovery開始後はそのrace sessionのStart Boostを再発動せず、LowSpeedRejoin前に
MPC prediction / control history / solver fallback、V2X behavior、OvertakeLine、pass-side / target
lockをresetする。再合流へ入る前にFront / Sideはepisode実測2.0 m、Rearは実測0.30 mの
escapeと車体clearanceを必須とする。未達でDriveへ戻った場合は`escape_not_confirmed`で
SafeStopする。再合流中もV2X completeを必須とし、欠落時は停止保持する。通常MPCが接触後に
0 m/sを返す場合も、全hard gateが成立しているLowSpeedRejoinだけは設定値を専用の前進目標にする。
参照曲率feedforwardと横偏差・heading誤差feedbackからrate-limit後のtire angleを求め、同じ値で
0.8 mの前進swept footprintと実commandを評価する。lateral / heading errorが所定時間閾値内に
入るまで継続し、速度設定値0以下は起動時に拒否する。

ForwardManeuver中のstatic / V2X hazardは駆動を即時停止するが、終端SafeStopにはせず
`STOP_AND_REASSESS`へ戻す。clearanceが残れば`WAIT_FOR_CLEAR`と既存のclear確認時間を使う。
map footprintがclearでも近傍wallがLeft / Right / Mixedなら0.40 m stepwise候補を使い、
単発4秒の非stepwise後退で2.0 m未達となる状態を避ける。

stepwise Forwardでは、現在footprintがclearな区間だけの妥当性確認済みodometry移動を
`STOP_AND_REASSESS` / `CHECK_CLEARANCE`を跨いで累積し、1回のduration limit直前で失った
数cmを次のmaneuverでやり直さない。clearへ移る境界周期の距離は加算せず、接触再発、Reverse、
odometry motion guard不成立、Recovery終了で累積値を0へ戻す。Forward escape確認では現在
maneuver距離とこのclear累積距離の大きい方を使う。Reverseの距離確認、swept-footprint、V2X、
step / attempt上限は従来どおりであり、予測停止距離を物理移動として加算しない。

`retry_on_timeout=true`では、LowSpeedRejoin timeout時にattemptまたはescape-step budgetが残る場合、
停止してescape確認ラッチとepisode距離をresetし、新しいstatic / V2X候補を選択する。以前の2.0 mを
次のescape完了条件へ流用しない。retry無効、budget消費済み、gear / solver / odometry / collision
worseningでは従来どおりSafeStopする。

step / attempt上限は1 Recovery episodeの予算である。`rejoin_complete`の最終ログには完了時の
カウンタを残し、cooldown後に新しいSuspected / Confirmedが独立したepisodeを開始する時点で0へ戻す。
SafeStop中の上限を自動解除するものではない。

現runtimeはFrontの`ReverseStraight` / `ReverseLeft` / `ReverseRight`、Rearの`ForwardStraight`、
Side / Mixed接触の前後両方向のStraight / Left / Right、後続車にReverseを塞がれた
clear-footprint時の`ForwardStraight` / `ForwardLeft` / `ForwardRight`を決定的に評価し、
actuation開始時のfeasible候補をepisode中固定して実commandへ変換する。RViz candidate表示と
終端rejoin scoreは未実装である。

2026-07-17時点で`stuck_recovery_core`、`v2x_overtake_core`、`recovery_footprint`の対象suiteは
すべて成功し、`make autoware-build`も成功した。package全体testには、本修正外の既存trajectory
fixtureとローカルのstale packageに起因する失敗が残るため、対象3 suiteを正本の回帰確認とした。
AWSIM単車のgear遷移、Reverse加速度符号、signed odometryは校正済みである。

`make dev3` run `output/20260717-093647`では、共通進捗の前方検出とmoving-front速度上限が
動作し、前走車への連続追突から3台が停止列になる現象は再発しなかった。P1とP3は走行を継続した。
P2は別途深いwall接触へ入り、stepwise recoveryがmap contactを
`29 -> 26 -> 12 -> 11 -> 4 -> 3 -> 0`へ4ステップ・累積1.219 mで解消した。その後の0.40 m候補は
約1.039 m先の新規collisionを予測し、Reverse / Forwardともsafeなrolloutを得られなかったため
`maneuver_direction_unknown`でSafeStopした。これは前方車デッドロックではなくfail-closed動作で、
深いwall侵入を起こさない通常制御の安定化と、そこからの追加離脱primitiveは別の残課題である。

最終binaryの`output/20260717-094634`でも追突による3台停止列は再発しなかった。P1はWP 323まで
走行を継続し、P2は壁へ入ったP3を共通進捗で検出してSafetyBrakeし、その後再発進してWP 318まで
到達した。clear-footprint時の候補ログでは`ForwardLeft / steering=+0.250 rad`が選択され、
選択primitiveの操舵符号が保持されることも確認した。この候補は確認中に車両が通常前進へ戻ったため、
Recovery actuation自体は行っていない。P3はWP 121で深いwall侵入後にsafe rolloutを失って
SafeStopした。またP2の周回端では、MPC内部のcircular smoothing用閉路点を共通進捗が再wrapして
ゼロ長segmentを例外にする不具合が判明した。共通進捗helperを、ゼロ長閉路点・連続重複点を進捗0で
skipするよう修正し、duplicate circular endpointとinterior duplicateの回帰testを追加した。

その修正後の`output/20260717-095229`ではWP 193まで`degenerate segment`は再発しなかったが、
周回端到達前にP3が深くwallへ入り、stepwise recoveryを5 step・累積1.083 m実行後もescapeを
確認できずSafeStopした。P2は前進fallbackを8 step再評価したが累積0.049 mしか進めずstep上限で
SafeStopし、P1は前方SafetyBrakeを維持した。車両同士の追突は起きていないものの3台は停止しており、
「安全な離脱rolloutがない先頭車」まで必ず再発進させる修正ではない。残課題は通常MPCのwall侵入抑制、
side-only近接時のdeadlock分類、および追加primitiveの安全設計であり、現行gateはfail-closedを維持する。

LowSpeedAvoidance stall監視とactuation直前の候補固定を追加した`output/20260717-225927`では、
対象unit test 120件と25 package buildは成功したが、dev3受け入れは不成立だった。このrunでは
LowSpeedAvoidance自体が発生せず、D1はWP 118付近のwall contactを5 step・累積2.015 mの退避で
解消した後、LowSpeedRejoin中の新規contactにより`rejoin_unsafe`となった。D3はWP 183付近で
約1.71 radのheading errorを持って停止し、安全なForward static fallbackが成立せず、Reverseも
D2に塞がれて`clearance_wait_timed_out`となった。D2は共通コース進捗でD3を検出してSafetyBrakeした。
したがって3台停止は再発しており、次の対象はRecovery gate緩和ではなく、通常走行中のwall接触と
LowSpeedRejoin新規接触の予防である。実験詳細は
`.steering/20260717-v2x-low-speed-recovery-deadlock-fix/results.md`に記録する。

その後の壁接触予防・再合流安定化では、SafetyBrakeを発火させた前方危険targetを1.0秒保持し、
短時間のV2X front/side分類欠落だけでCruiseへ戻らないようにした。新しい危険観測で期限を延長し、
targetが4.0 m以上明確にrearへ抜けた場合または期限到達で解除する。静止スタートグリッドを保持しないよう、
start-grid phaseが`WaitingForStart`または`Prepared`の間はholdをarmせず既存状態もclearする。

LowSpeedRejoinはnormal MPC先頭操舵による0.8 mの前進swept footprintを駆動前と走行中に確認する。
現在footprintがclearで前進rolloutだけblockedの場合は、`retry_on_blocked_path=true`なら停止確認へ戻り、
bounded recovery候補を再評価する。現在footprintにcontactがある場合は従来どおりSafeStopする。
また、current footprint clear時のReverse候補はstatic safeな候補から終端heading errorが最小のものを選ぶ。
これらは2025 AWSIM向け暫定値であり、2026公式値ではない。

`output/20260717-232948`では、危険target消失直後のCruise復帰は再発せず、D1のLowSpeedRejoinも
前方wall collisionを検出して新規contact前に停止した。D2は旧runの停止時刻約62秒を超えて走行した。
一方、D1には安全な前進rolloutがなく、D3がD1の後退corridorを塞いだためD1がSafeStopし、D3と
後から到達したD2が順にSafetyBrakeしてD2のStartから約79秒後に3台停止した。これはhazard dropoutによる追突ではなく、
壁際先頭車と後続車による閉塞である。次段はWP 72 / WP 123付近の通常MPC wall departure抑制を優先し、
必要な場合のみV2X completeness・rear-clear・static sweep・距離/時間上限を備えたcooperative yieldを
別ステアリングで設計する。実験詳細は
`.steering/20260717-wall-contact-prevention-and-rejoin-stability/results.md`に記録する。

`steering_tire_angle_gain_var=1.5`は2025 AWSIM向けの出力補償として扱う。QPの曲率制約、
前回操舵列からの予測曲率、操舵レート制約、低速直接操舵、遅延補償用BicycleModelはgain適用前の
desired equivalent tire angleを使用し、legacy / Recoveryではpublish時だけAWSIM commandへgainを
適用する。5-state canonical Track/Cruiseは物理証明と同一のcommand角をpublishし、このlegacy補償を
重ねない。
`output/20260723-190900`でgainを内部モデルにも適用したところ、通常走行中の
`measured_kappa / predicted_kappa`中央値はD1/D2/D3で0.53/0.58/0.54だった。一方、
gain適用前のraw角による予測との比は0.81/0.88/0.83で、D1は横偏差約2.7 mから
WP124〜126で1330周期連続solver failureとなった。このため内部モデルへのgain適用は
AWSIMでは回帰と判断し、出力補償の責務へ戻した。

`output/20260717-234612`ではD3がWP72、D1がWP123をStart後のwall contactおよび連続OSQP
failureなしで通過し、それぞれ136.583 s、142.589 sで1周した。旧runのD2 Start後約79秒での
3台停止も再発せず、通常MPCの壁逸脱予防はPassとする。一方D2は2周目WP34〜41のOvertakeLine
ShiftOut / Recovery中に、wall contactを伴わない別の連続solver failureで停止した。これは追い越し
復帰中のre-entry guardとして別ステアリングで扱う。gain値自体は2025 AWSIM向け暫定値であり、
2026公式値および実車値ではない。raw予測曲率とAWSIM実測`yaw_rate / speed`の一致は
引き続きdev3走行で確認する。過去のgain=1.0走行詳細は
`.steering/20260717-normal-mpc-wall-departure-prevention/results.md`に記録する。

協調Reverseとsolver failure Reverse-onlyを有効にした`output/20260718-011435`では、D1が
8 step・累積2.059 m後退してescapeを確認し、D2も0.148 m後退した時点で後方車を検出して停止した。
D3はWP282付近の連続solver failureからReverse-only Recoveryへ入ったが、現在footprintに16 contactが
あり、全Reverse候補が0.05 m先からcontactを悪化させるため`maneuver_direction_unknown`でSafeStopした。
D3はReverse gearを要求しておらず、static gateのfail-closed動作はPassとする。一方、D1 / D2も
その後solver unsafeとなって全車停止したため、デッドロック解消はFailである。wall証拠なしsolver
failure経路はunit testで確認したが、このrunのD3ではwall証拠も成立しており単独発火は未確認である。
詳細は`.steering/20260718-v2x-coordinated-reverse-recovery/results.md`に記録する。

適応contact操舵とrejoin solver graceを追加した3回の`make dev3`では、8 step版のD2が
1.820 mで停止する再現後、10 step版が2.182 mまで後退して`rejoin_complete`し、WP161から
WP199まで前進した。最終binaryの`output/20260718-164220`では、D3がSide / Mixed contactから
`-0.15`、`-0.20`、`-0.25 rad`を実際に選択してcontactを154から99へ低減し、D2も6 step・
2.006 mでLowSpeedRejoinへ入った。D1はstatic collision、D2は`rejoin_timed_out`、D3は
10 stepでescape未達となり、3台全停止は未解消である。危険候補の強制実行はなく、安全gateは
維持された。途中runで判明した完了episodeのstep予算持ち越しは、新しいepisode開始時にresetし、
2 episode連続unit testで確認した。対象testは合計93/93、25 package buildは成功した。
solver graceはunit testで確認したが、dev3のLowSpeedRejoin中solverは正常でruntime未観測である。
詳細は`.steering/20260718-adaptive-contact-escape-and-rejoin-solver-grace/results.md`に記録する。

stepwise clearance再評価、LowSpeedRejoin専用速度・操舵feedbackを追加した
`output/20260718-173846`では、D2が9 step・2.049 mの後退でescapeを確認し、
LowSpeedRejoinへ入った。専用目標1.0 m/sに対して実速度は0.000から0.879 m/sへ上昇し、
`e_y`は0.787から0.336 m、`e_psi`は-0.070から-0.043 radへ収束した。
開始から約3.1秒で`rejoin_complete`し、その後少なくとも約38秒は通常走行を継続した。
全rejoin sampleでstatic rejectは`none`、V2X corridorはclearで、最大操舵は-0.35 radだった。
別runの深い接触では10 step・0.091 mで`escape_step_limit_reached`となり、危険な前進は行わず
SafeStopした。対象core testは70/70、25 package buildは成功した。実験詳細は
`.steering/20260718-rejoin-feedback-and-stepwise-clearance-reassessment/results.md`に記録する。

`output/20260718-174716`では、D3のwall contact後にD1、D2がSafetyBrakeで停止し、全車が
復帰`step=0/10`、移動0.000 mのまま`rear_vehicle_blocked`からSafeStopへ入った。共通進捗では
D2が最後尾だったが、従来V2X矩形は自車前端1.49 m、margin 0.05 m、他車半径1.45 mを重ねるため、
前方約2.99 m以内のD1も後退障害物として扱っていた。

停止列では静的mapが選択した実rolloutの全姿勢について、向き付きfootprintと膨張他車円の
signed clearanceを評価する。初期から保守円が重なる場合は、全sampleで重なりが悪化せず、
rollout終端で有意に改善する場合だけ分離操作として許可する。前方車から離れる最後尾は進めるが、
後方車へ近づく車両、新規重なりを生む旋回、改善しない操作は引き続きblockする。選択済みrolloutが
ある場合は他車速度によらず、他車の速度予測を含むrollout signed clearanceと単調分離条件を使う。
このため0.2 m/s境界で停止車用判定と粗いmoving corridorが切り替わる不連続を作らない。
V2X不完全・不正速度・position jumpもfail closedのままである。再現D2の右旋回0.40 mを含む
`recovery_footprint` 31件とpackage buildは成功した。`make dev3` run
`output/20260718-181438`では、最後尾D2が9 step、合計2.055 m後退し、LowSpeedRejoinから
`rejoin_complete`へ到達した。その後D1もcoordinated recoveryで後退を開始したため、
`output/20260718-174716`の全車step 0停止は解消した。D1の旋回候補は停止D3とのclearanceが
`-1.442 m`から`-1.457 m`へ悪化するため拒否され、clearなStraight候補へ切り替わっており、
安全側の拒否も維持できている。

ただしレース全体は、D3の静的map候補が`contact_worsened`になる問題と、復帰後D2の通常MPCが
コース外へ逸脱する別問題により継続しなかった。D1も移動中D2を従来moving corridorで検出し、
0.058 m後退後にSafeStopした。したがって本修正の受け入れ範囲は停止列V2Xデッドロックの解除までで、
壁接触からの候補生成と通常MPC逸脱は別修正とする。詳細は
`.steering/20260718-v2x-reverse-rollout-clearance/results.md`に記録する。

`output/20260718-182347`では、停止車rollout判定自体は機能し、D2が2.045 mと2.140 mの
stepwise後退を2回完了した。一方、D1はForwardManeuver中に移動D2を検出して
`forward_hazard_appeared`、D3は`e_y=0.975 m` / `e_psi=-0.500 rad`で`rejoin_timed_out`、
D2の3回目はmap clear / Left wallなのに非stepwiseとなり、0.544 mでduration limit後に
`escape_not_confirmed`へ入った。これら3つの終端遷移に上記の停止・再評価を適用した。
対象testは`stuck_recovery_core` 77/77、`recovery_footprint` 34/34、25 package buildが成功した。
実走結果は`.steering/20260718-recoverable-recovery-terminal-transitions/results.md`に記録する。

追加修正後の`output/20260718-184701`ではD2がstepwise離脱から`rejoin_complete`へ到達したが、
D3はRear wallでもsolver evidence-free設定だけでreverse-onlyになり、安全なForward候補を生成できなかった。
wall-aware reverse-only判定後の`output/20260718-185619`ではD3が旧停止点WP251を通過し、D2も復帰した。
一方D1はsolver起因episodeの移動でdetector資格を失い、LowSpeedRejoinが同じsolver復旧を待つ循環へ入った。
資格ラッチ後の`output/20260718-190601`ではこの循環は解消したが、深い接触後にD3の全Reverse候補が
contactを悪化させ、D1 / D2もstep上限へ到達した。このためSide / Mixed接触のForward候補を同じ
`RequireImprovement`条件で比較する最終修正を追加した。

最終`make dev3`の`output/20260718-191547`では、race開始後約3分間、D1 / D2 / D3がすべて走行を
継続した。終了直前の速度はD1 3.87 m/s、D2 3.00 m/s、D3 2.63 m/sで、`SAFE_STOP`、
`escape_step_limit`、`maneuver_direction_unknown`、`rejoin_timed_out`はいずれも0件だった。
このrunではcontact Recoveryが発火しなかったため、双方向contact候補がForwardを選ぶ実走証拠は
未取得である。対象test 77/77 + 34/34と25 package buildは成功しており、選択分岐の実走確認は
次に同じ深いMixed接触が再現したrunで継続する。

`output/20260718-222730`では双方向contact候補が実走で初めてForwardを選び、D3のcontactを
30から0へ減らして0.823 m移動した。一方、非solver起因episodeだったためLowSpeedRejoin直後に
`solver_unsafe`で停止した。D1は後方hazard待機中にcontactが0から72へ急増し、10 stepで0.067 mしか
動けず上限停止、D2もReverse 7 stepで0.165 mしか動かなかった。このrunを根拠に、dev3を実車安全の
先行試験ではなく非協力的な2台とのシミュレーション競争試験として分離し、
`aggressive_sim_recovery_enabled`、solver-independent rejoin、回復可能終端の反復、alternate forward、
運営main準拠Reverse符号を導入した。純粋coreでは通常fail-closedを既定値として維持し、追加testを含む
package全431 testと25 package buildが成功した。実走結果は
`.steering/20260718-dev3-noncooperative-race-recovery/results.md`へ記録する。

`output/20260718-225500`では、D2のReverseが0.7秒で0.164 m動き、負のReverse駆動符号を確認した。
D1 / D2 / D3はSafeStopを34 / 27 / 31回再試行して永久ラッチを解除したが、最大横偏差が
5.153 / 3.727 / 5.132 mへ拡大し、`rejoin_complete`は全車0回だった。これを受けて
`rejoin.aggressive_force_after_retries: 3`を追加した`output/20260718-230630`では、D3の横偏差が
2.824 mから0.582 mへ縮小し、D2もrejoin中に約1.0 m/sで経路を横断した。一方、D2はheading error
2.169 radのまま反対側のwallへ達し、2回目も`rejoin_complete`は全車0回だった。したがって本設定は
永久停止解除と経路方向への移動には有効だが、競技復帰の完成条件は満たさない。次の正式修正は
接触前の衝突回避と、大姿勢誤差を扱う複数点rejoin plannerであり、retry回数の追加ではない。

この候補として`recovery_mpc`を追加した。符号付き移動距離のFrenet bicycle modelを10 step x
0.20 m予測し、横偏差、姿勢差、操舵量、操舵変化の離散beam searchから第一tire angleだけを使う。
LowSpeedRejoinでは毎周期再計画し、escapeではstatic / V2X gateを通る既存primitiveの順位付けに
のみ使う。計算不能時は従来P feedback / heading選択へfallbackする。`output/20260719-121902`では
D1がMPC rejoinへ入ったものの他車と物理的に噛んで実速度0.001〜0.003 m/sのままtimeoutし、D2は
深いcontact、D3はmap外でSafeStopした。全車`rejoin_complete=0`のため既定は
`recovery_mpc.enabled: false`とし、元のbounded stepwise Recoveryを正式設定とする。pure core 6 testと
25 package buildは成功している。詳細は`.steering/20260719-bounded-recovery-mpc/results.md`に記録する。

gear publisherはReliable / KeepLast(1) / Volatileであり、TransientLocalは古いREVERSEの
late-join replayを避けるため使用しない。

`output/20260721-184659`の3車停止では、D1が`maneuver_direction_unknown`、D2が
`clearance_wait_timed_out`、D3が`escape_not_confirmed`（実移動1.947 m / 目標2.0 m）で
それぞれSafeStopへ入り、`aggressive_sim_recovery_enabled=false`のため永久停止した。
dev3の競争シミュレーションに限定して同設定を有効化し、回復可能なSafeStopは再試行する。ただし
`clearance_wait_timed_out`は、同じV2X閉塞が残る間はSafeStopのまま待ち、clearanceが安定確認された後だけ
再評価する。escape距離には停止reserveと離散計測を吸収する0.10 m許容を設けるが、現在footprintの
clear確認は省略しない。さらにcollision hint、前方車、前進intent、停止速度が同時に成立した場合だけ、
通常のFollow / SafetyBrake由来`deliberate_stop`をRecovery検出から除外しない。このoverrideは
`simulation_only: true`かつ明示設定時だけ有効で、通常のno-progress時間、static/V2X clearance、
距離・時間・試行回数の上限は維持する。これらは2025 AWSIM dev3向け暫定修正であり、実車設定や
2026公式仕様ではない。

運営チャットの回答は、技術的実装を可としつつ、低速・短時間・後方clearな
スタック復帰に限定し、戦略的な後退を避けるよう案内している。本実装の0.8 m、2.0 s、
0.8 m/s、1 attemptはこの運用方針に沿うローカル値であり、2026公式上限ではない。

### AWSIM 2026 Start Dash Boost（2026-07-16）

2026公式Boostは通常の`AckermannControlCommand`加速度とは独立したAWSIM item commandとして扱う。

- `awsim_boost.enabled: true`、`mode: start_once`でシミュレーション時だけ有効化する。
- `trigger: first_forward_motion`では、`/awsim/state=Ready`で発車監視を準備し、正常な制御指令をpublishしたcycleで符号付き前進速度が`motion_speed_threshold_mps`以上になった瞬間を発動基準にする。`Start`は`Ready`欠落時の監視準備fallbackであり、発動基準にはしない。
- 自動制御有効、正常odometry、solver非fallback、V2X SafetyBrake・Reverse/Recovery・fail-safeのいずれでもないこと、freshな7要素`/awsim/status`、`boostRemaining >= 1`、`isBoosting < 0.5`をすべて満たした最初の正常control cycleで発動する。
- 初回前進検出から`motion_trigger_timeout_sec`以内かつ`max_trigger_speed_mps`以下の場合だけ発動する。安全制約が解除されないまま窓を過ぎた場合は、そのセッションでは発動・再送しない。
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
  trigger: first_forward_motion
  motion_speed_threshold_mps: 0.1
  max_trigger_speed_mps: 1.0
  motion_trigger_timeout_sec: 0.5
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

実装本体は `multi_purpose_mpc_ros/tools/kaleidoscope/kaleidoscope/` に切り出している。ROS nodeではなくPython/Tkアプリであり、現行の `ros2 run multi_purpose_mpc_ros trajectory_editor` と旧Python importは互換層で維持する。既定CSV・mapの探索はROS package shareまたは現リポジトリ配置を使用し、ホスト固有の絶対pathには依存しない。

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

Editorが保存する `psi_rad/kappa_radpm/vx_mps/ax_mps2` はoffline CSV metadataである。
C++ MPCはstrict loaderで7列を検証した後、XYを内部補間してheading／curvatureを再生成し、
起動時に`v[i+1]^2 <= v[i]^2 + 2*a_max*ds`と対応する後退制動pass、横加速度上限から
runtime速度profileを一意に計算する。Domain・追い越し・ACC等の動的速度上限はprofileを
一定値で置換せずcapとして適用する。

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

自己位置の計測時刻補正は MPC の waypoint offset ではなく、`aichallenge_submit_launch/launch/reference.launch.xml` から EKF に渡す。さらにSIMでは、EKF出力をMPC初期状態へ変換する直前に、2025 Pure Pursuit由来のCTRV予測を適用して制御計算・アクチュエータ遅延を補う。EKFの `0.3 s` とMPCの `0.125 s` はいずれも2025由来の暫定調整値であり、2026 AWSIMで実測した確定値ではない。

| launch 引数 | 現在の値 | 確認事項 |
|---------|---------|---------|
| `simulation_pose_additional_delay` | `0.3` | SIM の pose measurement 追加遅延 [s]。同条件の `0.0` との A/B 比較で要調整 |
| `vehicle_pose_additional_delay` | `0.0` | 実車の pose measurement 追加遅延 [s]。センサ時刻遅延を実測するまで補正しない |

| 設定項目 | 現在の値 | 確認事項 |
|---------|---------|---------|
| `map.yaml_path` | `env/final_ver3/occupancy_grid_map.yaml` | 占有格子地図が存在するか |
| `reference_path.csv_path` | `env/final_ver3/traj_mincurv.csv` | 最適化済み経路が存在するか |
| `reference_path.domain_csv_path` | Domain 1..3 別 CSV | `ROS_DOMAIN_ID` ごとの trajectory 上書き。未設定 Domain は `csv_path` を使う |
| `reference_path.update_by_topic` | `false` | CSV 直接読み込みモード（推奨） |
| `awsim_boost.enabled` | `true` | SIMで2026公式Boostを有効化。実車では無効 |
| `awsim_boost.domain_enabled` | Domain 1..3=`true` | `ROS_DOMAIN_ID`ごとの有効/無効上書き。未設定Domainは`enabled`を使う |
| `awsim_boost.mode` | `start_once` | 1セッションにつき1回だけ発動 |
| `awsim_boost.trigger` | `first_forward_motion` | Ready後の初回前進を基準に発動。`awsim_start`は旧タイミングとの互換値 |
| `awsim_boost.motion_speed_threshold_mps` | `0.1` | 発車と判定する符号付き前進速度 |
| `awsim_boost.max_trigger_speed_mps` | `1.0` | 遅延発動を禁止する最大前進速度 |
| `awsim_boost.motion_trigger_timeout_sec` | `0.5` | 初回前進検出後に安全条件成立を待つ上限時間 |
| `mpc.steering_tire_angle_gain_var` | `1.5` | legacy / Recoveryのpublish時だけ適用する2025 AWSIM向け出力補償。5-state canonical Track/Cruiseへは重ねない。内部モデル、実機値、2026公式値は未確定 |
| `mpc.state_prediction_delay_sec` | `0.125` | EKF補正後の自己位置を速度・ヨーレートで先行予測する時間 [s]。`0.0` で無効 |
| `mpc.state_prediction_simulation_only` | `true` | `true` のとき明示的なsimulation launchでのみMPC初期状態予測を有効化 |
| `mpc.waypoint_local_association_enabled` | `true` | 前回tracking WP近傍の連続探索を有効化。`false`は全経路最近傍へ戻す |
| `mpc.waypoint_local_lookbehind_m` / `mpc.waypoint_local_lookahead_m` | `8.0` / `30.0` | 前回WPから局所探索する後方／前方経路距離 [m] |
| `mpc.waypoint_local_lost_distance_m` | `4.0` | 局所候補がこれより遠い場合だけ全経路で再捕捉 [m] |
| `mpc.wp_id_low_offset` | `1` | 低速時の入力参照 preview offset。整数 `0..2`。状態・制約の追従 waypoint は変更しない |
| `mpc.wp_id_low_speed` | `15.0` | 低速判定閾値。値は km/h、`0.0` なら無効 |
| `mpc.wp_id_offset` | `2` | 通常時の入力参照 preview offset。整数 `0..2`。早期減速と同方向の早期切り増しにだけ適用する |
| `mpc.center_bias` | `0.0` | `0.0` = CSV trajectory 追従、`1.0` = 左右制約中央寄せ |
| `mpc.safety_margin_scale` | `0.0` | `0.0` = 追加 margin なし、`1.0` = 標準 margin |
| `mpc.use_v2x_gap_planner` | `true` | `/v2x/vehicle_positions` から rule-based gap を作る dev3 向け拡張 |
| `mpc.v_max` | `40.0` | 全車両が越えないglobal hard maximum。値は km/h |
| `mpc.domain_v_max` | Domain 1=`20.0`, 2=`40.0`, 3=`10.0` | `ROS_DOMAIN_ID` ごとの通常最高車速。global maximum以下へ制限される |
| `mpc.domain_start_v_max` | 未設定 | `ROS_DOMAIN_ID` ごとのスタート期間最高車速。通常domain値を一時的に上回れるがglobal maximum以下 |
| `mpc.domain_start_v_max_duration` | `0.0` | 重複を除いた`/awsim/state=Start`から`domain_start_v_max`を適用する秒数 |
| `obstacles.csv_path` | `""` | 空 = トピック購読モード（障害物回避が off なので影響なし） |

`domain_start_v_max`のepochはMPC初回odometryではなくrace-session trackerが受理した`Start`である。Readyでduration以上待機しても期間を消費せず、同一sessionの重複Startでは延長しない。通常は`Finish -> Spawned`で次sessionを再armする。Finishを省く手動resetは、単独の遅延Spawnedでwindowを消さないよう`Spawned -> Grounded/Ready -> Start`の完全な進行を確認してspeed windowだけ再開し、Boostの使用済みlatchは再armしない。Boost出力を車両別に無効化していてもstart速度が設定されたSIM車両は`/awsim/state`を購読する。`use_sim_time=false`ではAWSIM Startを取得できないため通常domain maximumへ安全側fallbackする。

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

MPC コントローラはノード起動時にCSVのXY geometryを読み、内部heading／curvatureと速度profileを
再計算する。最小曲率lineとoccupancy grid自体はoffline生成物なので、
**コースが変わった場合はこの手順で再生成が必要**。

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
  v2x_prediction_use_path_time: false
  v2x_prediction_use_course_progress: false
  v2x_prediction_use_course_lateral_velocity: false
  v2x_prediction_course_lateral_velocity_deadband: 0.15
  v2x_prediction_course_lateral_velocity_max: 1.0
  v2x_prediction_min_ego_speed: 1.0
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
- `v2x_prediction_use_path_time=true`では、V2X車両の予測時刻を`waypoint index * Ts`ではなく、各path segmentの距離を参照速度（下限`v2x_prediction_min_ego_speed`）で割った時間の累積として求める。観測ageは別途加算し、`v2x_prediction_time`を超えない。
- `v2x_prediction_use_course_progress=true`では、V2X車両を現在速度の直線上ではなくreference pathの進行方向へ予測し、投影時の横偏差を維持する。ヘアピン等で将来位置がコース外へ直進する誤予測を抑えるための実験設定であり、bounded course projectionが成立しない車両は従来のCartesian等速直線予測へ戻す。
- `v2x_prediction_use_course_lateral_velocity=true`では、V2X速度推定に使った前回位置と現在位置をそれぞれbounded course projectionし、Frenet横偏差の差分から低速車の横速度を求める。`v2x_prediction_course_lateral_velocity_deadband`以下の揺れは0とし、残りを`v2x_prediction_course_lateral_velocity_max`へclampして、観測ageと各horizon時刻だけ横位置を先読みする。投影不能時は既存のCartesian横予測へ戻る。これは低速車が開けた空間を再び塞ぐ動きの先読みに限定した2025由来シミュレータ向け暫定実験であり、縦方向course-progress予測とは独立して切り替える。
- gap が選べる場合は `xr` を gap 中央へ寄せる。
- gap が壁と他車に挟まれている場合は、`v2x_wall_clearance_margin` で壁側制約を内側へ削り、`v2x_wall_avoidance_bias` で target を車側へ寄せられる。
- 左右が両方とも V2X 車両の gap は `v2x_vehicle_vehicle_gap_enabled=false` で通常候補から外せる。通常周回では `v2x_vehicle_vehicle_gap_min_distance/min_width` を適用する。start-grid専用の3回廊評価は後述の明示設定で分離し、通常値を暗黙に緩和しない。
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
  v2x_front_progress_detection_enabled: false
  v2x_front_progress_detection_distance: 0.0
  v2x_front_progress_lookbehind_distance: 3.0
  v2x_follow_distance: 8.0
  v2x_safety_brake_distance: 3.0
  v2x_safety_brake_margin: 2.0
  v2x_follow_gap_planner_enabled: false
  v2x_follow_gap_planner_no_gap_speed_limit_enabled: false
  v2x_follow_gap_planner_respect_overtake_forbidden: true
  v2x_follow_speed_limit_enabled: false
  v2x_follow_speed_limit_distance: 0.0
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
  v2x_overtake_block_inner_curve_pass: false
  v2x_overtake_outer_curve_entry_enabled: false
  v2x_overtake_outer_curve_hard_continuation_enabled: false
  v2x_overtake_inner_curve_entry_enabled: false
  v2x_overtake_inner_curve_min_open_distance: 0.0
  v2x_overtake_inner_curve_hard_continuation_enabled: false
  v2x_overtake_hard_curve_entry_enabled: false
  v2x_overtake_forbidden_curve_lookahead_distance: 0.0
  v2x_overtake_gap_lookahead_distance: 0.0
  v2x_overtake_try_both_sides: false
  v2x_overtake_velocity_advantage: 0.0
  v2x_overtake_stage_speed_enabled: false
  v2x_overtake_shiftout_adaptive_closing_speed_enabled: false
  v2x_overtake_shiftout_min_closing_speed: 1.0
  v2x_overtake_shiftout_max_closing_speed: 1.0
  v2x_overtake_pass_unlatched_max_closing_speed: 0.5
  v2x_overtake_shiftout_adaptive_min_time_sec: 0.5
  v2x_overtake_guard_enabled: true
  v2x_overtake_guard_min_gap_width: 2.5
  v2x_overtake_guard_min_gap_points: 3
  v2x_overtake_continue_min_gap_points: 3
  v2x_overtake_guard_min_prepare_distance: 8.0
  v2x_overtake_guard_max_lateral_shift: 1.2
  v2x_overtake_guard_reachable_gap_enabled: false
  v2x_overtake_guard_max_lateral_accel: 2.0
  v2x_overtake_guard_min_gap_time: 0.8
  v2x_overtake_continue_min_gap_time: 0.8
  v2x_overtake_guard_min_speed_for_reachable: 1.0
  v2x_overtake_guard_min_front_distance: 3.0
  v2x_overtake_continue_min_front_distance: 3.0
  v2x_overtake_close_follow_enabled: false
  v2x_overtake_close_follow_min_front_distance: 1.5
  v2x_overtake_close_follow_max_closing_speed: 0.8
  v2x_overtake_close_follow_min_side_clearance: 2.0
  v2x_overtake_before_curve_enabled: false
  v2x_overtake_before_curve_max_front_speed: 8.0
  v2x_overtake_before_curve_min_speed_advantage: 1.0
  v2x_overtake_continue_in_forbidden_enabled: false
  v2x_overtake_continue_inner_soft_curve_enabled: false
  v2x_overtake_active_hard_curve_completion_enabled: false
  v2x_overtake_active_hard_curve_rear_clear_distance: 0.5
  v2x_overtake_active_hard_curve_buffer_distance: 0.5
  v2x_side_overtake_entry_rear_tolerance: 0.5
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
  v2x_overtake_line_target_intrusion_ordering_margin: 0.10
  v2x_overtake_line_target_intrusion_guard_distance: .inf
  v2x_overtake_line_return_clear_distance: 4.0
  v2x_overtake_line_phase_hold_time: 0.3
  v2x_overtake_target_hold_sec: 0.0
  v2x_overtake_active_gap_loss_hold_sec: 0.0
  v2x_overtake_clear_confirm_sec: 0.0
  v2x_overtake_reacquire_enabled: false
  v2x_overtake_reacquire_window_sec: 0.0
  v2x_overtake_reacquire_max_return_progress: 0.0
  v2x_overtake_recovery_velocity_limit_enabled: true
  v2x_overtake_recovery_velocity: 3.0
  v2x_overtake_recovery_stall_speed: 0.15
  v2x_overtake_recovery_stall_timeout_sec: 1.0
  v2x_overtake_recovery_timeout_sec: 5.0
  v2x_overtake_recovery_max_observation_gap_sec: 0.2
  v2x_overtake_line_entry_retry_cooldown_sec: 1.0
  v2x_overtake_solver_cooldown_sec: 2.0
  v2x_overtake_solver_failure_abort_cycles: 3
  v2x_overtake_line_debug_log_enabled: false
  v2x_moving_front_speed_threshold: 1.0
  v2x_moving_follow_speed_margin: 2.0
  v2x_moving_follow_hard_distance: 0.0
  v2x_moving_follow_target_distance: 0.0
  v2x_moving_follow_recovery_speed_margin: 0.0
  v2x_moving_follow_distance_gain: 0.0
  v2x_moving_safety_brake_distance: 1.5
  v2x_moving_safety_brake_margin: 1.0
  v2x_moving_safety_brake_time_headway: 0.3
  v2x_start_grid_grace_time: 0.0
  v2x_start_grid_breakout_enabled: false
  v2x_start_grid_inter_vehicle_corridor_enabled: false
  v2x_start_grid_inter_vehicle_min_gap_width: 0.2
  v2x_start_grid_inter_vehicle_min_open_distance: 3.0
  v2x_start_grid_inter_vehicle_longitudinal_span: 12.0
  v2x_start_grid_inter_vehicle_lookbehind_distance: 4.0
  v2x_start_grid_inter_vehicle_lateral_radius: 1.25
  v2x_require_gap_for_overtake: true
  v2x_low_speed_avoidance_enabled: false
  use_v2x_local_path_planner: false
  v2x_low_speed_avoidance_distance: 10.0
  v2x_low_speed_avoidance_lookahead_distance: 18.0
  v2x_low_speed_avoidance_velocity: 1.5
  v2x_low_speed_avoidance_shift_velocity: 1.0
  v2x_low_speed_avoidance_pass_control_velocity: 1.0
  v2x_low_speed_avoidance_rejoin_control_velocity: 1.0
  v2x_low_speed_avoidance_shift_lateral_gain: 0.4
  v2x_low_speed_avoidance_shift_heading_gain: 1.3
  v2x_low_speed_avoidance_shift_lateral_tolerance: 0.4
  v2x_low_speed_avoidance_shift_heading_tolerance: 0.2
  v2x_low_speed_avoidance_shift_clear_hold_sec: 2.0
  v2x_low_speed_avoidance_max_front_speed: 1.0
  v2x_low_speed_avoidance_min_gap_width: 0.5
  v2x_low_speed_avoidance_min_gap_points: 2
  v2x_low_speed_avoidance_clear_distance: 8.0
  v2x_low_speed_avoidance_stall_speed: 0.15
  v2x_low_speed_avoidance_stall_timeout_sec: 1.5
  v2x_low_speed_avoidance_stall_cooldown_sec: 3.0
  v2x_low_speed_avoidance_stall_max_observation_gap_sec: 0.2
  v2x_local_path_pass_clearance: 3.0
  v2x_local_path_return_distance: 6.0
  v2x_local_path_invert_target: false
  v2x_low_speed_pass_side: auto      # auto, left, right
  v2x_low_speed_pass_ramp_ratio: 1.0
  v2x_overtake_forbidden_wp_ranges: []
  v2x_state_hold_time: 0.5
```

- `Cruise`: 他車なし。V2X 由来の横目標変更は入れない。
- `v2x_behavior_debug_log_enabled=true` の場合、`v2x_behavior_debug_log_period_sec` 周期で V2X FSM の詳細ログを出す。ログには desired/final state、state hold 後の結果、front distance/speed、required decel、risk、`danger_action`、overtake forbidden、curve guard、左右gapと各拒否理由、対象vehicle ID、locked target相対位置、soft desired velocity、Follow速度capの適用有無と前走車種別、近距離回復capと符号付き速度margin、ShiftOut速度cap適用、hard curveまでの利用可能距離・必要追い越し距離、solver連続失敗数、block reason を含める。`danger_action=RelativeSpeedLimit`はmoving frontへ全停止せず相対速度capを使う状態、`SafetyBrake`はEmergency、hard center distance内、または停止・低速前車への全停止を示す。追い越しに入らず `Follow` に居続ける場合は、`follow_cap` / `follow_moving` / `clearance_cap` / `clearance_margin` / `left_reason` / `right_reason` / `block` を確認する。
- `v2x_front_progress_detection_enabled=true` の場合、自車とV2X車両を同じreference pathのbounded polylineへ射影し、自車直近waypointの接線距離ではなくコースに沿った前方距離でfront/danger/risk/overtake判定を行う。`v2x_front_progress_detection_distance`は既存Follow距離・動的停止距離との最大値として使い、`v2x_front_progress_lookbehind_distance`は自車直後のsegmentを射影候補へ残す。circular終端はwrapするが、lookaheadより遠い別ヘアピン枝は探索しない。MPC内部のsmoothing用閉路点など、ゼロ長の閉路segment・連続重複点は進捗0としてskipし、制御周期全体をsolver fallbackへ落とさない。V2X速度は採用segmentの接線方向へ射影する。debugの`progress=1`は共通進捗採用、`local_fd`は従来接線距離、`path_lat`は対象の経路横偏差を示す。現行dev3の24 m / 3 mは2025 AWSIM final_ver3向けのローカル暫定値であり、2026公式仕様ではない。
- locked targetの前回path progressを使う連続性制約付き投影が失敗した場合は、同じbounded searchから連続性制約だけを外した投影を診断目的で1回行う。制約なし投影が成功する場合だけ真のcourse progress discontinuityとし、制約なし投影も失敗するlookahead外、cross-track外、方向不一致は通常のtarget lossとして扱う。診断用投影結果はfront判定や追跡値には採用しないため、近接する別コース枝への関連付けを緩和しない。どちらの失敗もrear-clearとはみなさず、既存のtarget hold後にRecoveryへ移行する。
- `Follow`: 前方車あり、追い抜き禁止または gap 不足。`v2x_follow_speed_limit_enabled=true` の場合、汎用Follow速度capは `v2x_follow_speed_limit_distance` 以内だけ適用する。停止/低速車には前方距離から計算した停止可能速度と `v2x_follow_velocity` の小さい方を使う。動く前走車には `clamp(v2x_moving_follow_distance_gain * (front_center_distance - v2x_moving_follow_target_distance), -v2x_moving_follow_recovery_speed_margin, v2x_moving_follow_speed_margin)` を前走車速度へ加えた上限を使う。target distanceまたはgainが0の場合は、従来互換として固定の`front speed + v2x_moving_follow_speed_margin`を使う。前走車が動いている場合は固定の `v2x_follow_velocity` を重ねない。距離を `0.0` にするかキーを省略した場合は、従来互換として検出された全距離で汎用capを適用する。これは前方車の検出距離とは独立しており、現行dev3では共通コース進捗で24 m先まで検出しつつ、汎用capは5 mから開始する。5 mは2025 AWSIMシミュレーション予選向けの攻めた暫定値であり、2026公式仕様ではない。front risk、curve risk、front decel guard、SafetyBrakeはこの5 mゲートの対象外なので、緊急回避判定を無効化しない。`v2x_follow_gap_planner_enabled=true` の場合は、Follow のままでも feasible な gap planner 出力だけを横制約と target に反映する。`v2x_follow_gap_planner_no_gap_speed_limit_enabled=false` なら、gap 不成立時の `no_gap_target_velocity` は Follow では使わず、譲り減速を避ける。`v2x_follow_gap_planner_respect_overtake_forbidden=true` なら、曲率または WP 範囲で overtaking forbidden の区間では Follow の gap planner も止め、ヘアピン入口で横に張りに行って進路を失う挙動を抑える。`v2x_follow_preposition_enabled=true` の場合は、WP 禁止ではないソフトな曲率禁止区間でも、カーブ外側かつ前方距離が残っているときだけ弱い lateral target を出し、真後ろ追従から外れる準備をする。通常の overtake pass side が内側を選んだ場合でも、Follow preposition だけはカーブ外側へ差し替える。
- `v2x_front_decel_guard_enabled=true` の場合、通常 Follow の速度制限を無効にしていても、近距離の動く前走車に対しては `front speed + v2x_front_decel_guard_speed_margin` の速度上限を掛ける。これは通常追走で失速させるためではなく、前走車の減速に追従できず追突するケースの緊急ガードである。`v2x_front_decel_guard_min_closing_speed` 未満の閉じ速度では発火させず、前走車の後ろに付いているだけの後続車を不要に失速させない。追い越し禁止カーブ中は `v2x_front_decel_guard_curve_distance` と `v2x_front_decel_guard_curve_ttc` まで判定距離を広げ、速度差を付けた車両がカーブで前走車へ追いつくケースを早めに抑える。直線で譲らせずヘアピンだけ追従させる場合は、`v2x_front_decel_guard_distance=0.0` と `v2x_front_decel_guard_ttc=0.0` にし、curve 側の距離/TTC だけを使う。ヘアピンで前走車が `v2x_moving_front_speed_threshold` 以下まで落ちる場合は、`v2x_front_decel_guard_curve_include_slow_front=true` にしてカーブ中だけ低速前走車も速度上限の対象にする。前走車が曲がり込みで横から進路を塞ぐ場合は、`v2x_front_decel_guard_curve_lateral_margin` でカーブ中の前方判定横幅を広げる。`v2x_front_decel_guard_curve_lookahead_distance` は速度上限用の曲率先読み距離で、`v2x_overtake_forbidden_curve_lookahead_distance` より短くすることで、横攻め停止だけを早めて失速開始を遅らせられる。
- `v2x_front_risk_arbitration_enabled=true` の場合、前走車との相対速度と有効距離から required decel を計算し、`EmergencyBrake` では SafetyBrake へ倒す。`BrakePrepare` は既定では警戒レベルとして扱い、`v2x_front_risk_brake_prepare_limit_enabled=true` の場合だけ速度制限に使う。`AvoidCandidate` も既定では速度制限に使うが、レース中に譲りすぎる場合は `v2x_front_risk_avoid_candidate_limit_enabled=false` で警戒レベルに落とせる。Phase 1 では reachable gap との統合は行わず、ブレーキ開始の遅れを切り分けるための braking guard として使う。
- `v2x_front_risk_curve_limit_enabled=true` の場合、曲率ガード中だけ required decel ベースの速度上限を追加する。直線の競り合いでは `BrakePrepare` / `AvoidCandidate` の速度制限を使わず、ヘアピンなど先行車が急減速する区間だけ `v2x_front_risk_curve_limit_decel` を前提に追突しない相対速度へ落とす。`v2x_front_risk_curve_limit_required_decel` は発火閾値、`v2x_front_risk_curve_limit_speed_margin` は前走車速度に足す余裕速度である。
- レース中に譲りすぎる場合は、距離/TTC ベースの `v2x_front_decel_guard_enabled` を `false` にし、required decel ベースの `v2x_front_risk_arbitration_enabled=true` へ切り替える。さらに `v2x_front_risk_brake_prepare_limit_enabled=false` と `v2x_front_risk_avoid_candidate_limit_enabled=false` にすると、通常の競り合いでは速度を落とさず、EmergencyBrake だけを残す。
- `Overtake`: 低曲率かつ十分な gap がある場合だけ gap planner を許可する。`v2x_overtake_guard_enabled=true` の場合は、gap 幅だけでなく、連続した gap 点数、gap までの準備距離、前方車との距離を追加確認してから Overtake に入る。`v2x_overtake_guard_reachable_gap_enabled=true` の場合は、最初に使える gap までの距離を現在速度から時間へ変換し、`2 * abs(target_ey - current_ey) / t^2` で必要横加速度を近似する。`v2x_overtake_guard_min_gap_time` 未満で現れる gap、または `v2x_overtake_guard_max_lateral_accel` を超える gap は Overtake 候補から外す。`v2x_overtake_guard_max_lateral_shift` は絶対横移動量の追加上限で、`0.0` の場合は無効である。早めに追い越し準備へ入るほど総横移動量は大きく見えるため、通常は `0.0` にして時間込みの lateral accel guard を優先する。これにより、通れない側へ一瞬振ってから反対側へ戻るような近距離 gap 飛び込みや、高速度では横移動が間に合わない gap へ突っ込む挙動を Follow / SafetyBrake 側へ倒す。`v2x_overtake_forbidden_curve_lookahead_distance` を指定すると、MPC horizon `N` より先の曲率まで見て overtake forbidden を立てるため、ヘアピン手前から横攻めを止められる。`v2x_overtake_gap_lookahead_distance` を指定すると、追い越し可否判定と Overtake 中の gap planner だけを MPC horizon より先まで伸ばして評価する。前方検出距離が MPC horizon より長い場合でも、低速前走車の先にある通過側 gap を早めに見つけられる。`v2x_overtake_target_ramp_enabled=true` では、長い lookahead 上で見つけた最初の追い越し target へ、現MPC horizon の先頭側から `v2x_overtake_target_ramp_ratio` の比率で target_ey を立ち上げる。これにより、前方車がまだ10点 horizon内に入っていない段階でも横方向の意思をMPCへ渡せる。
- `v2x_overtake_guard_min_front_distance` は新規追い越しの開始閾値である。すでに `Overtake` へ入った車両には `v2x_overtake_continue_min_front_distance` を使い、開始時専用の `v2x_overtake_guard_min_prepare_distance` は再適用しない。gap連続点数とgap timeも入口の`v2x_overtake_guard_min_gap_points` / `v2x_overtake_guard_min_gap_time`から、継続用の`v2x_overtake_continue_min_gap_points` / `v2x_overtake_continue_min_gap_time`へ切り替えられる。継続用キー省略時は入口値を使う。`v2x_overtake_active_gap_loss_hold_sec`が0より大きい場合、locked ShiftOut / Pass中のgap width、gap time、reachable gap欠落だけを、最後に有効gapを確認した時刻から指定秒数だけholdする。hold自身では期限を延長せず、front distance、横加速度、反対側gap、明示禁止WP、cooldown、EmergencyBrake、target position jumpは緩和しない。現行dev3の入口/継続前方距離5.0 m / 1.8 m、連続点2 / 1、gap time 0.5 s / 0.3 s、hold 2.0 sは2025 AWSIM向けの攻めたローカル暫定値である。`output/20260721-102131`ではWP61からWP63まで約0.9秒だけ選択側gapが欠落し、旧0.5秒holdが切れて`ShiftOut -> Recovery`となった。さらに`output/20260721-221420`ではWP63からWP64の回転座標上で選択側gapが一時的に消え、1.0秒holdが約0.03秒不足して`Pass -> Recovery`となったため2.0秒へ延長した。
- bounded active gap-loss hold中は、同じ一時的なplanner不成立が返す`no_gap_target_velocity`を速度上限へ重ねない。hold中は明示lineを継続すると既に判断しているため、no-gap hard limitだけを残すとline継続と最大制動を同時要求するためである。hold期限切れ、明示禁止WP、cooldown、EmergencyBrake、target position jumpではhold自体が成立せず、従来のcorridor blockとno-gap速度制限へ戻る。通常Follow、front risk、Recovery、wall、solver/odometry guardは変更しない。
- `v2x_overtake_line_enabled=true` の場合、通常 `Overtake` 中の横参照を `ShiftOut` / `Pass` / `Return` / `Recovery` の内部フェーズで生成する。ShiftOut / Passでは明示lineを横参照の唯一のownerとし、legacy side targetとgap plannerの横目標を重ねない。gap plannerは各周期で車両占有を含むlive回廊の有無を監視するが、明示lineがShiftOut / Passを所有している間は、選択したfree intervalをMPCのhard `lb/ub`へ強制しない。車両膨張で分断されたpass側intervalは現在状態から連続な到達可能集合ではなく、ShiftOut開始時だけでなく横分離latch直後のPassでもtargetを不連続に跳ばしてQPを不成立にするためである。live plannerが回廊なしと判定した場合と、ahead locked targetが選択側lineへ侵入した場合は、hard boundではなく明示lineのRecovery遷移で中止する。pass sideと安定した対象vehicle IDは追い越し開始時にlockし、phase開始からの累積走行距離とhorizon距離を使った`smoothstep`で横移動するため、制御周期ごとにrampが自車位置から再開しない。追い越し入口では、gap plannerが対象車両の膨張と壁余裕を適用した後の選択側free intervalについて、最初の有効区間の中央を自車中心の横目標として固定する。これにより前車の横位置へ吸い寄せられず、車両端と壁側限界の中央を狙う。有効な区間中央を取得できない場合だけ、従来のlocked target横位置と`v2x_overtake_line_min_target_separation`による目標へfallbackする。ShiftOutからPassへは`v2x_overtake_line_shift_distance`の走行と、wall clearanceを反映した横目標への収束を両方確認して移行する。Pass中の前後重なり解除は、対象と自車を同じreference pathへ射影した横差で判定し、一度成立したらlatchする。latch後は同一target、同一pass side、position jumpなしを満たす間をcommitted Passとし、入口用のgap幅・gap time・到達横加速度を再適用して中断しない。hairpin中に対象がfront/side分類から一時的に外れても同じlineを維持するが、明示禁止WP、curve実行許可、EmergencyBrake、target continuity、solver/odometry guardとlive回廊可否監視は緩和しない。1周期のfront/side欠落ではReturnへ入らず、`v2x_overtake_target_hold_sec`までPassを保持する。対象が`v2x_overtake_line_return_clear_distance`以上後方にあり、その観測が`v2x_overtake_clear_confirm_sec`継続して初めてReturnへ入る。Return初期に同一ID・同一sideを再取得した場合は、時間、復帰進捗、gap、curve実行許可を再確認してPassへ戻せる。不明ID、position jump、timeoutはRecoveryへ倒す。targetはwall `lb/ub`から`v2x_overtake_line_min_wall_clearance`だけ内側へclipし、必要横加速度が`v2x_overtake_line_max_lateral_accel`を超える場合はtarget変化を抑える。ShiftOut / Return / Recovery の進捗距離は周期ごとの前進速度を積算し、時刻巻き戻り、非有限値、`v2x_overtake_recovery_max_observation_gap_sec`を超える観測間隔は積算しない。Recoveryは`v2x_follow_velocity`を流用せず、専用flagが有効な場合だけ`v2x_overtake_recovery_velocity`を固定上限として使う。現在速度との`min`は取らないため、停止後も再発進可能な速度参照を維持する。`v2x_overtake_recovery_stall_speed`未満が`v2x_overtake_recovery_stall_timeout_sec`続くか、Recovery全体が`v2x_overtake_recovery_timeout_sec`を超えた場合はline stateを解除する。`LowSpeedAvoidance`と`SafetyBrake`は常に優先する。現行dev3設定はshift 4.0 m、Return 6.0 m、target bias 1.0、最大横加速度6.0 m/s2、rear-clear 2.0 m、確認0.10 sであり、2025 AWSIMシミュレーション予選向けの攻めた暫定値である。
- 攻撃設定のminimum-motion Passでfront capを解除済み、locked targetが前方3.0 m以内、target continuityと実壁・EmergencyBrake・solver guardが正常、現在車体が非重複または未確認重複grace内、かつ予測body footprint sweepが非重複の場合は、`side_by_side_commit`として同じpass sideのSafeSeparationへ早期にcommitできる。commit前には、`max(current speed, target speed + safe-separation delta)`を上限速度でclipした前進速度と相対閉速度からrear-clear 2.0 mまでの必要前進距離を求め、SafeSeparation局所距離上限（現行12 m）内で抜き切れる場合だけ許可する。初回認可後はforward completionをlatchし、予測sweepだけが重複した場合は`v2x_overtake_pass_predicted_overlap_confirm_sec`（現行0.25秒）未満なら同じ側の前進完遂を維持する。連続確認後は前進固定を解除し、同じ側のSafeSeparationを中止する。初回認可、予測不能、確認済み車体重複、実壁接触、EmergencyBrake、target discontinuity、solver failureにはgraceを適用せずfail closedを維持する。forward completion開始済みの場合はPass全体の絶対距離・時間上限を越えても現在のSafeSeparation局所枠だけは完遂に使用できるが、絶対上限到達後の局所枠再延長は許可しない。Returnはrear-clear確認後に限る。診断は`forward_commit`、`required_forward`、`distance_ok`、`overlap_elapsed`、`SafeSeparation entered: trigger=side_by_side_commit`で確認する。これは2025 AWSIM競技シミュレーション向けの暫定方針であり、実車安全仕様ではない。
- 20260808以降のforward completionでは、`v2x_overtake_safe_separation_full_speed_forward_escape_enabled=true`のとき、横へ分離済みで同じ側を前進完遂できるSafeSeparationの速度参照を`target speed + safe-separation delta`へ固定せず、domain/global cap内の通常コース速度まで解放する。加速度上限、コース曲率速度、wall/static-map、EmergencyBrake、solver/odometry guardは維持する。`v2x_overtake_safe_separation_mission_aligned_budget_enabled=true`では、選択済みMissionの予測rear-clear地点に距離marginを加えた値と、その距離を現在速度で走る時間に時間marginを加えた値から局所SafeSeparation枠を開始時に固定し、Pass全体の絶対上限（現行10秒 / 40 m）を越えない範囲へclipする。これにより、成立確認済みMissionが固定12 mの局所枠だけで途中終了する不整合を避ける。
- `v2x_overtake_safe_separation_rearward_progress_time_grace_enabled=true`では、locked targetが既に後方、short horizonが安全、同側forward escapeがactive、かつtarget相対位置の改善がfreshな場合に限り、局所／絶対wall-clock上限だけによるAbortを抑止する。局所・絶対距離上限、short-horizon、wall/body hard faultは緩和しない。fresh progressが失われた次周期では従来の時間Abortへ戻る。これはAWSIM衝突ペナルティや加速応答により時間だけを消費している一方、車両はrear-clearへ進んでいるPassをRecoveryへ誤遷移させないための2025競技シミュレーション向け暫定処理である。
- `v2x_overtake_contact_continuation_enabled=true`は、2025 AWSIM競技シミュレーションで接触後に自車だけが追い越しMissionを即破棄する事象への限定的な例外である。Pass中、forward completion latch済み、同一target継続、front cap解除済み、確認済み車体overlap、targetがcommit側と反対、横差0.75 m以上、縦差2.5 m以内、縦closing速度3.0 m/s以下、横相対速度0.5 m/s以下、自車速度0.5 m/s以上を満たし、開始0.25秒以内または前進進捗が0.5秒以内に更新されている場合だけ、最長0.8秒、同じtarget / side / Mission generationを保持する。縦速度は上記forward completionを維持し、横目標は相手から離れるcommit側へ0.10 mだけbiasする。正面衝突、高closing衝突、高い横接近速度、壁接触またはwall sample不成立、target discontinuity、EmergencyBrake、solver failure、進捗停止、0.8秒超過では従来どおりfail closedとする。接触を目標にする機能ではなく、回復可能な横接触から前進分離するためのsim-only方針であり、実車安全仕様ではない。診断は`full_speed`、`budget_sec`、`budget_m`、`contact_continue`、`contact_elapsed`、`contact_progress`、`contact_bias`および`OvertakeLine ContactContinuation entered/ended`で確認する。
- full Mission採用後、実測相対速度が新規Overtake entry gateに達するまでのpre-armはbase racing lineを維持し、採用Missionのclosing speedを使って縦速度参照を作る。採用Missionが無効な場合だけ設定上の最大ShiftOut closing speedへfallbackする。これにより、pre-armだけがMission rolloutより強い速度差を要求して入口直前に距離を潰す不整合を避ける。
- `v2x_overtake_mpcc_lite_control_enabled=true` では、左右のcomplete Missionに加えて、ShiftOutからbody-clearと短い同側継続までを検証したreceding prefixを共通scoreで比較する。prefixはwall、target surface clearance、body-clear deadline、横加速度をhard constraintとして維持し、rear-clear/Returnが現在horizon外であることだけでは棄却しない。5 Hzの戦術評価間は同一targetのlast-feasible prefixを最大`v2x_overtake_mpcc_lite_shadow_last_feasible_max_age_sec`保持し、40 Hzの低レベルMPCがそのlineを追従する。prefixのcontrol authorityは新規ShiftOutのside、goal、closing speedに限定する。active Missionの反対側置換は従来どおりcomplete Mission、atomic preflight、no-return gateを要求し、Return、wall/contact hard fault、EmergencyBrake、solver recoveryを上書きしない。このMPCC-liteは2025 AWSIM競技シミュレーション向けの段階実装であり、連続非線形MPCCによる全Phase同時最適化ではない。
- 通常の新規ShiftOut候補は、候補corridor中央と対象車横分離から最終横目標を求めた後、実行時と同じpath bound、`v2x_overtake_line_min_wall_clearance`、実車体static-map footprint、`v2x_overtake_line_max_lateral_accel`でhorizonを事前評価する。追加壁余裕が収まらない、physical footprintの通路がない、またはstatic wall clamp後の横加速度が上限を超える候補は、OvertakeLineへcommitする前にその側だけを棄却する。したがって同じ幾何条件で`Idle -> ShiftOut -> Recovery`を繰り返さない。start-grid専用の膨張済み車間corridorは既存の専用検証を正本とし、この通常入口preflightを重ねない。実行中に同じstatic-wall系失敗が発生した場合も、`v2x_overtake_line_entry_retry_cooldown_sec`の間は同一target・同一sideのRecovery reacquireを抑止する。反対側候補、通常Follow、実footprint接触、EmergencyBrake、solver/odometry guardは変更しない。現行1.0秒は2025 AWSIM `make dev2`用の暫定値である。
- 通常ShiftOutのpreflightは、MPC horizon内でcandidate gapが返した車両膨張済みfree intervalの共通部分とroad boundを交差し、その中で対象車との最小横分離を満たす固定`goal_ey`を求める。この共通部分がないsideはcommit前に棄却する。承認した`goal_ey`は実行lineへそのまま渡し、距離ベース`smoothstep`に加えて周期単位のendpoint slewを重ねない。これにより入口で検証した4 m ShiftOutが、異なる目標や二重平滑化で長距離化する不整合を防ぐ。
- target ID・pass side・固定goalを持つShiftOut / PassをPass Missionとして扱う。SafetyBrakeは従来どおり速度0を最優先する一方、missionを消去せず`FollowPrepare`へpauseし、pause中はOvertakeLineの横出力と追い越し速度を出さない。危険解除後はlocked targetを最新V2Xへ再投影して両side、wall、candidate corridor、横加速度を再検証し、成立したsideで`ShiftOut`を再開する。新規entry用のhard-curve完遂距離判定はcommit済みmissionへ再適用せず、停止後に実速度差を先に要求する循環を作らない。通常のRecoveryが距離または横復帰で完了した場合も、target continuityが有効で実壁接触・solver recovery・明示禁止WPがなければ`FollowPrepare`へ移してmissionを保持する。target jump、course-progress discontinuity、timeout、実壁接触、EmergencyBrake、solver/odometry fail-safeは緩和しない。
- 明示lineは各horizon点の`e_y`からFrenet offsetの接線方向を計算し、`e_psi = atan2(d'(s), 1-kappa*d)`もMPC参照へ渡す。これによりShiftOut中に「横へ移動する`e_y`」と「base trajectoryへ常に平行な`e_psi=0`」を同時要求しない。ShiftOutまたは横分離未成立のPassでgap plannerが追い越し側の実行回廊を生成不能とした場合は、live回廊専用の最終成立時刻から`v2x_overtake_active_gap_loss_hold_sec`だけ明示lineを保持し、期限内に回廊が復帰しなければRecoveryへ移行する。通常gapの成立時刻とは分離し、hold中の不成立では期限を延長しない。ただしShiftOutとPassは実行幾何が異なるため、現在の自車・locked target車体footprintが非重複な状態でPassへ入った場合は、Pass開始時刻とlive回廊の最終成立時刻の新しい方をPass中のhold基準にする。これによりShiftOutで消費したhold時間だけをPassへ持ち越さず、同じPass中の不成立で期限を自己延長することはない。aheadのlocked targetについて、横分離成立前に`pass_side_sign * (target_lateral - ego_lateral)`が`-v2x_overtake_line_target_intrusion_ordering_margin`以上になった場合は、対象が選択側lineへ到達・横切ったとみなし、start-grid/未確定Pass continuityより優先してRecoveryへ移す。現行margin 0.10 mは左右順序の揺れを吸収する2025 AWSIM向け暫定値で、車体クリアランスそのものはgap plannerのinflationが担当する。Passが`v2x_overtake_pass_front_overlap_lateral_clearance`の横分離をlatchした後は、hairpinの回転座標で変化するlive planner不成立と左右順序を診断専用とし、それ単独ではRecoveryへ移さない。このdiagnostic-only状態では、同じplanner不成立が返す`no_gap_target_velocity`も速度上限へ適用しない。feasibleなplanner速度上限、ShiftOut・横分離未成立Passのno-gap制限、通常Followの設定、front risk、EmergencyBrake、Recovery、wall制約は維持する。この段階ではrear-clear確認が正常完了を担当し、wall clip、position jump、明示禁止WP、cooldown、EmergencyBrake、target continuity、solver/odometry guardを終了条件として維持する。
- OvertakeLineの通常目標は膨張済み通路中央とし、通路中央が得られないfallbackの対象車横分離には`v2x_overtake_line_min_target_separation`、汎用front-brake除外には`v2x_overtake_pass_front_overlap_lateral_clearance`を使い、同じ値を共用しない。`output/20260722-064048`では共通コース進捗上の前後逆転とReturn完了を記録したが、映像ではP2がP3を物理的に抜いていなかった。旧line最小分離0.75 m、固定line offset 1.2 m、横クリアlatch 1.15 mはいずれも、V2Xの横膨張`v2x_vehicle_radius + v2x_prediction_margin`（当時1.55 m、covarianceを除く）より小さく、制御内部だけがPass成立と判断できた。現行dev3暫定値は実車幅1.45 mを維持し、prediction marginを0.05 m、wall clearanceを0.72 m、line最小対象分離fallbackを1.40 m、横クリアlatchを1.35 mとする。これは5〜15 cm程度の接近を許す攻めたシミュレーション値であり、映像上の物理追い越しを次回dev3で確認するまで成功扱いにしない。ShiftOut最大closing speedは1.2 m/s、横分離latch前のPassは0.5 m/s、入口前方距離は5.0 mを維持する。旧キーしかない設定ではline最小分離へ旧値をfallbackし、既存yamlとの互換性を保つ。これらは2025 AWSIM dev3向け暫定値であり、実車値ではない。
- Passの横分離latchはline継続の履歴として保持するが、locked targetをgeneric front保護から外すには毎周期の現在横離隔が`max(v2x_overtake_pass_front_overlap_lateral_clearance, v2x_vehicle_radius + v2x_prediction_margin)`以上であることを要求する。過去に一度横離隔しただけで、hairpin後の再接近を無保護にしない。committed Passは`v2x_overtake_line_progress_watchdog_distance`進むごとにlocked targetの相対前方距離が`v2x_overtake_line_progress_watchdog_min_progress`以上改善したか確認し、改善しなければRecoveryへ移る。start-grid inter-vehicle corridorは単一target進捗を持たないため対象外とする。
- OvertakeLineのwall safetyは予測状態ではなくraw odometry poseの実車体footprintを静的gridへ毎周期照合する。ShiftOut / Pass中にposeまたはsampleが無効、map外、物理接触、wall clearance違反ならRecoveryへ移る。静的壁へclipした後も必要横加速度を再計算し、`v2x_overtake_line_max_lateral_accel`を超えるtargetは実行しない。Recovery targetにも同じ静的wall horizonを適用する。
- ShiftOut / Passで`v2x_overtake_solver_failure_abort_cycles`連続failureへ到達するか、Recovery中にsolver fallbackへ入った場合はsolver失敗episodeとして扱う。Recovery中のfallback操舵は`steer_rate_max`以内で中立へ戻しながら既存`a_min`で減速する。episode終了後は、`v2x_overtake_solver_cooldown_sec`経過と`v2x_overtake_solver_recovery_success_cycles`回の連続正常解を両方確認するまで、OvertakeLine、legacy side target、追い越しgap plannerを再開しない。gate中の1回のfailureで成功カウントは0へ戻る。2026-07-18の`output/20260718-001009`では、D2がWP86の8連続failureから復旧し、2秒と20連続正常解を確認してgateを解除した後、WP220まで走行した。Recovery中立復帰は対象unit testで確認したが、このrunではRecovery直後にsolverが復旧したため実command未観測である。値は2025 AWSIM向け暫定値である。
- 通常MPCのsolver failureでは、`solver_failure_steering_hold_cycles`以下の単発failureだけ直前操舵を保持し、その次の周期から`steer_rate_max`以内で操舵を0へ戻す。既定4周期は40 Hzで0.1秒の2025 AWSIM向け暫定値である。OvertakeLine Recovery / solver re-entry gateは待機せず中立復帰する。`output/20260718-003725`のD3では、連続failure時の操舵が1周期目`0.559 rad`、5周期目`0.529 rad`、10周期目`0.379 rad`、20周期目`0.079 rad`、30周期目`0.000 rad`となり、旧runの長時間固定操舵を解消した。一方、failure開始時に`e_psi=-1.808 rad`まで姿勢がずれており、Stuck Recoveryが`forward_duration_limit`でSafeStopしてD1/D2も後続停止したため、この変更は安全fallbackとして採用するがdev3デッドロックの解消とは扱わない。詳細は`.steering/20260718-normal-mpc-fallback-neutralization/results.md`に記録する。
- `solver_failure_crawl_enabled=true`では、simulation time、control enabled、V2X `Cruise`、front vehicleなしをすべて満たす通常MPC failureに限り、減速fallbackのゼロ速度化を上位arbitrationで置き換える。速度は`min(solver_failure_crawl_speed_mps, effective_v_max)`、操舵は既存の低速横偏差・heading feedbackを`e_y=0`へ向けて使用する。SafetyBrake、Follow、Overtake、LowSpeedAvoidance、front検出中、実時間動作では従来の減速fallbackを維持する。停止速度以下でsolver fallbackが継続する場合は、AWSIM collision correctionとwaypoint再関連付けによる離散pose/progress変化を走行再開とは扱わず、既存の2–3秒stuck recovery gateを進める。現行1.0 m/sは2025 AWSIM dev3向け暫定値である。
- OSQP wrapperはvalidation、CSC allocation、setup、solve、status、solution、constraint checkを失敗stageとして区別する。solver infoが得られる失敗ではstatus文字列、status value、iteration、primal residual、dual residualを`MPC control failed`のreasonへ含める。上位層で一律`OSQP failed`だけに丸めず、infeasible、最大iteration、数値誤差、post-solve制約違反を次runで切り分けられるようにする。
- V2X behavior、gap planner、OvertakeLineが現速度より低い動的速度上限を要求した場合、全horizonへ最終上限を即時適用しない。現速度から`abs(a_min) * Ts`ずつ下げるreachable upper-bound envelopeを速度参照と上限制約の両方へ適用する。これにより`SafetyBrake=0 m/s`や`Overtake -> Follow=3 m/s`でも既存`a_min`を超える減速を要求せず、最終上限とpublished acceleration clampは維持する。現行`a_min=-1.35 m/s^2`、40 Hzは2025 AWSIM向け暫定値である。
- `v2x_overtake_try_both_sides=true`では、追い越し開始前に第一候補が不成立なら反対側を同じ条件で再評価する。ShiftOut以降はlocked側だけを評価し、反対側だけが空いても即side flipしない。通常追い越しの候補生成幅は`max(v2x_overtake_min_gap_width, v2x_overtake_guard_min_gap_width)`を使うため、共通`gap_min_width`が後段guardより大きくてもguard設定を前段で無効化しない。vehicle-vehicle/multi-front policyは引き続き適用する。
- 現行dev3の攻めたgap設定は`v2x_overtake_min_gap_width=0.2 m`、`v2x_overtake_guard_min_gap_width=0.2 m`を維持する。一方、明示追い越し線の`v2x_overtake_line_min_wall_clearance`は0.8 mとする。これは候補回廊の必要幅ではなく、ShiftOut / Passの各予測点でMPC wall boundから追加確保するego-center余裕であり、車-車回廊の0.2 m判定は変更しない。`output/20260723-060039`では通常wall-vehicle線の固定`goal=-4.13 m`をrear-clear後も17秒以上保持したため、WP62までイン壁へ張り付いた。Pass中はbehaviorのcommitted continuityよりrear-clear確定を優先してReturnへ移し、後方へ抜いた同一targetによる即時Pass再取得も禁止する。これらは2025 AWSIMシミュレーション予選限定の暫定値で、実車値ではない。`output/20260719-233608`では0.8 m未満の候補区間が判定対象になった一方、実際に選択された側は`side_clear=2.56〜3.97 m`で、狭い回廊の実通過は未観測だった。詳細は`.steering/20260719-aggressive-overtake-gap-02/results.md`に記録する。
- `lb/ub`はreference path各点から壁cellへ引いた横断線を平滑化した値であり、ヘアピンの向き付き車体が実壁へ接触しないことまでは保証しない。`output/20260723-062353`では`wall_clearance=0.80`を適用したPass目標`e_y=-3.94 m`に対し、実車`e_y=-3.916 m`で停止復帰用occupancy gridが32 cellの壁接触を検出した。OvertakeLineは各horizonの横目標へ、停止復帰と共通のoccupancy gridおよびego矩形を使う追加検証を行う。左右extentだけを`v2x_overtake_line_min_wall_clearance`分膨張し、接触する目標をreference path側の最初の成立位置へclampする。要求marginがコース幅へ収まらない場合は物理車体だけの接触回避へ縮退し、それも不成立ならreference path目標とRecovery速度上限を使う。debugの`static_wall_limited`、`static_margin_degraded`、`static_wall_infeasible`で各状態を区別する。これはOvertakeLineだけの補正であり、通常trajectory追従の`lb/ub`契約は変更しない。
- gapの幾何可否とcurve実行許可は別に保持する。soft curvature zoneだけでなくhard curvature zoneでも曲率符号から内外側を分類するが、明示禁止WPでは分類結果を実行許可に使わず常にblockする。開始直前curve clearance、inner/outer curve設定、cooldownを満たさない候補はMPCへ反映しない。`v2x_overtake_completion_guard_enabled=true`では、新規開始時に`v2x_overtake_completion_hard_curvature`を超える次のhairpinまでの距離を求め、前方距離、前後速度差、rear-clear距離、ShiftOut距離、bufferから推定した必要距離を確保できない追い越しを拒否する。planned ego speedにはnominal/reachable速度と実際のOvertake stage速度上限の小さい方を使い、ShiftOut中に指令できない速度を前提として完遂可能と判定しない。完遂距離不足をsoft/hard curve entry例外で迂回する場合だけ、target前方距離が`v2x_overtake_guard_min_front_distance + v2x_overtake_line_shift_distance + v2x_overtake_line_return_clear_distance`以内で、自車の実測速度が前走車より`v2x_overtake_completion_min_relative_speed`以上速いことを要求する。現行dev3では11 mであり、14〜24 m先からpass lineへ出てヘアピン減速で投影範囲外へ離される反復を抑える2025 AWSIM向け暫定範囲である。横lineのshift/rear-clear値をcurve-entry捕捉距離にも使う結合であるため、この範囲を独立調整する必要が生じた場合は専用paramへ分離する。通常の完遂距離が成立する入口、start-grid breakout、開始済みcurve-side追い越しの継続にはこの近距離・実測速度差条件を追加しない。
- 完全なShiftOut/Pass/Return Missionの机上成立は、新規Overtakeの実測速度条件を迂回しない。Mission rolloutは自車の`a_max`とpath speed capを扱う一方、対象車の将来加速は一定速度近似であるため、`v2x_overtake_entry_min_relative_speed`以上の実測相対速度を`v2x_overtake_entry_speed_confirm_sec`連続確認してから横Missionへhandoffする。完全Missionが同周期で成立し、Emergencyでないが速度だけ未達の場合はentry pre-armとし、Behavior表示は`Follow`、OvertakeLineは`Idle`のまま基準走行線を維持する。この間はgeneric Follow cap、follow gap planner、follow prepositionを重ねず、対象速度+採用Missionのclosing speedまでの縦加速だけを許可する。実測速度確認はtarget単位で保持し、左右、横goal、closing speedの候補順位変化ではリセットしない。完全Missionが一周期程度欠落しても、同一targetかつhard guardが健全なら`v2x_overtake_entry_prearm_validation_hold_sec`だけ速度確認履歴を保持するが、その周期はpre-arm加速ownershipも横Mission handoffも許可しない。handoffは必ずその周期に再検証済みの完全Missionを使い、target変更、Emergency、明示禁止waypoint、position jump、solver recoveryでは保持を即解除する。body-clear、rear-clear、壁、Returnまで検証済みの完全Missionは新規entryのcompletion正本とし、粗い次hard-curve距離比較だけでは二重棄却しない。start-grid breakoutは等加速車両間で正の相対速度を先に要求すると発進できないため、専用観測・corridor検証成立時の即時handoffを維持する。現行`+0.3 m/s`、`0.3 s`、validation hold `0.15 s`は2025 AWSIM競技シミュレーション用の暫定値である。debugの`prearm`、`prearm_lease`、`entry_rel`、`entry_stable`、`prearm_v`、`completion_mission_override`で確認する。
- OvertakeLineがShiftOut / Passへcommitした後は、Behavior FSMのsoft/hard curve入口、completion距離、cooldown、soft forbiddenなど新規開始用の候補判定を再度キャンセル条件に使わない。Behaviorが一時的にCruise / Followへ切り替わっても、同一locked targetのcourse progressが連続し、targetが前方、pass側侵入なし、live execution corridor blockなし、明示禁止WPなし、Emergencyなしであれば固定済みtarget / side / corridorを保持する。target位置ジャンプ・期限切れ・進捗不連続、actual/static wall、横加速度不成立、solver failureは従来どおりRecoveryへ移す。`output/20260727-234145/d1`では20回の新規ShiftOutに対して`locked target no longer executable`が32回、同じtargetのRecovery再取得が19回発生し、Pass到達2回、Return完了0回だった。代表区間ではBehaviorの`Overtake -> Cruise`から約0.3秒後に同じtarget・sideを再取得していたため、これは新規候補判定のチャタリングであり実行時hard guardではないとして分離した。
- 新規Overtakeのpass sideは、内側が最低連続距離を満たしただけでは確定しない。`v2x_overtake_line_side_quality_selection_enabled=true`では左右それぞれのinflated vehicle side clearance、gap corridor width、連続open距離、ShiftOut preflight必要横加速度から品質値を作り、`v2x_overtake_line_side_quality_min_score_advantage`以上の差があれば高品質側を選ぶ。差が小さい場合だけ従来の幾何preferred sideをtie breakに使う。commit済みShiftOut/Passでもno-return前かつ現在車体が非重複で予測が有効なら左右のfull Missionを再評価でき、現在側の予測sweep重複は再評価禁止ではなく現在Mission不成立として扱う。反対側のShiftOut・Pass・Return全体が成立し、debounceを満たした場合だけfrozen Missionを一度だけatomicに置換する。予測不能、現在車体重複、no-return後、置換回数上限後はsideを変えない。別系統のearly side replanでは、locked targetが`v2x_overtake_line_side_replan_target_guard_distance`以内で選択側へegoを越えたcourse-relative orderingを`v2x_overtake_line_early_side_replan_stable_sec`継続した場合、左右を再評価する。横移動量が`v2x_overtake_line_early_side_replan_max_lateral_progress`以下かつ前進距離が`v2x_overtake_line_early_side_replan_max_traveled_distance`以下なら、成立した反対側へ現在位置から一度だけShiftOutを再計画する。window外、再計画済み、または反対側不成立なら直接side反転せずRecoveryへ移し、旧sideをentry retry blockする。`output/20260728-000047/d1`では左右成立から内側を選択し、target相対横位置が`-0.93 m`から`+0.85 m`へ変化して左右が再び成立してもfixed sideを保持し、最後は`wall=front`と8回連続solver failureになったため導入した。debugの`left_q`、`right_q`、`side_conflict`、`replan_window`、`replan_candidate`、`replan_stable`、`replan_ready`、`replan_abort`で判断を追跡する。過去A/Bで不安定だった生の`v2x_prediction_use_course_lateral_velocity`は今回有効化せず、現在orderingの継続時間で観測チャタリングを抑える。これは2025 AWSIM `make dev2`向けの暫定挙動である。
- `v2x_overtake_opponent_side_replan_enabled=true`では、frozen ShiftOut / Passに加えて`FollowPrepare`でも左右のfull Missionを周期評価する。paused frozen Missionではlocked targetの現在車体と予測sweepを観測対象へ含めるが、FollowPrepareへPass専用のfront brake抑止や速度所有権は与えない。dynamic mission waitへの新規入場は、targetがno-returnより前、現在車体非重複、予測有効、alternate replacement回数あり、かつtarget continuity、実壁余裕、Emergency、solver guardが正常な場合だけ許可する。待機中は旧側へ即再開せず、最新評価で現在側が完全成立した場合だけ同側を再開し、no-return前に反対側がdebounce成立した場合はMission全体をatomicに置換して現在位置からShiftOutする。no-return後に新たなsoft failureが発生した場合はdynamic waitへ変換せず、既存のsame-side forward completion、RecoverBehind、Recoveryを使う。actual wall contact/margin違反、wall観測不能、現在車体重複、target discontinuity、Emergency、solver failureは従来どおりRecoveryとする。`opp_no_return`は縦方向のno-return通過だけを示し、幾何未観測やreplacement回数切れとは分離する。debugの`opp_current`、`opp_alt_ok`、`opp_action`、`mission_wait`で確認する。これは2025 AWSIM競技シミュレーション向けの暫定挙動である。
- `v2x_overtake_stage_speed_enabled=true`では、ShiftOutと物理横離隔が未成立のearly Passで、前走車速度+closing speedの上限を作る。commit済みShiftOut / Pass中でも、同一targetとの現在横離隔が`max(v2x_overtake_pass_front_overlap_lateral_clearance, v2x_vehicle_radius + v2x_prediction_margin)`以上であることだけでは解除しない。pass側の横目標へ到達し、かつ当周期のOvertakeLine horizonが横加速度、wall bound、static wallのいずれにも制限されていない場合に限り、現在横離隔成立または同一targetの相対進捗が0 m以下を条件として、前走車由来上限を解除して元のtrajectory速度参照を使う。速度capの解除履歴はShiftOutからPassへ引き継ぎ、現在横離隔が`v2x_overtake_pass_front_cap_reapply_lateral_clearance`未満へ縮むか、横目標未到達またはhorizon制限が発生した場合は再適用する。committed Pass continuity用のfront-overlap exclusion latchは従来どおりPass移行後だけ確定し、ShiftOut中の一時的な横離隔だけでは継続条件を強めない。Behavior側はOvertakeLineが前周期に確定した解除状態だけを参照し、当周期に制約が発生すればOvertakeLine側の速度参照が即時に安全側へ上書きする。behaviorが一時的にFollowへ戻っても、解除条件未成立ならactive OvertakeLine側から同じ上限をMPC制約へ適用する。SafetyBrake、front risk、別の前方車、wall/corridor、curve動的上限、domain/global cap、加速度制約は維持する。無効時はlegacyの`v2x_overtake_velocity_advantage`を追い越し全期間のsoft上限として使う。`output/20260727-230821/d1`では横差2.41 mになった後もShiftOut capを約4.2秒保持したため横離隔だけの早期解除を導入したが、`output/20260727-232326/d1`ではtargetが7.53 m前方、横差1.52 m、`lat_limited=1`、`wall_limited=1`、`static_wall_limited=1`で解除し、約1.15秒後に`actual footprint wall margin violated`となった。このため横離隔を必要条件の一つへ戻し、横ライン完了とhorizon非制限を追加した。これは2025 AWSIM `make dev2`向けの暫定挙動である。
  `v2x_overtake_shiftout_adaptive_closing_speed_enabled=true`の場合は、走行距離と横移動残量の大きい方から見積もった残りShiftOut距離を現在速度で走る時間と、`front_distance - v2x_overtake_guard_min_front_distance`の距離予算から接近速度を求め、`v2x_overtake_shiftout_min_closing_speed`〜`v2x_overtake_shiftout_max_closing_speed`へclampする。現行dev3では距離budget枯渇時にも完全速度一致で横移動が止まらないよう最小値を0.5 m/sとする。Passへ入っても前後重なり解除がlatchするまでは`v2x_overtake_pass_unlatched_max_closing_speed`を上限にし、横クリアランスを作る前の追突を抑える。現行dev3値は0.5 m/sである。残り時間は`v2x_overtake_shiftout_adaptive_min_time_sec`未満にしない。適応時は進入後に上昇した自車速度が前走車由来上限を無効化しないよう、進入速度下限も同じ上限までに制限する。MPC投入前に全horizonのbounds、横target、速度参照、曲率参照をpreflightし、追い越し中に`v2x_overtake_solver_failure_abort_cycles`回連続でsolverが失敗した場合は減速fallbackを維持しつつ同じ横targetをRecoveryへ倒す。solver failure由来のRecoveryを解除した後は`v2x_overtake_solver_cooldown_sec`の間、lineだけでなくgap plannerとfallback lateral targetも抑止し、同じ不成立条件への即時再突入を避ける。現在のstall 1.0秒、Recovery 5.0秒、cooldown 2.0秒は2025由来シミュレータでのローカル暫定値であり、2026公式値ではない。`output/20260724-021819`のD1では物理横離隔latch後に`cap_release=1`、`desired_v=11.11 m/s`でも、diagnostic-onlyにしたcorridor不成立の`no_gap_target_velocity=2.0 m/s`が残り、実速度は6.40から1.82 m/sまで低下した。適用条件を揃えた`output/20260724-070818`では、同じdiagnostic-only区間の指令加速度が1.00 m/s2、指令速度が11.11 m/sを維持し、実速度は4.48から8秒後6.77 m/sへ増加した。ただしこの短時間runは後段のtarget course progress discontinuityでRecoveryへ入っており、正常Return完了の確認ではない。
- `v2x_overtake_close_follow_enabled=true` の場合、近距離で通常 fallback guard の `min_prepare_distance` を満たせないときでも、前方距離・横余裕・相対速度が安全側の範囲内なら `Overtake` の横 targetだけを許可する。ただしclose-followはprepare距離だけの例外であり、通常の新規入口`v2x_overtake_guard_min_front_distance`を下回って開始しない。さらに通常 overtake guard と同じ `v2x_overtake_guard_min_gap_time` / `v2x_overtake_guard_max_lateral_accel` で横移動の到達性を確認し、至近距離で急なU字経路が必要になる場合は Follow / SafetyBrake 側へ残す。真後ろに詰まってから永久に Follow に落ちるケースを避けるための例外で、相対速度が大きい場合や emergency front risk では使わない。既定は `false`。
- `v2x_overtake_before_curve_enabled=true` の場合、WP 明示禁止ではなく曲率先読みだけで overtake forbidden になっている区間では、前走車が `v2x_overtake_before_curve_max_front_speed` 以下で、自車が `v2x_overtake_before_curve_min_speed_advantage` 以上速く、かつ `front_decel_guard_curve_lookahead_distance` ではまだガードされていない場合だけ、新規 Overtake を許す。これは長い曲率先読みで直線中の低速前走車に張り付く問題を抑えるための例外である。`v2x_overtake_continue_in_forbidden_enabled=true` の場合は、すでに Overtake 中なら同じ soft forbidden 区間で Overtake 継続を許し、ヘアピン前に横へ出た車両が途中で Follow に戻される挙動を抑える。
- `v2x_overtake_continue_inner_soft_curve_enabled=true` は、開始済みOvertakeのロック側が曲率先読みで内側に変わってもsoft curve内だけ継続するdev3設定である。この設定単独では新規inner passを許可しない。明示禁止WP、curve cooldown、EmergencyBrakeでは継続を禁止し、gap到達性とwall clearanceも従来どおり再評価する。設定省略時の既定は`false`だが、現行dev3 configは開始済みPassをヘアピン境界まで完了させるため`true`としている。
- `v2x_overtake_outer_curve_entry_enabled=true` は、曲率符号から求めたカーブ内側と反対の外側に連続gapがある場合だけ、soft curve禁止区間で新規ShiftOutを許可する。`v2x_overtake_outer_curve_hard_continuation_enabled=true`では、同じ外側のlocked ShiftOut/Passが開始済みで、target観測と外側gapが継続している場合だけhard curveでもOvertakeLineを維持する。両設定の省略時既定は`false`、現行dev3では2025 AWSIM比較実験として`true`である。debugの`outer_entry=1`はsoft curve外側進入、`outer_hard=1`はhard curve外側継続を示す。
- `v2x_overtake_inner_curve_entry_enabled=true` は、曲率符号から求めた内側に連続gapがある場合だけsoft curve禁止区間で新規ShiftOutを許可する。未ロック時は左右の膨張済み回廊を同時に評価し、内側の連続有効距離が`v2x_overtake_inner_curve_min_open_distance`以上ならイン差し、それ未満なら実行可能な外側を既定として選ぶ。現行dev3は3.0 mであり、最低幅を一瞬満たしただけの内側回廊へ飛び込まないためのシミュレーションレース値である。どちらも成立しない場合はFollow/SafetyBrakeへ残る。一度ShiftOutを開始した側はtargetとともにlockし、後続周期の内外判定で反対側へ切り替えない。`v2x_overtake_inner_curve_hard_continuation_enabled=true`では、同じ内側locked ShiftOut/Passについて、target観測と内側gapが継続する場合だけhard curveでもOvertakeLineを維持する。明示禁止WP、cooldown、EmergencyBrake、target continuity/intrusion、wall/body回廊は従来どおり解除しない。設定省略時の既定はentryが`false`、最小連続距離が0.0 mで、現行dev3ではentryを`true`としている。
- `v2x_overtake_line_target_intrusion_guard_distance`は、横分離成立前の左右順序侵入をRecovery条件として使う前方距離である。それより遠方では、前車の後方にいる間に選択回廊へ横移動できるため左右順序だけでは中止せず、膨張済み回廊と到達横加速度を実行条件にする。現行dev3の2.0 mとordering margin 0.10 mは2025 AWSIM向けの攻めた暫定値で、キー省略時は無限距離として従来挙動を維持する。Passの横分離latch後は従来どおり左右順序を診断専用とする。
- `v2x_overtake_hard_curve_entry_enabled=true`は、上記の内側／外側entryが有効で、hard curve認識後にも対象側gapが成立する場合に新規ShiftOutを許可するdev3専用例外である。通常の完遂距離が不足している場合は、targetが上記の近距離entry範囲内にあり、自車の実測速度が前走車より`v2x_overtake_completion_min_relative_speed`以上速いことも要求する。明示禁止WP、curve cooldown、EmergencyBrake、target position jump、gap plannerのwall/body境界は緩和しない。設定省略時既定は`false`。debugの`outer_hard_entry=1` / `inner_hard_entry=1`はhard curve内からの新規進入、`gap_hold=1`は成立中ラインの時限gap holdを示す。
- `v2x_overtake_active_hard_curve_completion_enabled=true` は、新規追い越しのhard completion guardを変更せず、OvertakeLineがすでに`Pass`へ入ったlocked targetだけをhard境界残距離で再評価するdev3設定である。現在速度と`a_max`から境界までの到達可能速度を計算し、さらに実際のOvertake stage速度上限でclampした速度をplanned ego speedとする。`v2x_overtake_active_hard_curve_rear_clear_distance`だけ対象を後方へ抜くためのrequired distanceが、境界距離から`v2x_overtake_active_hard_curve_buffer_distance`を引いたavailable distance以下なら継続する。共通コース横差で前後重なり解除をlatch済みの場合も、すでに成立した横クリアランスを失わないよう継続する。ShiftOut/Return/Recovery、新規pass、明示禁止WP、curve cooldown、EmergencyBrakeでは使わず、locked side、wall clearance、SafetyBrakeは維持する。現行0.5 m / 0.5 mは2025 AWSIMシミュレーション予選だけを対象にした攻めた値であり、実車向け値ではない。設定省略時の既定は`false`、現行dev3 configは`true`である。`output/20260719-223908`のD2ではPass中に`hard_continue=1`を維持した。
- `v2x_side_overtake_entry_rear_tolerance` は、frontではなくside候補から新規追い越しへ入るときのコース進捗後方許容量である。対象の共通コース相対進捗が`-tolerance`未満なら、すでに後方へ抜いた車両を追うShiftOutを開始しない。前方車がなく、この後方side候補しか存在しない場合は`Follow`ではなく`Cruise`とし、後方車へ合わせるV2X縦横制御を生成しない。継続中のlocked Passには適用しない。現行dev3値は0.5 mである。`output/20260721-102131/d3`ではスタート直後から`front_distance=inf`なのに後方side候補だけで`Follow`へ断続的に遷移していたため修正した。
- `v2x_overtake_front_velocity_limit_enabled=true` の場合、active ShiftOut/Pass中にlocked targetとは別の前方車を検出したとき、その車両に通常Followの車間回復capを掛ける。locked target自身のgeneric capは抑止し、Overtake stage speedが縦方向の接近を所有する。required decel / front decel guardはこの設定値に関係なく常に評価し、`false`でも`EmergencyBrake`と停止・低速前車のinside stopping distanceはOvertake判定より先にSafetyBrakeを維持する。
- `LowSpeedAvoidance`: 近距離の低速前方車両に対して通過可能な側がある場合、`v2x_low_speed_avoidance_velocity`を上限として徐行回避する。開始条件では`v2x_low_speed_avoidance_max_front_speed`以下のV2X推定速度を低速車両として扱う。local path開始時は通常MPCへ成立しない横移動を強制せず、横/heading feedback gainによるbounded直接操舵で選択回廊へ入る。直接操舵の速度は、回廊へ入るShift、車列を抜くPass、基準経路へ戻るRejoinに分け、現行dev3ではそれぞれ `v2x_low_speed_avoidance_shift_velocity=3.0 m/s`、`v2x_low_speed_avoidance_pass_control_velocity=6.0 m/s`、`v2x_low_speed_avoidance_rejoin_control_velocity=4.0 m/s` とする。開始後にFSM表示がSafetyBrakeへ変わっても、このlatchと直接操舵ownershipは停止車列clearanceが消え、再合流後のMPC probeが成功するまで維持する。これはgate2のような停止車列回避で、車列途中にMPCへ戻ってOSQP failureと停止を起こさず、Shift用の低速値のまま通過し続ける遅延も避けるための2025 AWSIM向け暫定制御である。
- 直接操舵latchは関連車両列が`v2x_low_speed_avoidance_shift_clear_hold_sec`だけclearになると、開始時のpass targetを待ち続けず、現在のpath constraint内で`e_y=0`に最も近い再合流targetへ切り替える。再合流中に関連車両を再検出した場合は保存したpass targetを復元する。再合流姿勢の成立後もMPCをprobe solveし、成功した周期だけ所有権を渡す。失敗時は通常のsolver failure counterを増やさず直接再合流制御を継続する。これは`output/20260720-164126`のD2で、停止車両rear-clear後も古い`target_ey=-2.61`が残ってlatchが63.4秒継続し、解除直後にOSQPが25周期失敗した事象への修正である。
- LowSpeed direct controlの開始には、以前の異なるsource時刻の観測から有限速度を計算できた同一vehicle IDについて、distinctな受信sampleを`v2x_low_speed_avoidance_stopped_confirmation_samples`回連続して確認する。同一sampleを複数control周期で読んでも加算せず、moving/invalid sample、ID変更、候補消失、`v2x_low_speed_avoidance_stopped_confirmation_max_gap_sec`超過でresetする。現行3 sample / 1.0 sは2025 AWSIM向け暫定値である。
- LowSpeed direct controlがactiveまたはMPC handoff中はOvertakeLineを停止し、開始時に選んだpass targetとsideをdirect controlだけが所有する。操舵は実速度、wheelbase、`v2x_low_speed_avoidance_max_lateral_accel`、最終steering gainから求めた上限をdirect内部と最終publish直前の両方で適用する。phase速度は`v_max`、behaviorの`desired_velocity`、`target_velocity_limit`の最小値を超えない。raw odometry poseと静的grid/footprintを毎周期確認し、geometry/pose/sample無効、map外、物理接触、`v2x_wall_clearance_margin`違反では速度・操舵を0へfail closedする。このwall stopはlegacy Boostを無効化し、加速度low-passを通さず`a_min`を適用する。runtime steering gain更新はdirect内部上限へも同時反映する。
- LowSpeed direct controlの開始前には、raw odometry poseから選択済み`pass_target_ey`側のplanning horizonまでを、実行時wall guardと同じ静的occupancy grid、実車矩形footprint、`v2x_wall_clearance_margin`で掃引検査する。endpoint間もmap resolution以下のcorner motionとなるよう補間し、unknown/occupied、map外、geometry/pose無効はfail closedとする。`v2x_low_speed_pass_side=auto`で最初のsideが不成立なら反対sideを同条件で検証し、成立する場合だけside lockする。実行中のraw-pose wall guardは最終防護として維持する。
- `LowSpeedAvoidance`確定中に実速度が`v2x_low_speed_avoidance_stall_speed`以下で`v2x_low_speed_avoidance_stall_timeout_sec`継続した場合は、局所回避targetを解除し、dangerならSafetyBrake、front/sideありならFollow、それ以外はCruiseへ戻す。`v2x_low_speed_avoidance_stall_cooldown_sec`中は同じ回避への即再進入を抑制する。時刻逆行、非有限値、`v2x_low_speed_avoidance_stall_max_observation_gap_sec`超過では継続時間を加算しない。現行0.15 m/s、1.5 s、3.0 s、0.2 sは2025 AWSIM向けローカル暫定値であり、2026公式値ではない。
- 低速回避では `v2x_low_speed_pass_side` で通過側を `auto` / `left` / `right` から選べる。`right` は reference path 座標系の負の lateral 側、`left` は正の lateral 側である。`auto` の場合は開始前の静的壁preflightまで成立した側を低速回避中のside lockとして使う。direct control開始後は反対側へ切り替えず、live corridorまたはwall guard不成立中は停止する。configured side に通過可能 gap がない場合は逆側へ無理に振らず、Follow / SafetyBrake 側へ倒す。
- `use_v2x_local_path_planner=true` の場合、`LowSpeedAvoidance` は従来の constraint-only gap planner ではなく、停止/低速車両列を reference path の `s/d` 座標へ射影し、選んだ側の「壁と膨張済み車両の間」を通る `target_ey` 列と横制約 `lb/ub` を生成する。この target は全 horizon 点で active になり、障害物が horizon 上で重なる前から MPC の `xr[e_y]` を通過側へ向ける。
- local pathの開始triggerは`v2x_low_speed_avoidance_max_front_speed`以下の車両に限定するが、corridor評価では同じlookahead内の全active V2X車両を膨張済みblockerとして扱う。開始後にtrigger車両が動き出してもside lock中は車列全体の再評価を続ける。また、local path成立時にegoが既にpass corridor内なら、Shift完了待ちでdirect controlを抑止せず`Pass` phaseから起動する。
- gap成立前のlow-speed candidateだけでは、target ID・side・fixed goalを持つ通常Pass Missionを破棄しない。feasibleな`LowSpeedAvoidance`またはactive direct controlが横計画を実際に所有した場合だけhandoffする。direct control中もlive local pathを毎周期再評価し、moving blocker等でcorridorが不成立になった間は速度・操舵を0として旧targetへの走行を止める。再成立時は最新の`pass_target_ey`へ更新して再開する。Rejoinは通過済み車列corridorに依存せず、raw-footprint wall guardとMPC handoff probeを維持する。
- local path planner の target は `v2x_wall_avoidance_bias` を反映する。`0.0` は通路中央、`1.0` は膨張済み車両から `v2x_vehicle_side_target_margin` だけ離れた車両側寄りで、壁側へ膨らみすぎる場合は `0.5` から `1.0` の範囲で上げる。
- local path planner は通過中の horizon 後半で基準 trajectory 側へ戻さない。`LowSpeedAvoidance` が継続している間は選んだ通過側 corridor を制約にも反映し、MPC が反対側をすり抜け候補として選ばないようにする。
- local path planner の通過側 target への入り方は `v2x_low_speed_pass_ramp_ratio` を使う。Gate2では0.2/0.5が初期OSQP failureとなったため1.0を維持し、実際の近距離横移動は低速直接feedbackへ分離する。
- local path planner は `v2x_low_speed_avoidance_lookahead_distance` 内の低速車両を先読みし、選んだ側の通路が車列全体で成立する場合だけ `LowSpeedAvoidance` を許可する。成立しない場合は Follow / SafetyBrake 側へ倒す。
- `v2x_local_path_pass_clearance` は最後の低速車両を抜いた後も通過側 target を保持する距離、`v2x_local_path_return_distance` は基準 trajectory の `e_y=0` へ戻すブレンド距離である。
- `v2x_local_path_invert_target` は local path planner が選んだ `target_ey` を MPC へ渡す直前に反転する切り分け用設定である。RViz 上の通過方向と `e_y` 符号が逆に見える場合の検証に使い、恒久対応では座標系と操舵符号を整理する。
- `v2x_low_speed_pass_ramp_ratio` は、低速回避で通過側 target へ入る速さである。`use_v2x_local_path_planner=true` では停止車両手前の横移動距離を短くし、`false` の旧 gap planner 系では手前の horizon 点にも side-pass target をソフト参照として入れる。
- MPC の操舵レート制約は、前回出力した実操舵から horizon 先頭の操舵にも掛ける。これにより、MPC 表示が「実際にはまだ切れていない操舵」を前提にした進行方向を描くことを避ける。近距離停止車両の横抜けで操舵が遅い場合は `steer_rate_max` を上げるが、実効値は `steering_tire_angle_gain_var` で割った値になる。
- 近距離停止車両が `LowSpeedAvoidance` の距離条件に入っているが、連続した安全 gap が確認できない場合は、通常 `Overtake` へ落とさず `Follow` に倒す。これは回避ラインが確定する前に通常追い越しで横へ振り、停止車両の前を横切って接触することを防ぐためである。
- 停止車両回避の入口候補は、緊急ブレーキ用の狭い車体重複幅ではなく、共通コース進捗上の前方距離と走行可能な横回廊で収集する。これによりカーブや停止車両の横ずれで通常front判定から外れても、横回避を先に計画できる。`SafetyBrake`の重複幅は従来どおり狭いままとし、横に離れた停止車両だけで全停止しない。現行dev3では`v2x_low_speed_avoidance_min_prepare_distance=3.0 m`から入口を許可し、3.0 m未満では開始済み`LowSpeedAvoidance`をholdするか、通常`Overtake`の近距離guardが成立した場合だけそのlineを継続する。通常`Overtake`選択後は、対象が停止中であることだけを理由に`OvertakeLine`を解除しない。この値と挙動は2025 AWSIMシミュレーション予選向けの暫定であり、実車値ではない。
- AWSIM状態を購読できるシミュレーションでは、通常は`/awsim/state=Start`をrace epochとするが、start-grid graceを使う場合は`Ready`からV2X planningを有効にする。`Ready -> Start`は同一planning sessionであり、Startではstuck recovery内部状態とRecovery completeness epochだけを初期化し、V2X車両位置・速度履歴、選択済みpass line、cooldownは保持する。これによりStart直後の最初のV2Xサンプルを速度0として扱わない。一方、Spawned / Grounded / Finishなど真のsession境界では履歴を破棄する。Recoveryはreset後に受信した完全なV2X message/setを従来どおり必要とする。AWSIM状態追跡を使わないlaunchではV2X判定を常時有効とする。
- 低速直接feedbackの解除には、横/heading許容値への収束に加え、front/sideと共通コース進捗上の車列clearanceがすべて消えた状態が`v2x_low_speed_avoidance_shift_clear_hold_sec`継続することを要求する。`v2x_low_speed_avoidance_clear_distance`内の停止車は、横へ避けて通常のfront判定から外れてもclearance車両として追跡する。現行8 m/2秒は2025 AWSIM Gate2向けのローカル暫定値であり、2026公式値ではない。
- 前方車判定は、進行方向前方にあり、かつ `v2x_vehicle_radius + v2x_prediction_margin` の横方向衝突幅に重なる車両だけに限定する。共通コース進捗を使う場合、projectionの`lateral_m`はtargetのコース中心基準座標であってego相対値ではないため、`target lateral - ego e_y`を衝突横距離として使う。projectionまたはego lateralが非finiteなら車両ローカル相対横距離へfallbackする。混走スタートの斜め横車両を前方車として `Follow` に落とすと片方だけ加速が遅れるため、横に並ぶ車両は `side vehicle` として扱う。
- 動いている前走車に対する危険距離は相対速度ベースで判定する。停止車向けの大きな停止距離をそのまま使うと、競り負けた直後に不要な強制減速が入るため、`v2x_moving_safety_brake_distance`、`v2x_moving_safety_brake_margin`、`v2x_moving_safety_brake_time_headway` を別に持つ。ただしV2Xのfront distanceは車体間距離ではなく共通コース上の中心間距離である。moving frontが幾何距離内へ入っただけでは原則0 m/sにせず`RelativeSpeedLimit`を使うが、`v2x_moving_follow_hard_distance`以下は相対速度が小さくてもSafetyBrakeとする。hard distanceを超えたFollowでは距離連動capが前走車より低い速度を要求し、`v2x_moving_follow_target_distance`まで車間を回復する。確立済みShiftOut/Passではgeneric Follow回復capを外し、Overtake stage speedが前走車との接近速度を所有する。EmergencyBrake、front-risk上限、curve上限、front decel guardはこの所有権切替の対象外で、追越中も維持する。hazard holdはmoving frontの非Emergency状態では再armせず、同一対象が観測されclosing speedが0以下なら即解除する。現行dev3のhard 2.05 m、target 5.0 m、recovery 0.6 m/s、gain 1.0、hold 0.25秒は2025 AWSIM向けの攻めた暫定値である。
- `v2x_start_grid_grace_time` は、同時走行スタート直後に静止前走車が `v2x_low_speed_avoidance_max_front_speed` 以下である場合だけ、停止車向けの `LowSpeedAvoidance` と `inside stopping distance` 判定を猶予する。AWSIMのcount開始では車両が `Ready` のまま物理発進し、スタートライン通過後に各車の `Start` が届くため、`Ready` でstatic-grid suppressionをprepared状態にし、V2X planning sessionも有効にする。これにより車両がすでに動いているのに通常trajectoryだけで数秒走る区間を作らず、初期3回廊配置のままbreakoutを選べる。PreparedからStartへは同じplanning sessionを維持し、選択済みlineをStartイベントでresetしない。この準備時間は設定durationへ算入せず、race-session trackerが受理した `/awsim/state=Start` の ROS timeを猶予終了用epochとして、そこから設定秒数後にExpiredへ移る。Prepared/Grace中のfront横判定には通常の`v2x_vehicle_radius + v2x_prediction_margin`だけを使い、ヘアピン用`v2x_front_decel_guard_curve_lateral_margin`は加算しない。これにより隣グリッドを前方衝突車として扱わず、grace終了後は従来のヘアピン横判定へ戻す。重複Ready/Startでは延長しない。Finish / Spawned / Groundedと、Finishを省く `Spawned -> Grounded/Ready -> Start` の手動resetでは前session状態を破棄する。`use_sim_time=false`、V2X車両なし、設定 `0.0` では通常判定を変更しない。現行 `5.0 s` は新規breakoutを開始できる時間であり、すでに同一targetの`ShiftOut`/`Pass`が成立した場合は完遂までそのtargetとsideを保持する。2025由来のローカル暫定値であり、2026公式スタート配置の確定値ではない。
- MultiThreadedExecutorでは、制御周期がROS時刻を取得した直後にV2X callbackが更新されると、同じ周期内のreceipt ageが小さな負値になり得る。現行はsource timestampと同じ50 msまでをcallback順序差としてfresh扱いし、それを超える未来時刻、timeout超過、非有限値はrejectする。この許容はV2X台数、ID、sample、位置jumpの完全性条件を緩和しない。また、Start-grid grace、動的観測、またはbreakout継続中の静止車列は正常な初期配置なので、新規coordinated-stop Recoveryの入口から除外する。開始済みRecovery、衝突・壁証拠、solver fallbackによるRecoveryは抑止しない。
- Prepared開始時に静止車として確認したfront target IDはsession内で1台だけlatchし、その車両の発進速度が `v2x_moving_front_speed_threshold` 以下の立ち上がり中も同じグリッド車として扱う。最初のV2X更新でside分類がまだ揃わないことがあるため、breakout入口はfront targetだけで候補化する。実行可否は全受信車両を膨張して壁境界と照合するgap plannerが判定する。`v2x_start_grid_breakout_enabled=true` では、latchした静止グリッド車に限り、side lock前は左右両側を評価する。片側だけ成立する場合はその側を選び、両側が成立してfront targetのego相対横ずれが`v2x_start_grid_breakout_side_deadband`を超える場合はtargetと反対側を優先する。deadband内ではgap plannerが返した最大corridor幅の広い側を優先し、同幅のときだけnearest frontと壁の幾何関係をtie-breakに使う。選択したsideは同じtargetのbreakout中にlatchし、ShiftOut中の通常の相対横位置変化だけでは再選択・Follow復帰しない。ただし横クリア前にtarget-ordering guardが選択側を拒否した場合はside latchだけを解除し、次の有効な入口で左右を再評価する。既存gap plannerが車両inflation・壁境界・連続gapを満たすside corridorを返した場合、通常の5 m entry guardより先に`Overtake`へ入る。generic gap targetの横加速度評価はcorridor中央への移動量を使い、実行するbounded OvertakeLineと一致しないためbreakout時だけ対象外とする。全車両膨張後gapと実行zoneが同じ周期で成立した時点からdomain/global race速度参照を解放し、内部の`ShiftOut -> Pass`ラベルを待って前車速度+1.2 m/sへ追従しない。targetが横クリアしてgeneric front集合から外れた後も、同じtargetのShiftOut/Pass lineとV2X観測が有効で位置ジャンプがない間はbreakout速度所有を維持し、rear-clearまたはline完了まで通常Passのfront+closing速度へ戻さない。MPCの`a_max`、domain/global上限、wall boundは維持する。OvertakeLineはbreakout入口で選んだ膨張済みego-center corridorの中央を1回だけ固定し、前車がヘアピンへ入った後の相対横位置を追って目標を車両側または壁側へ動かさない。ShiftOut / Pass中はwall clipと横加速度制限を維持し、live vehicle corridorは可否監視に使うが、分断されたpass側intervalをhard clipへ変換しない。breakoutの専用behaviorが決めた速度参照に対し、OvertakeLineは通常のlocked-target front capを二重適用しない。入口で検証したtarget ID、side、OvertakeLineが継続し、target位置ジャンプと明示禁止WPがない間は、ヘアピンで回転したframeによる後続のgap幅・到達性再評価だけではbreakoutを解除しない。この継続中は`latched start-grid breakout continuity`を記録する。横クリア前はahead targetの選択側侵入と永続的なlive corridor消失をRecovery理由にするが、Passが横クリアをlatchした後は回転frame由来の両信号を診断専用とし、rear-clearと明示的hard guardが完了・解除を担う。target変更・位置ジャンプ・明示禁止WP・solver・odometry・非finite入力・control disable・collisionなどのfail-safeも解除要因として維持する。ROS clockが非finiteまたはStart epochより巻き戻った場合は猶予をExpiredとして抑制しない。この積極解放、横目標固定、初期corridorの継続所有は2025 AWSIM dev3シミュレーション予選向けの暫定方針であり、実車値ではない。
- `v2x_start_grid_inter_vehicle_corridor_enabled=true` では、新規breakout時に単一front targetの左右へ先に固定せず、膨張済みの `壁-車 / 車-車 / 車-壁` 自由区間を同じ集合で評価する。スタート車列は縦にずれて同一Frenet断面へ2台が現れないため、各車をそれぞれの最寄りコース線分へ投影した共通Frenet横座標を使い、選択時だけ境界候補2台を`v2x_start_grid_inter_vehicle_longitudinal_span`の長さを持つ横境界として扱う。AWSIMではReady中に物理発進し、domainごとのStart受信時刻がずれるため、`v2x_start_grid_inter_vehicle_lookbehind_distance`以内の側方・直後車も境界候補に残す。これは通常の前方車判定を後方へ広げる設定ではない。攻めたシミュレーション予選用の`v2x_start_grid_inter_vehicle_lateral_radius`は車-車候補だけに適用する。V2Xはtarget yawを持たないため、壁-車候補はego/target矩形の半対角を姿勢不明時の最大横半幅として追加膨張し、ego-wall clearanceもego矩形半対角以上とする。自由区間は車両膨張と壁clearanceを全量適用した後の残余幅でfilter・採点し、旧処理のようにclearanceを狭い区間の半幅へclampして必ず隙間を残さない。通常走行時の車両長、半径、衝突判定は変更しない。`v2x_start_grid_inter_vehicle_min_gap_width`以上の候補から現在`e_y`との横移動量が最小の回廊を選び、同距離なら広い側をtie-breakとする。車-車回廊はその自由区間を作った下側・上側のvehicle IDを保持し、同じ組が`v2x_start_grid_inter_vehicle_min_open_distance`以上連続するときだけ進入する。OvertakeLineは両IDと膨張後回廊中央を固定し、予測断面で片側車両だけが外れた場合も固定中央が自由区間内なら継続する。第三車両または壁で固定中央が閉塞し、既存live-gap hold内に回復しなければRecoveryへ移る。start-grid中は通常の瞬間side-clearance fallbackを使わず、幾何回廊の消失後にtarget相対の新しい固定イン目標へ再進入しない。Returnは両境界車両が`return_clear_distance`後方へ抜けたことを要求し、片側だけを抜いて中央trajectoryへ戻らない。現行0.2 m / 3.0 m / 12.0 m / 4.0 m / 1.25 mは2025 AWSIM dev3用の攻めた暫定値で、2026公式仕様および実車設定ではない。
- `v2x_start_grid_dynamic_decision_enabled=true`では、Ready直後の静止配置で回廊を即時lockしない。P1はbase trajectoryとdomain速度参照で加速を続け、peer速度が`v2x_start_grid_dynamic_peer_motion_speed`へ達してから`v2x_start_grid_dynamic_motion_observation_sec`の間、毎周期3回廊を再評価する。同じstrategy・side・境界IDが`v2x_start_grid_dynamic_candidate_stable_sec`継続すればcommitし、peerが動かない場合も`v2x_start_grid_dynamic_max_observation_sec`で観測を打ち切る。現行値は0.25 m/s / 0.4 s / 0.2 s / 1.25 s。観測中はCruiseとして中央線を維持し、Follow速度制限や横回避目標を入れない。壁-車のinside候補は予測区間の固定offset曲率`kappa/(1-kappa*e_y)`が`tan(delta_max)/wheelbase`以内の場合だけ許可し、不可能ならoutsideを再計画する。車-車weaveは専用rear-clear契約を維持する。これは2025 AWSIM dev3用の暫定race policyである。
- ShiftOut / Pass / Returnの参照経路が入口のphysical execution certificateを通過しても、solverが再計算した状態列は別物として扱う。発行直前に、実solver解の各stage `e_y`を同じwall bounds、同じrequired clearance、raw poseからの`swept-current-to-horizon`車体掃引で検証し、不成立解は制御出力・予測表示・last-feasible cacheへ採用しない。開始後0.05 m以内のShiftOutは未走行entryとして直接Idleへ巻き戻し、不成立sideをretry blockしてfreshな左右探索へ戻す。走行済みShiftOut / PassはDynamicMissionWait、ReturnはRecoveryへ渡して横制御権を失わない。これはsolver failureではないためsolver failure counterを増やさず、直前の安全な操舵と非加速の1周期holdを出す。`Overtake executed solution wall contract`は参照証明、実行解検証、target / generation / phase / side、required clearance、failover actionを同一decision IDで記録する。この契約は2025 AWSIM由来のMPCC一本化前の統合安全条件であり、壁余裕設定を緩和するものではない。
- `LowSpeedAvoidance` に入った後も、最初の `target_ey` を絶対値として固定し続けない。3台以上の停止車列では車両ごとに通れる gap が変わるため、固定するのは通過側だけに留め、horizon 上の障害物に応じて目標を再選択する。
- `SafetyBrake`: required decelがEmergency域に入る、または停止・低速前車が現在速度から見た停止距離内に入っている。gap plannerを使わず`v2x_safety_brake_velocity`に速度制限する。moving frontの非Emergencyな幾何距離超過は`RelativeSpeedLimit`へ分離する。
- SafetyBrake 判定距離は `max(v2x_safety_brake_distance, v^2 / (2 * abs(a_min)) + v2x_safety_brake_margin)` とする。横方向は corridor 全体ではなく、`v2x_vehicle_radius + v2x_prediction_margin` の衝突幅に重なる場合だけ危険判定する。
- `v2x_require_gap_for_overtake=false` の場合は、追い越し禁止条件に入っていなければ gap 幅の事前判定を必須にせず `Overtake` へ遷移する。
- `LowSpeedAvoidance` は `front_distance <= v2x_low_speed_avoidance_distance`、かつ `v2x_low_speed_avoidance_min_gap_width` 以上の gap が `v2x_low_speed_avoidance_min_gap_points` 以上連続する場合に使う。
- `v2x_vehicle_radius` は V2X 車両単体の半幅ではなく、自車中心が入ってはいけない横方向禁止幅として扱う。V2X 車両幅 1.45m の場合、相手半幅 0.725m + 自車半幅 0.725m として 1.45m 程度を基準にし、必要に応じて余白を足す。前後方向の占有判定には `v2x_vehicle_length` と自車長を使う。
- `v2x_overtake_forbidden_wp_ranges` は `[start, end]` の配列で指定し、ヘアピン入口など追い抜き禁止区間の抑制に使う。ただし通常はコース位置固定の禁止より `v2x_overtake_guard_*` の条件で抑制する。
- 既定 `false` のため、通常設定では既存挙動を維持する。

### Dynamic Escape MPCC接続と連続profile（2026-08-22、2025由来の暫定）

`progress_contouring_mpcc_overtake_only=true`でも、OvertakeLineの
`ShiftOut / Pass / Return`に加えて、validated Dynamic Escapeを
contouring-progress MPCCの対象にする。Dynamic Escapeは高位phaseが`Idle`のまま
動的障害物を回避するため、phaseだけでlegacy elapsed-time MPCへ戻してはならない。
progress preparationまたはextended dynamicsが不成立の場合は、既存の3-state progress
MPCCまたはlegacy MPC縮退を維持し、停止へ直結させない。

Dynamic Escapeの到達性bridgeは、各stageを同じ実測横位置から独立に判定せず、
前stageで選択した横位置と終端横速度を次stageへ渡すforward passとする。各segmentの
collision corridorと横加速度reserveの交差内へsoft targetをclipし、horizon全体に一つの
連続target profileが作れない候補はexact tracking QPへ投入しない。hard collision corridor、
壁clearance、footprint preflightは緩和しない。

Dynamic Escapeの`attempt_id`は、planner requestやsolver採否の1周期ではなく、同一targetとの
遭遇を表す。Behavior層のplanner requestは新規遭遇のentry triggerに限定する。同一targetが
relevantな間はentry requestが一時的にfalseへ落ちても、attempt lifecycleが
`planning_requested=true`を維持してGapPlannerと横制御authorityを継続する。
Followは新規entry条件であり、開始済みattemptの周期ごとの必須条件ではない。
target観測が外れた場合も
`v2x_dynamic_obstacle_lateral_escape_attempt_target_loss_grace_sec`以内は同一遭遇として扱う。
grace超過、target変更、race session終了、OvertakeLine Recoveryへの明示移行でattemptを終了する。
ただし、この継続は戦術コンテキストだけを保持するもので、解済みpath、worker result、V2X予測、
wall certificateの有効期限を延長しない。ライフサイクルは
`Dynamic escape attempt lifecycle`ログの`event`、`attempt`、`target`、`reason`、
`planner_requested`、`effective_planning`、`continuation`、`cycles`で追跡する。計画ログの
`lifecycle=entry/plan/continuation/active`でも同じ所有状態を確認できる。

Dynamic Escape実行は、現在周期のfreshなcanonical解と現在周期の物理wall admissionが揃った
場合だけnormal authorityを持つ。旧private retained solution leaseは、その根となるproducerが
legacy normal solve削除後に存在せず、実行不能なconsumer分岐だけを残していたため
2026-08-25に物理削除した。同じ前方targetがblockingでfresh解がない場合も古い解を再生せず、
active lifecycleがfresh candidateを生成する。候補不成立時はcanonical Emergency、solver失敗時は
bounded solver-failure supervisorへ型付きで帰着し、別のnormal formulationやnode-level holdへ
切り替えない。

同日、canonical commandの後段に残っていたsolver／active-overtake／DynamicEscape用の
node-level wall admissionおよびDynamicEscape exit gateも物理削除した。physical wall proofは
canonical producerのcurrent-world certificateと`executed_solution_wall_hold_active`が所有し、
publisherは同じpathを別の距離定義で再評価して操舵・速度を置換しない。Emergency、
bounded solver-failure continuation、Stuck／gear／reverse Recoveryは独立supervisorとして維持する。
`DynamicEscape wall handoff`ログは決定後の観測専用telemetryであり、command authorityを持たない。

現行grace 0.50秒は2025 AWSIM競技
シミュレーション向けの暫定値であり、2026公式仕様または実車安全仕様ではない。

`Overtake decision trace: stage=tracking`は`connected_profile`、`segment_shift`、
`required_ay`、`solver_formulation`、`formulation_source`、`workspace_reset`を出力する。
これにより、幾何候補不成立、profile接続不成立、MPC/MPCC preparation縮退、OSQP数値収束
失敗を区別する。この処理は2025 AWSIM競技シミュレーション向けの暫定であり、
2026公式仕様または実車安全仕様ではない。

### Track/Cruise 5-state MPCC shadow（2026-08-22、移行診断）

`progress_contouring_mpcc_overtake_only=true`の現行権限境界を維持したまま、通常の
Track/Cruise周期でも本番観測、stage geometry、壁bounds、速度参照から5-state
contouring-progress問題を構築し、専用OSQP contextでshadow solveする。これは通常走行の
出力権限を変えない移行診断であり、production commandは引き続き
`legacy-normal-bypass`が所有する。shadow結果は`authority=shadow, selected=0`として記録し、
live solver workspace、circuit breaker、last-certified solution、retained execution、
command post-processingへ渡さない。

shadow warm startはintent、formulation、horizon、schema、stage geometry identityが互換な
forward rolling horizonだけで使用する。不連続なgeometry、intent/schema/horizon変更では専用
context epochを更新する。解はfinite/constraint確認後、既存prediction layoutへ変換し、同じ
wall boundsとraw poseからの車体掃引で物理証明する。metadata/solve/certificate coverage、
build/solve/certificate時間、warm/reset、productionとの差を1秒集約で記録する。

`output/20260822-124134`の単車6周ではmetadata/solve coverageは10,777/10,777、物理証明は
10,147/10,777（94.15%）、shadow総時間は平均3.341 ms、最大15.689 msだった。通常権限への
昇格基準95%へ未達のため、Track/Cruiseの5-state authority昇格は保留する。確認された阻害要因は
次のとおりであり、パラメータではなく共通契約を先に修正する。

- circular trajectoryの末尾が先頭座標を重複しており、raw stage geometryは0 m遷移を持つ一方、
  5-state dynamicsだけがminimum stage distanceへ正規化する。その結果、solverと物理証明が
  異なるhorizon距離を使い、周回継ぎ目で`solution heading unavailable`となる。
- 5-stateのstage-1予測速度をlegacyのtarget-speed input slotへ変換しており、予測状態速度と
  command target speedが同じ数値契約として扱われる。権限移行前にtarget speed、predicted
  velocity、accelerationを分離したactuation proposalを定義する必要がある。
- 上記継ぎ目以外にもhard wall / swept-path不成立解があり、物理証明を外して採用してはならない。

このshadowは診断基盤として維持するが、上記二つの共通契約をテストで固定し、物理証明率を再評価
するまでproduction authorityへ昇格しない。

#### Canonical progress geometry / typed actuation contract（2026-08-22）

5-stateの時間発展が使用する正のstage距離から`EffectiveStageGeometry`を一度だけ構築する。
waypoint遷移の識別子はraw geometryから保持するが、transition distance、cumulative distance、
fingerprintは時間発展と同じeffective distanceで再構築する。progress-contouring 3-stateと
velocity-progress 5-stateはこのidentityをproblem context、warm-start互換判定、物理wall証明で
共有する。legacy 3-stateは従来のraw geometryを使用する。これにより、周回継ぎ目の重複座標を
5-state dynamicsだけが0.005 mへ正規化し、物理証明だけが0 mのまま残る不整合を禁止する。

5-state解の最初の実行sampleは`ActuationProposal`として、stage-1予測速度、u0最適化加速度、
u0曲率、曲率から得たタイヤ角、仮想進捗速度を別々に保持する。抽出処理は有限性とshapeだけを
保証し、物理boundsの判定はQP／certificateの責務とする。既存legacy layoutへの変換は予測表示の
互換用途に限り、stage-1速度をcanonical target-speed commandとは扱わない。

`output/20260822-132440`の単車6回周回継ぎ目走行では、shadow attempt 11,233回のうち
10,995回（97.88%）が物理証明を通過した。残る238回はすべて実際のhard-wallまたは
swept-path判定であり、`solution heading unavailable`とactuation抽出拒否は0回だった。
shadow総時間は平均3.559 ms、最悪1秒窓p99／最大13.503 msだった。一方、production callbackには
25 ms超過が2回あり、実wall不成立も残るため、通常権限への昇格は引き続き保留する。全shadow出力は
`authority=shadow, selected=0`を維持する。

#### Physical wall certificate provenance（2026-08-22）

物理wall証明はboolean結果に加えて、最初に失敗した理由、stage、waypoint、累積距離、横位置、
QP上下限と境界余裕、heading offset、world pose、map/contact、swept path indexを型付き診断として
返す。booleanの合否判定は従来validatorだけが所有し、診断は合否を変更しない。通常時のログ量を
増やさないため、詳細はshadow outcomeの状態遷移時だけ出力し、理由別件数は1秒窓で集約する。

`output/20260822-135649`の単車4周では7,579回の物理検証中7,491回（98.8389%）が合格し、
88回の棄却はhard wall contact 67回、現在poseからhorizonへのswept path violation 21回だった。
invalid input、QP lateral bound violation、heading unavailable、wall sample unavailableは0回だった。
状態遷移時の初回証拠ではQP境界余裕が0.925--1.511 m残ったまま姿勢付き車体が壁へ接触し、
9/10件がstage 0、1件がstage 1だった。したがって残件の主因は数値solverやbound toleranceではなく、
車体中心へ課すscalar Frenet boundsと、2.0 m x 1.45 mのyawed footprintを検証する実行証明の
幾何契約不一致である。swept違反はこれに加え、実現在poseから最初の予測stageへの接続可能性を
QPが直接保証していない可能性を示す。ただし当時の診断はswept path index 0の現在姿勢unsafeと
index 1以降の候補区間unsafeを分離していなかったため、21回すべてを候補経路起因とは扱わない。
次段階では既存証明を弱めず、heading-awareなfootprint-safe
stage boundsをshadow問題へ入力してA/Bする。production authorityへの昇格は引き続き禁止する。

#### Canonical 5-state execution pose（2026-08-22）

velocity-progress 5-state解を物理実行証明へ渡す際は、legacy 3-state予測配列から横位置差分で
姿勢を再構成せず、solverが解いたstage 1..Nの`e_y`、`e_psi`、速度、絶対進捗を
`ExtendedExecutionTrajectory`として保持する。抽出時にshape、有限性、進捗単調性、適用済み
横boundsをstage単位で検証し、失敗stageを診断へ残す。既存production callerは移行中の互換性の
ためheading vectorを空のまま渡し、従来の横profile由来headingを使用する。exact headingの接続は
Track/Cruise shadowだけに限定し、`authority=shadow, selected=0`を維持する。

`output/20260822-142549`の単車4周では7,505回の物理検証中7,427回（98.9607%）が合格し、
5-state pose抽出拒否、heading unavailable、actuation join拒否は0回だった。比較対象
`output/20260822-135649`の98.8389%からは小幅改善したが、hard wall contact 62回、swept-path
violation 16回が残った。したがってheading再構成による情報損失は一因だが、残件の根本原因は
scalar Frenet center boundsがyawed footprintの物理安全を保証しない契約差である。次段階は物理
証明を緩和せず、exact `e_psi`に応じたfootprint-safe stage boundsをshadow問題へ入力し、raw
current poseから最初のstageまでの掃引到達性を別契約として検証する。通常権限への昇格は保留する。

#### Current pose / candidate wall provenance（2026-08-22）

`SweptFromCurrentPose`のpath index 0はproduction controllerが作った
`actual_wall_monitor_pose_`、index 1..Nはshadow MPCCのhorizon stage 0..N-1である。
physical certificateは現在姿勢を同じclearance footprintで先に検査し、sample不能またはcontactを
`current-pose-wall-sample-unavailable` / `current-pose-hard-wall-contact`としてstage -1へ記録する。
現在姿勢が合格した後だけcandidateの離散poseとswept segmentを検査する。swept indexからstageへの
写像は型付きcontractで行い、index 0へ最後のhorizon stage診断が残ることを禁止する。

shadow outcomeの抑制keyはstatusだけでなくphysical certificate reasonを含む。同じ
`physical-certificate-reject`の継続中でも原因がcandidate contact、current pose contact、swept
segmentへ変化した場合は即時に一行記録する。1秒集約ではcandidateの`contact`、production由来の
`current_sample/current_contact`、候補接続区間の`swept`を別々に数える。これは診断順序とprovenance
だけの変更で、certificateのboolean合否、wall margin、trajectory、command、authorityを変更しない。

`output/20260822-164756`の単車約5周では9,630/9,630 solve、9,500 physical certificate
（98.6501%）だった。rejectはcandidate hard contact 54回、current production pose contact 73回、
真のswept segment 3回へ分離され、invalid/bound/heading/sample/current-sampleは0回だった。
現在姿勢が安全だった9,557周期だけを母数にするとcandidate certificateは9,500/9,557
（99.4036%）である。最大solve/shadow時間は11.724/14.186 ms、callback最大29.111 ms、overrun 1回、
shadow selection 0回だった。したがってSlice 3昇格は、candidate側57件の根因解消に加え、legacyが
unsafe current poseを作った状態から新authorityへ安全にhandoffする契約が定まるまで保留する。

physical certificateが証明するworld modelは、controllerが読み込んだ静的occupancy gridに
限られる。AWSIM collider／penalty sourceとの幾何同値性は契約に含まれず、certificate合格は
AWSIM非接触の直接証明ではない。`output/20260823-065700`では、runtime-equivalent samplerが
実測poseを少なくとも1.0 mの検索範囲でclearと判定し、controllerが前進・正加速度を指令したまま、
実速度が`10.089 -> 1.424 m/s`へ`0.120 s`で低下した。したがって、certificateと外部挙動の
境界は`Abrupt measured speed loss:`および、利用可能な環境では
`/aichallenge/pitstop/condition`で相関する。これは診断契約であり、certificateの合否、margin、
solver weight、authority、commandを変更しない。

#### Solved-progress course-frame wall contract（2026-08-22）

5-state解の`e_y` / `e_psi`は固定の`ref_wp_id + stage`へ貼り付けず、同じ解が持つ絶対進捗
`theta`でsamplingしたcourse frameへ適用する。`EffectiveStageGeometry`からworld `x/y/yaw`と
waypoint provenanceを持つstrictly increasingな`CourseFrameKnot`列を構築し、物理wall証明は
各solved progressで補間したframeから車体poseを再構成する。provenanceが非finite、非単調、欠落、
または表現範囲外なら`course-frame-unavailable`としてfail closeし、固定stage frameへfallback
しない。

solverが許容したprogress equality residualにより解がprovenance端点をわずかに越える場合だけ、
同じmetre-domain solver toleranceをsamplingへ渡して端点へclampする。別の固定epsilonで証明範囲を
広げない。QPのscalar lateral boundは現段階ではcentre-path contractであり、2.0 m x 1.45 mの
oriented footprintと設定wall marginを確認するexact certificateを置き換えない。現在production pose
から最初のsolved poseまでのswept proofも引き続き必須である。

`output/20260822-173527`では、candidate hard contact時に固定stage frameがsolved progressより
1.0--1.9 m先行していることを確認した。修正後の`output/20260822-181304`では4,794/4,794 solve、
4,782 certificate（99.7497%）となり、candidate discrete hard contactとcourse-frame unavailableは
ともに0件だった。残件はunsafeなlegacy production pose 10件と、current poseからfirst stageへの
真正なswept failure 2件である。Track/Cruise出力は引き続き`authority=shadow, selected=0`とし、
first-stage reachabilityとunsafe current poseからのhandoff契約を解決するまで昇格しない。

#### Track/Cruise canonical production authority（2026-08-23、2025由来の暫定）

Gate Aのfresh canonical実行証明とGate Bのcurrent-world retained再証明を取得したため、既存の
`progress_contouring_mpcc_overtake_only=true`移行境界におけるTrack/Cruise通常出力を5-state
canonical MPCCへ昇格する。通常権限の選択順は、当周期の物理証明済みfresh plan、現在pose・現在
course frame・現在の空障害物観測で再証明したretained plan、明示Emergency Stopだけである。
eligibleなTrack/Cruise周期ではextended解をlegacy 3-state配列へ変換せず、3-state/legacy solve、
extended handoff smoothing、solver crawl、bounded continuationを通常fallbackとして使用しない。
Follow、Hold、Stop、Overtake、RecoveryはこのSliceの対象外で、既存経路を維持する。

canonical commandは予測速度、最適化加速度、曲率、タイヤ角、仮想進捗速度を別々に保持する。
callbackは通常canonical出力に速度誤差由来の加速度再計算、加速度low-pass、操舵low-pass、操舵rate
limiterを重ねない。設定済み物理上下限によって値が変わる場合はcanonical labelを維持してclipせず
fail closeする。5-stateモデルと物理証明が使用したタイヤ角は、canonical normalおよびcanonical
Emergency Stopでは`steering_tire_angle_gain_var`を再適用せず、そのままpublisherへ渡す。legacy
normal pathだけは移行期間中の既存actuator calibrationを維持する。Stuck Recoveryはcanonical
command全体を置換できる上位supervisorとして残し、Recovery側の既存操舵規約を使用する。

final execution traceはproblem/solution fingerprintに加え、execution plan ID、当周期の
execution-certificate decision ID、fresh/retained sourceを保持する。retained planの元solve decision
と、現在世界で実行を再証明したdecisionは別IDとして扱い、後者がpublish周期と一致しなければ
canonical normal sourceとして記録しない。fresh planはcommand・predictionまで構築できた後だけ
atomic retained storeを置換し、後段拒否で直前の実行可能planを破壊しない。

この昇格はTrack/Cruise限定のSliceであり、パラメータ調整や新規fallback追加ではない。動的受入では
6周反復、fresh/retained/Emergency source、legacy normal source 0回、formulation switch 0回、
callback overrun、wall/contact、Recoveryを確認してから次のintentへ進む。

`output/20260823-062054`と`output/20260823-064933`では各3周を完走し、canonical normalとEmergency
Stopのraw/published操舵角一致、最終actuation差0、legacy normal source 0を確認した。後続の6周試験
`output/20260823-065700`は1周後のwp53付近でRecoveryより前に衝突したが、これをlegacy gain欠落と
みなしてcanonicalへ1.5倍補正を適用した`output/20260823-072038`は、最初の周回のwp77--113で
操舵反転が不安定になり、約8.03 m/sから0.37 m/sへ急減速して壁接触した。このcounter-hypothesisは
棄却し、元の6周衝突は別原因として扱う。一方、単位が
混在するQPでOSQPのglobal terminationを通過しても、実行に使う曲率・加速度・予測速度の行が固有
toleranceを超える周期が残る。その周期は後段でfail closeしており、安全証明を緩めて採用しては
ならない。全constraint rowの一括scaleは`output/20260823-063519`で収束率0%へ退行したため撤回済み
である。この数値定式化は別Sliceでfailure-firstに扱い、本Sliceへsolver設定調整やfallbackを混ぜない。

#### Overtake entryのcanonical速度証明（2026-08-24、2025由来の暫定）

ShiftOutへ入るprogressive entryの最低速度判定は、選択済みfive-state physical execution
certificateが完全な場合、そのexact trajectoryの全stage速度の最小値を使用する。先行する幾何Missionの
kinematic rollout速度を、別のfive-state解が選択された後も実行証明として再利用してはならない。
exact certificateが存在しないproducerは従来どおりMission rollout速度でfail closeする。非finite、
不完全またはcertificate未成立のexact trajectoryを新しい証明として扱わない。

速度証明の選択は最低速度比較だけを所有する。close-entry completion proof、target provenance、body
clear、wall、freshness、current-world revalidationは独立した必須Gateとして維持する。
`output/20260824-200419`では、設定変更なしで10.02 mのentryが
`speed_proof=certified-execution`を記録し、`Idle -> ShiftOut`とcanonical five-state
`contract_join=1`まで到達した。Pass/Returnと衝突なしの実用品質はこの証明の合格に含めない。

#### Rejoin canonical production authority（2026-08-24、2025由来の暫定）

OvertakeLineの`Recovery`から通常ラインへ戻る`ControlIntent::Rejoin`は、専用solver context、
warm-start identity、plan storeを持つvelocity-progress 5-state MPCCだけを通常出力権限とする。
当周期のfresh解が数値・物理・canonical artifact証明をすべて満たす場合だけpublishし、証明できない
周期は明示Emergency Stopへ移る。Rejoinにはcurrent-world semanticsを定義したretained証明がまだ
存在しないため、plan ageだけで直前解を再利用しない。legacy 3-state normal solverへのfallthroughは
削除し、EmergencyおよびStuck/gear/reverse Recoveryは独立した上位authorityとして維持する。

5-state問題の固定state zeroはdelay補償後の実行poseである。そのstateから出る最初のaffine dynamics
だけは、同じ実測Frenet lateral/lag/heading、実測速度、現在到達可能な曲率で線形化する。desired path
とdesired velocityで最初の遷移を線形化すると、固定stateと異なる接点からstage 1を予測し、QP可行でも
exact footprint証明と実行commandが一致しない。stage時間はlinearization anchorから逆算せず、既存の
immutable stage-geometry scheduleを正本とする。stage 1以降は従来どおりnominal trajectoryを接点にする。

修正前の`output/20260824-191213`では、最初の1周期だけmaximum iterationsへ到達した後、33/33周期が
fresh physical certificateを通過した。production昇格後の`output/20260824-192226`では83/83周期がsolveし、
69周期がfresh canonical、14周期がhard-wallまたはswept-wall不成立としてfail closeした。最終decision
traceでRejoinのlegacy normal sourceとcontract join failureは0、Recoveryからの離脱は1回確認した。
したがってsingle-authority構造は合格とするが、物理reject期間のEmergency率と25 ms callback超過は
Overtake実用品質の残課題であり、wall margin緩和やretained fallbackで隠してはならない。

#### Steering-rate 6-state certified plan store（2026-08-25、移行shadow）

操舵角を状態、操舵レートを入力として解くrate-resolved 6-state MPCCでは、数値解の
`ExecutionArtifact`と、同じartifactをexact course frameへ復元した物理wall証明を、別々の
latest-resultから後段で推測結合してはならない。既存の単一直列worker内でsolve、artifact抽出、
物理証明を完了し、物理結果が`Accepted`かつartifact identity、decision、problem fingerprint、
stage geometry、intent、snapshot時刻が完全一致する場合だけ、1個のimmutable `CertifiedPlan`を
構築する。

`mpcc_rate_resolved_certified_plan::Store`はartifact sequenceに対して単調なall-or-nothing置換を行う。invalid／staleな
置換は直前のaccepted planを破壊しない。これは将来のsame-formulation retained pathの証拠保管で
あり、元のpose snapshotとcourse-frame windowに対する証明を後の制御周期へ延命するものではない。
retained実行には、exact cursor、現在intent、現在poseからremaining horizonへのconnector、現在の
wall／obstacle worldを別のdecision identityで再証明する必要がある。

本段階は`authority=shadow, selected=0`であり、5-state Track/Cruise publisher、Emergency、Recovery、
solver設定、wall margin、制御parameterを変更しない。6-state production昇格はcurrent-world retained
admissionの動的証拠と、旧5-state通常ownerを同じSliceで削除できる条件が揃うまで禁止する。

#### Steering-rate 6-state retained current-world shadow（2026-08-25、移行診断）

`CertifiedPlan`は、accepted physical resultだけでなく、その結果を生成したimmutableなwall-grid
shared owner、footprint、course-frame knots、wall clearance、sampling policyを含むexact physical
snapshotを保持する。artifact、snapshot、resultのidentityが完全一致しない組み合わせはstoreへ入れない。

retained consumerは元のsuffixを制御callbackで再走査しない。元suffixの静的wall証明を同一snapshotに
束縛したまま、現在周期について次だけをshadowで再証明する。

- Track/Cruise intentとexact time cursor
- circular course progressの同一branchへのlift
- 前回publish済み操舵から1 publication intervalで到達可能な操舵actuation
- 観測時刻の現在速度からcontrol originまでの実delayで到達可能な予測速度
- measured-to-control delay pathと、current control poseからretained expected poseへのconnector
- 同一wall-grid owner・footprint
- currentかつ明示的にemptyなV2X observation

出力はproofと拒否理由だけで、command、publisher、production candidateを持たない。`make dev2`の
`output/20260825-061048`では、retained判定はおおむね0.01--0.02 ms、観測最大値1.071 msで、
新規判定に起因する連続callback overrunは確認されなかった。一方、全周期が
`dynamic-observation-unavailable`でfail closeした。単車`output/20260825-061340`でも同様であり、
現行V2X producerは「対象車両が存在しない」というcurrent empty messageを供給しない場合がある。
未受信をemptyと推定してretained authorityを昇格してはならない。6-state production昇格前に、
dynamic worldのexplicit-empty契約、または対象車両を含むcurrent obstacle tube証明を別Sliceで定義し、
動的Acceptanceを取得する必要がある。

#### Steering-rate 6-state retained dynamic-world shadow（2026-08-25、移行診断）

後続Sliceでは、`active_vehicle_count == 0`というempty-world proxyをretained admissionから除いた。
`V2XGapPlanner`は最新V2X arrayのmessage identity、timestamp、vehicle ID集合、各vehicleの状態を
1回のmutex acquisitionでsnapshot化する。tracking mapに残る別世代vehicleを混ぜず、stale message、
position jump、invalid velocity、motion estimate未成立はfail closeとする。NoDataをempty worldとは
推定しない。

snapshot内の各peerは、既存V2X policyのvehicle radius、prediction margin、position covarianceで
保守的なmoving circleへ膨張する。observation時刻からcurrent control時刻まで線形予測した状態を
基準に、次を同じworld座標と時刻軸で再証明する。

- measured-to-control delay pathとcurrent poseからretained expected poseへのconnector
- exact cursorから残る全control stage（immutableなstage durationを使用）
- 各sampleにおけるoriented ego footprintと全peer circleのsigned clearance

空間sample数はego平行移動、ego cornerのyaw sweep、peer移動量の上界から決定する。現在または
将来のoverlapは`dynamic-path-blocked`、入力や再構成不能は`dynamic-path-invalid`として分離し、
最初に確定した物理block理由を後続segmentで上書きしない。

`output/20260825-064109`の`make dev2`では、peerが存在してもclearなsuffixを両domainでacceptし、
d1がd2へ接近した区間は`blocked_by=d2`、最小signed clearance `-0.006 m`としてrejectした。
判定時間は通常約0.04--0.23 ms、観測最大1.337 msだった。結果は引き続き
`authority=shadow, selected=0`である。これによりdynamic Acceptanceの証拠は得たが、production昇格は
Track/Cruise retained authority接続と旧5-state通常owner削除を同一Sliceで行うまで禁止する。

#### Steering-rate 6-state causal command shadow（2026-08-25、移行診断）

rate-resolved worker requestを同期5-state評価の途中でsubmitすると、そのrequestのstate-zero steeringは
当該周期のproduction出力が確定する前の`previous_steering`になる。その後5-state commandまたは
EmergencyStopが別の操舵をpublishすると、非同期artifactが内部的に正しくても、実際の制御履歴から
到達不能になる。この因果不整合を、solver後のclampやartifact補正で隠してはならない。

Track/Cruise周期の順序を次へ固定した。

1. 5-state候補を評価し、rate-resolved request draftをsealする。
2. production出力を解決し、実際にpublishする操舵をcommitted historyへ反映する。
3. draftのstate-zero steeringを、そのcommitted predecessorへbindする。
4. 完全なsource problem contextとともにimmutable snapshotをworkerへsubmitする。

accepted retained proofからは、5-state専用`CanonicalNormalCommand`を流用せず、別型の
`RateResolvedCommandCandidate`を構築する。candidateはcurrent decision、artifact sequence、source
decision、problem fingerprint、stage geometry、intent、stage index、速度、加速度、操舵レート、操舵角、
曲率、virtual progressを保持し、identity欠損または非有限actuationを拒否する。publisherとの接続は
持たず、production 5-state commandとの差分観測だけを行う。

`output/20260825-072127`の`make dev2`では、両domainでcandidateを生成し、全件が
`authority=shadow, selected=0`だった。fresh workerのdecisionとcausal submissionを対応付けた結果、
state-zero steeringとcommitted predecessorの最大表示差はd1で0.00004876 rad、d2で
0.000049244 radであり、predecessorを小数4桁で出力するログ丸め範囲内だった。control callbackの
25 ms超過は両domainとも0で、観測最大は12.4 ms未満だった。

このSliceはcommand provenanceを閉じたが、production昇格ではない。measured-to-control connectorと
retained suffix originの時刻意味を1つに定義し、fresh／retained／worker replacement／rejectの
動的Acceptanceを得た後、6-state owner接続と5-state Track/Cruise owner削除を同一Sliceで行う。

#### Steering-rate 6-state control time origin（2026-08-25、移行診断）

5-state MPCのstate zeroは、観測poseそのものではなく
`state_prediction_delay_sec`後の実行poseである。したがって6-state artifact、retained cursor、
measured-to-control prefix、動的障害物予測は、callback観測時刻とcontrol-effective時刻を区別する。

- `observation_sec`はV2X freshnessと生観測の時刻。
- `control_origin_sec = observation_sec + state_prediction_delay_sec`はsolver state zeroが表す時刻。
- artifactの`prediction_origin_sec`は`control_origin_sec`と一致する。
- async solveのcompletionは未来のcontrol originより早くてもよく、capture時刻以降であることを要求する。
- retained cursorは現在周期の`control_origin_sec`で解決する。

measured poseからcontrol poseまでのprefixは、poseと単調なelapsed timeを同じ型で保持する。動的証明は
peerを時刻0へ固定せず、prefixを`0..delay`、control poseからretained expected poseへのconnectorを
`delay`、retained suffixを`delay`以降として1本の連続時刻軸で評価する。pose/time個数不一致、開始時刻
非0、終端時刻とdelayの不一致、時刻逆行はfail closeする。

`output/20260825-074847`の`make dev2`では、両domainでcontrol delayが一貫して`0.130000 s`となり、
d2最終窓はsolve 81/81、retained current-world 81/81、command candidate 81/81だった。すべて
`authority=shadow, selected=0`を維持した。約208秒の走行中、d2で25 ms周期を1回だけ1.559 ms超過したが、
連続overrunはなく、最終窓のsolver最大は2.281 ms、retained proof最大は0.260 msだった。この単発超過は
production昇格Sliceの複数周timing gateへ残す。

本Sliceは時刻producerの不変条件を修復したものであり、delay margin、age lease、fallback、normal
authorityを追加していない。次のproduction昇格は6-state Track/Cruise owner接続と5-state通常owner削除を
同じSliceで行い、二重authorityを恒久化しない。

この時刻契約はactuation種別ごとのpredecessor provenanceにも適用する。操舵角は直前にpublishした
command履歴から次のpublicationまでの到達可能性を検査する。一方、`current_speed_mps`はcallbackの
観測時刻、retained artifactの`predicted_speed_mps`はcurrent control originを表すため、その速度差は
`control_origin_sec - observation_sec`で検査する。両者へ同じpublication intervalを使わない。

`output/20260825-084721`では、この修正前後でd1の最初の404 retained周期を直接比較できた。修正前
`output/20260825-081954`はvelocity reject 137、accepted 236だったのに対し、修正後はvelocity reject
0、accepted 382だった。d2も修正前178件、修正後0件となった。動的path blockはd1で17件、d2で95件
として独立して残り、速度判定の緩和で障害物証明を迂回していない。結果は全件
`authority=shadow, selected=0`で、両domainのcallback overrunは0だった。

#### Steering-rate 6-state canonical identity（2026-08-25、移行shadow）

rate-resolved solver requestは6-state／3-inputである一方、従来はそのsource
`MpccProblemContext`を`VelocityProgress5State`としてsealし、command candidate内の別enumで
6-stateと表示していた。この状態ではproblem fingerprint、solver artifact、physical certificate、
retained proof、commandが同じformulationを証明できないため、production authorityへ昇格できない。

共有execution contractへ`VelocitySteeringProgress6State`を追加し、専用のstate／input／bounds／cost
schemaで独立contextをsealする。artifact identityがformulationを所有し、physical、retained、commandは
その完全identityをコピー・比較する。commandだけが持っていた重複formulation enumは削除する。
unresolvedまたは5-state identityを持つrate-resolved artifactはfail closeする。

`output/20260825-081954`の`make dev2`では、両domainで
`formulation:velocity-steering-progress-6state`のcandidateを観測し、artifact／mailbox identity rejectは0、
全件`authority=shadow, selected=0`だった。d2最終窓はsolve 81/81に対してcommand candidate 73/81であり、
identity不整合は解消したがretained admissionの非連続性は残る。したがって本Sliceはidentity Gateのみ
合格とし、availability holeの原因を閉じる前にpublisherへ接続してはならない。昇格時は6-state owner
接続と5-state Track/Cruise owner削除を同一Sliceで行い、cross-formulation normal fallbackを作らない。

#### Steering-rate 6-state source-context provenance（2026-08-25、移行shadow）

非同期solverへ渡した完全なsealed `MpccProblemContext`は、artifact以降でも同じsource identityとして
保持する。problem fingerprint、decision、stage geometry、intent、formulationだけを別々に複製しては
ならない。複製された断片からはstate／input／bounds／cost schema、horizon、generation、target、sideを
再検証できず、retained実行時に現在周期のcontextで補うとsolve元identityを偽装するためである。

rate-resolvedのexecution artifact、physical proof、retained proof、command candidateは、一つの完全な
`source_context`を不変のまま渡す。retained proofの`decision_id`は現在worldに対する実行証明であり、
source contextのsolver decisionとは別の責務を持つ。どちらかで他方を上書きしない。

`output/20260825-091930`の短時間`make dev2`では、両domainでavailable candidateがすべて
`VelocitySteeringProgress6State`、artifact／mailbox identity rejectが0、physical identity mismatchが0、
callback overrunが0だった。全件`authority=shadow, selected=0`を維持しており、本変更はproduction
authorityを増やしていない。次の昇格Sliceでは、この完全source contextをfinal decision traceへ渡し、
6-state Track/Cruise owner接続と5-state owner削除を同じ変更で行う。

#### Steering-rate 6-state Track/Cruise production authority（2026-08-25、2025由来の暫定）

Track/Cruiseの通常出力ownerを、current-worldで再証明したrate-resolved 6-state retained proofへ
切り替えた。Track/Cruise周期は5-state通常solverを呼ばず、次周期用6-state requestを独立してsealし、
当該周期で実際に選択した操舵角をstate-zero predecessorとしてbindする。6-state proofがない周期は
別のnormal formulationへfallbackせず、明示的なcanonical EmergencyStopとする。Follow、Overtake、
Emergency、Recoveryのauthorityは本Sliceで変更しない。

初回production run `output/20260825-094733`では、OSQPの既存physical residual tolerance内で
加速度が上限を微小超過し、publisherの無条件clampが証明済みactuationを変更したため、最終identity
guardが`canonical normal command mutated before publication`として拒否した。これはparameter不足では
なく、residualを許容するsolver rowとexactなphysical publication boundaryの所有不整合である。

velocity、acceleration、steering-rate、virtual-progressのsolver-facing physical rowは、既存の
physical OSQP toleranceから導く最大許容残差だけ内側へ移す。元のphysical boundsは変更せずartifactへ
保存し、execution artifactでexactに再検査する。canonical publisherはcertified actuationをclamp／補正
しない。区間がsolver insetを収容できない場合はproblem constructionでfail closeする。Recoveryと
noncanonical出力の既存clampは維持する。

修正後の`output/20260825-100454`では、両domainでpublisher mutationは0、d2は複数窓で
`attempted=81/available=81/production_canonical=81`、speed／acceleration／steering差分は全て0となった。
最終traceは`VelocitySteeringProgress6State`、`authority=certified-normal-solution`、
`canonical=satisfied`を保持した。したがってTrack/Cruiseの6-state authority migrationは構造上合格と
する。動的world observation欠損はage-only再利用で隠さず、provenance付きuncertainty tubeの後続課題と
する。traffic中のcallback overrunも独立したtiming課題として残す。到達不能になった5-state
Track/Cruise helperの物理削除はSlice 6で行う。

#### 5-state Track/Cruise ownerの物理削除（2026-08-25、2025由来の暫定）

6-state Track/Cruise production authorityの動的受入後、到達不能だった5-state Track/Cruiseの
retained evaluator、plan store、solver context、warm-start identity、mode switch、telemetryを
Slice 6で物理削除した。互換flagやcross-formulation fallbackとして残していない。

従来Track/CruiseとRejoinで共有していた5-state evaluatorはRejoin専用責務へ縮小した。したがって
通常Track/Cruiseのownerは引き続き`VelocitySteeringProgress6State`または明示的Emergencyだけであり、
Rejoinの既存5-state canonical behaviorは本Sliceの対象外として維持する。source-contract testは
退役済みTrack/Cruise mode、store、solver、retained evaluatorがproduction sourceへ戻ることを禁止する。

本変更は到達不能コードの削除であり、parameter、timeout、ROS interface、Recovery policy、live
rate-resolved workerを変更しない。25-package build、49 test target、1,869 testが合格した。動的挙動の
基準はauthority昇格時の`output/20260825-100454`を継続使用する。Follow、Overtake、Rejoinおよび
残存legacy／3-state経路の削除は別Sliceで扱い、Slice 6完了前のparameter tuningは禁止する。

#### Normal dispatchのlegacy／3-state fallthrough削除（2026-08-25、2025由来の暫定）

`MPC::get_control()`のnormal dispatchは、解決済みintentをcanonical ownerまたは明示的Emergencyへ
必ず帰着させる。Track／Cruise、Follow、ShiftOut／Pass／Return、Rejoin、Stopの分岐後に残っていた
synchronous extended 5-state solve、progress 3-state fallback、legacy spatial MPC fallbackと、その
command postprocess経路を物理削除した。

Track／CruiseとRejoinはmigration eligibility booleanではなくcontrol intentでownerを選ぶ。eligibilityは
選択済みowner内部のadmission proofであり、不成立時は同じcontractのEmergencyとなる。したがって
configやmetadata不成立によって別formulationへ暗黙移行しない。unsupported／Unknown intentも
`canonical normal intent has no production owner`としてfail closeする。

旧`solve_problem()`、private persistent solver、warm-start、age、3-state formulation historyも削除した。
dynamic-escape追跡ログは旧`progress-3state`／`legacy-mpc`の推測値ではなく、実際のcanonical problem
contextのformulationを記録する。25-package build、49 test target、1,870 testが合格した。retained
legacy dynamic-escape execution、migration-only circuit／reentry telemetry、node-level wall handoffの
物理削除は、final publisher／Recovery責務との境界を監査した後続Sliceで完了した。

#### Node-level normal wall handoffの物理削除（2026-08-25、2025由来の暫定）

全normal intentのcanonical MPCC昇格後もpublisher側に残っていた`WallPathAdmissionGate`、
`LegacyWallHandoffAuthority`、DynamicEscape exit gateと、solver／Overtake／DynamicEscape用hold出力を
Slice 6で物理削除した。ActiveOvertakeとDynamicEscape gateはcanonical dispatch後に到達不能であり、
solver recovery gateだけは到達可能だったが、fresh canonical commandのcurrent-world wall certificateを
別のnormal ownerが再解釈して置換する二重authorityだった。

canonical current-world physical wall proof、executed-solution wall hold、明示的Emergency、bounded
solver-failure continuation、Stuck／gear／reverse Recovery、観測専用wall traceは維持する。parameter、
margin、timeout、solver設定、ROS interfaceは変更していない。25-package build、49 test target、1,840 testが
合格し、`output/20260825-124515`では旧wall-handoff source／traceは両domainとも0件、canonical normal
publicationとcomplete execution identityは継続した。既存のasync ShiftOut候補未準備によるcanonical
Emergencyは別の実用品質課題であり、本削除Sliceへ対症処理を混在させない。

#### LowSpeedDirect通常authorityの物理削除（2026-08-25、2025由来の暫定）

Slice 4でproduction authorityを退役させた後も残っていた停止車両用direct controllerを、Slice 6で
物理削除した。監査時点で`low_speed_shift_control()`は定義1件・call site 0件であり、active latchを
trueへ設定できる到達可能producerも存在しなかった。このため、private phase/latch、direct
retained-pass／rejoin、publisher override、`LowSpeedDirect`／`LowSpeedWallStop` final source、direct用
execution formulationおよび専用YAML keyは安全機能ではなく、到達不能な第二normal authority表現だった。

停止／極低速V2X車両の確認、`LowSpeedAvoidance` intent、gap／local-path生成、static-wall preflight、
canonical local-corridor speed reference、bounded solver-failure crawl用の横feedbackは維持する。したがって
停止車両回避は引き続き、障害物と壁から構成したbounds／reference／speed windowをcanonical MPCCへ
渡し、certified canonical commandとして出力する。外部EmergencyとStuck／gear／reverse Recoveryも
変更しない。

failure-first source contract、25-package build、49 test target、1,822 testが合格した。authority退役時の
決定論的replayでは`LowSpeedDirect` publication 0件とcanonical Dynamic Escape継続を確認済みであり、
本Sliceは到達不能コードだけの削除なので重複する動的replayは行っていない。wall margin、solver
tolerance、horizon、weight、timeoutおよび到達可能なbehavior parameterの調整は行っていない。

#### 旧3-state normal formulation表現の物理削除（2026-08-25、2025由来の暫定）

normal dispatchの3-state fallback削除後もexecution contractに残っていた
`LegacySpatialMpc3State`／`ProgressContouring3State`と、5-state解を旧3×2配列へ変換する
`convert_extended_solution_to_legacy()`をSlice 6で物理削除した。前二者はproduction producer 0件で
enum／文字列表現／到達不能schema switchだけ、converterはproduction call site 0件で専用testだけが
consumerだった。

非canonical formulationをfail closeするcontract testは、現存する例外用`SolverDerivedBypass`を入力に
使うため、退役済みnormal formulationをテストのためだけに再表現しない。canonical formulation集合は
5-state `VelocityProgress5State`と6-state `VelocitySteeringProgress6State`、例外表現は
`SolverDerivedBypass`である。`Unresolved`は未解決identityを表すがnormal command ownerではない。

failure-first source contract、25-package build、49 test target、1,821 testが合格した。到達可能なsolver、
Rejoin、publisher、Emergency、Recoveryを変更しておらず、parameter tuningも行っていないため、新規の
動的replayは要求しない。

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
