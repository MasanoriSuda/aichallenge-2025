# MPC Stuck Recovery Design

作成日: 2026-07-12
更新日: 2026-07-13
状態: Implementation Complete / 2026-07-13 AWSIM Verification Pending

## 方針

通常走行は現行MPCへ任せ、スタック時だけRecoverySupervisorへ制御権を移す。
MPCの速度入力を負にする改造は行わない。

論理的にはRecoveryを独立責務とするが、最初から別ROS nodeとgeneric command muxを追加しない。
初期実装ではpure C++ coreをmpc_controller_cpp node内へ統合し、
/control/command/control_cmdの最終publisherを現在の1か所に維持する。

導入は次の順序とする。2026-07-12時点では公式gear契約の文書化、2〜3のコードと
pure test、4のpure footprint / rollout API、AWSIM単車でのgear / acceleration校正、
P1 / P2のSIM限定有効化まで実装済みである。正面壁スタックを用いたend-to-end確認は
未完了である。

1. 公式gear契約とAWSIM実挙動の確認。
2. 制御を変えないShadow detector。
3. SIM限定の直進後退MVP。
4. runtime footprint checkerと3後退プリミティブ。2026-07-13にStraight / Left / Rightを
   決定的な優先順で実制御へ統合した。
5. 実測で不足が確認された場合だけ二段切返し以降を検討。

## 実装結果の概要

- 設定は未列挙Domainの`enabled: false`を既定とし、P1 / P2だけ
  `domain_enabled=true`、`shadow_mode: false`、`simulation_only: true`である。
  P3と実車は無効である。
- AWSIM実測に基づき、Reverse駆動は正加速度、停止は負加速度とする。
  `reverse_acceleration_sign=+1`、駆動`+0.5 m/s^2`、停止`-0.8 m/s^2`、
  保守的停止減速度`0.4 m/s^2`、制御遅延予約`0.2 s`を使用する。
- 3台SIMのV2Xは自車を含まない2 entryだったため、P1 / P2検証では
  `expected_v2x_vehicle_count=2`、`self_filter_mode=excluded`とする。
- Shadowはdetector、static footprint、V2X / Boost完全性、FSM candidateのログを
  出すが、control / gear commandを上書きしない。
- Active時は選択primitiveのstatic swept footprint、fresh V2X rear corridor、freshかつ
  inactiveなBoost、freshで一致したGearReportのhard conditionを順に満たす。
- Recoveryがcommand ownerの周期は通常MPCの`u / acceleration`を破棄し、
  同じC++ nodeの既存publisherからRecoveryまたはSafeStopだけをpublishする。
- Front / Side後退はepisode実測2.0 mを必須とし、停止予約込み最大3.0 m / 4.0 s /
  0.8 m/s / 2 attemptsとする。速度上限到達時はReverseのまま減速して再開する。
- Left / Rightの後退自転車model、static safety、runtime候補選択、実操舵commandを実装した。
  RViz候補表示と終端rejoin scoreは未実装である。
- 正面壁スタックからLowSpeedRejoinまでのend-to-end実走は未検証である。

## 現行構成と問題

現行フロー:

    Odometry / trajectory / V2X
              |
              v
        MpcControllerCpp
              |
              v
          Spatial MPC
        speed lower = 0
              |
              v
      AckermannControlCommand
              |
              v
    /control/command/control_cmd

正面が壁へ押し付けられた場合、前進可能な入力がなくてもMPCは後退を生成できない。
solver fallbackとfail-safeも減速停止のみである。

既存のcollision heuristicは次の理由でRecovery triggerの正本にできない。

- /aichallenge/pitstop/conditionは2026公式collision topicではない。
- condition差分は壁方向、接触位置、後方安全を表さない。
- 現行コードではis_collidingを計算後に使用していない。
- 2026シミュレーターではrepair item自体が未実装と記載されている。

## 変更後の論理構成

    Odometry / race state / gear report / V2X / static map
                            |
                            v
                 node-side safety adapter <----- RecoveryFootprint pure API
                 |- Straight footprint            |- Straight (runtime)
                 |- fresh V2X rear corridor        |- Left (evaluation only)
                 `- fresh Boost inactive           `- Right (evaluation only)
                            |
                            v
                 StuckRecoveryCore
                 |- StuckDetector
                 `- RecoverySupervisor
                            |
                            v
                  Recovery / SafeStop
                            |
        Normal MPC ---------+
                            v
                 FinalCommandArbitrator
                 |- one control publisher
                 |- one gear publisher
                 `- Boost inhibit
                            |
             +--------------+--------------+
             |                             |
             v                             v
       /control/command/             /control/command/
          control_cmd                    gear_cmd

Recovery coreはNormal MPCの内部最適化へ侵入しない。
FinalCommandArbitratorは1周期にNormalまたはRecoveryのどちらか一方だけをpublishする。

## 候補コンポーネント

実装済みファイル:

    include/multi_purpose_mpc_ros/stuck_recovery_core.hpp
    src/stuck_recovery_core.cpp
    test/test_stuck_recovery_core.cpp

    include/multi_purpose_mpc_ros/recovery_footprint.hpp
    src/recovery_footprint.cpp
    test/test_recovery_footprint.cpp

ROS adapter、map adapter、V2X adapter、command publishはmpc_controller_cpp.cpp側に残す。
coreはEigen、OpenCV、rclcpp、OSQPへ依存しない値型中心とする。

## データ構造

### DetectorInput

    struct DetectorInput
    {
      double now_sec;
      bool race_started;
      bool control_enabled;
      bool odometry_fresh;
      bool solver_fallback;
      bool deliberate_stop;
      bool gear_transition_active;
      bool awsim_recovery_settling;
      double signed_speed_mps;
      double requested_forward_speed_mps;
      double requested_acceleration_mps2;
      double pose_displacement_m;
      double unwrapped_progress_delta_m;
      bool wall_evidence;
      bool collision_hint;
    };

deliberate_stopは少なくともSafetyBrake、Followによる停止、LowSpeedAvoidance待機、
operator stopを集約した値とする。

### DetectorDecision

    enum class StuckVerdict
    {
      NotEligible,
      Moving,
      Suspected,
      Confirmed,
    };

    struct DetectorDecision
    {
      StuckVerdict verdict;
      StuckRejectReason reject_reason;
      double stationary_duration_sec;
      double pose_displacement_m;
      double progress_delta_m;
    };

reasonは文字列判定ではなくenumを正本にし、ログ表示時だけ文字列へ変換する。

### RecoveryInput

    struct RecoveryInput
    {
      double now_sec;
      DetectorDecision detector;
      bool race_active;
      bool control_enabled;
      bool odometry_valid;
      bool solver_healthy;
      bool hard_stop_requested;
      bool awsim_recovery_settled;
      Gear reported_gear;
      bool gear_report_fresh;
      bool rear_static_clear;
      bool rear_v2x_clear;
      bool rear_information_complete;
      bool collision_worsening;
      bool recovery_escape_confirmed;
      bool rejoin_safe;
      double signed_speed_mps;
      double traveled_distance_m;
      double lateral_error_m;
      double heading_error_rad;
    };

### RecoveryAction

    enum class RecoveryActionType
    {
      NormalControl,
      HoldStop,
      RequestReverse,
      ReverseCreep,
      RequestDrive,
      LowSpeedRejoin,
      SafeStop,
    };

    struct RecoveryAction
    {
      RecoveryActionType type;
      Gear requested_gear;
      double acceleration_magnitude_mps2;
      double steering_tire_angle_rad;
      double rejoin_speed_limit_mps;
      bool inhibit_boost;
      bool reset_normal_control;
      RecoveryReason reason;
    };

coreはROS messageを返さない。後退加速度も非負のmagnitudeで返し、node adapterが
実測済みの`reverse_acceleration_sign`を適用してGearCommandと
AckermannControlCommandへ変換する。sign 0は意図的な駆動禁止値である。

## スタック検出

### 基本条件

次を全て満たす状態が設定時間継続した場合にSUSPECT_STUCKとする。

- race_started。
- control_enabled。
- odometryがfreshかつ有限。
- gear遷移中ではない。
- 通常はsolver fallbackではない。下記の限定例外だけを認める。
- deliberate stopではない。
- Normal MPCが明確な前進速度または前進加速度を要求している。
- signed speedの絶対値が停止閾値以下。
- pose変位が閾値以下。
- unwrapped path progressが閾値以下。

さらにConfirmedへ進むには、wall proximity、front footprint接触、急減速履歴、
legacy collision hintのいずれかを補助証拠として要求する。

solver fallback例外は、通常のMPC失敗で車両が停止し続けるP1/P2事象を拾うための
限定経路である。次を全て満たさなければならない。

- fallbackが連続2.0秒以上。solverが1周期でも復帰すればtimerをresetする。
- detector更新間隔が0.2秒を超えた場合もtimerをresetする。
- solver出力ではなくpath上限から得た前進要求が存在する。
- signed speed、pose変位、unwrapped path進捗が停止条件を継続する。
- 現在の車体footprintがoccupied wall cellと交差する。
- deliberate stop、Start前、gear遷移、AWSIM recovery settlingではない。

この例外ではlegacy collision hint単独を証拠として採用しない。qualified fallbackで
Recoveryが制御権を取得した後は、通常MPCのfallback継続だけでFSMをSafeStopへ落とさない。
ただし`LOW_SPEED_REJOIN`は通常MPCを再使用するため、solverが復帰していなければSafeStopとする。

初期config候補:

    stuck_recovery:
      enabled: false
      shadow_mode: true
      simulation_only: true
      detector:
        solver_fallback_recovery_enabled: true
        solver_fallback_duration_sec: 2.0
        max_observation_gap_sec: 0.2
        stopped_speed_mps: 0.15
        forward_intent_speed_mps: 1.0
        stationary_duration_sec: 1.2
        max_pose_displacement_m: 0.15
        max_progress_delta_m: 0.20
        awsim_recovery_settle_sec: 1.2

これらは公式値ではない。baseline logとShadow modeで誤検知率を確認してから採用する。

### 進捗

nearest waypoint IDはヘアピンの隣接区間や周回seamで飛ぶ可能性がある。
検出では次を併用する。

- window開始poseからのEuclidean displacement。
- 前回投影近傍だけを探索する連続path projection。
- lap seamでunwrapしたpath progress。

poseがAWSIM壁リカバリーにより回転している間は、位置が止まっていても独自Recoveryを開始しない。
yaw変化率またはpose jumpが安定するまでWAIT_AWSIM_RECOVERYを維持する。

### 除外状態の受け渡し

現在のV2X behavior outputはMPC内部で評価される。
Recovery detectorへ次の集約状態を公開する。

- behavior state。
- has_front_vehicle。
- SafetyBrakeまたはEmergencyBrake。
- LowSpeedAvoidance candidate / active。
- solver fallback。
- operator / race stop。

文字列ログの解析で除外状態を判断しない。

## Recovery FSM

### 状態一覧

    NORMAL
      |
      | confirmed stuck
      v
    WAIT_AWSIM_RECOVERY
      |
      | pose stable and still stuck
      v
    STOP_AND_CONFIRM
      |
      | speed below threshold for hold time
      v
    CHECK_CLEARANCE
      | safe                  | unsafe / unknown
      v                       v
    SHIFT_TO_REVERSE      WAIT_FOR_CLEAR / SAFE_STOP
      |
      v
    WAIT_REVERSE_REPORT
      | matching report       | timeout
      v                       v
    REVERSE_MANEUVER       SAFE_STOP
      |
      | clear / limit / hazard
      v
    STOP_BEFORE_DRIVE
      |
      v
    SHIFT_TO_DRIVE
      |
      v
    WAIT_DRIVE_REPORT
      | matching report       | timeout
      v                       v
    LOW_SPEED_REJOIN       SAFE_STOP
      |
      | heading and lateral error accepted
      v
    NORMAL

全状態にoperator stop、Finish、odom stale、非有限値の共通割込みを持たせる。
これらの割込みではHoldStopまたはSafeStopを返す。

### WAIT_FOR_CLEAR

後方車が一時的に通過する可能性があるため、直ちにlatched failureへせず、
短い待機と再評価を許可する。

- wait timeoutを持つ。
- 待機中はgearをDRIVEまたはNEUTRALへ固定し、駆動を出さない。
- V2Xがfreshになり後方がclearになった場合だけCHECK_CLEARANCEへ戻る。
- timeoutまたは情報欠落継続でSafeStopへ遷移する。

### retry

- 最初はmax_attempts=1とする。
- Phase 3以降も上限は1〜2回に限定する。
- 同じcandidateを同じ初期poseから無限再実行しない。
- attemptごとに開始pose、candidate、終了理由を保存する。

## Gear I/O

### Publisher / subscriber

- publish: /control/command/gear_cmd
  autoware_auto_vehicle_msgs/msg/GearCommand
- subscribe: /vehicle/status/gear_status
  autoware_auto_vehicle_msgs/msg/GearReport

multi_purpose_mpc_rosへautoware_auto_vehicle_msgs依存を追加する。
公式interfaceをdocs/interface/participant-interface.mdへ先に反映する。

### 切替手順

1. HoldStopをpublishする。
2. signed speedが停止閾値以下でhold時間継続したことを確認する。
3. GearCommand::REVERSEをpublishする。
4. fresh GearReport::REVERSEを確認する。
5. 実測済みの低い駆動加速度で後退する。
6. 距離、時間、hazardのいずれかで停止する。
7. 再び停止を確認する。
8. GearCommand::DRIVEをpublishする。
9. fresh GearReport::DRIVEを確認する。
10. LowSpeedRejoinへ進む。

GearReportを確認できない場合、加速度を出さない。
REVERSE時の加速度符号は公式interfaceだけでは確定できないため、
GearActuationAdapterへ閉じ込め、AWSIM実験結果をtest fixtureと文書へ残す。

## Command arbitration

Normal走行では従来どおりMPC結果をpostprocessしてpublishする。
Recovery active時はMPC出力をpublishせず、RecoveryActionだけを最終commandへ変換する。

    if recovery_action.type == NormalControl:
        publish(normal_mpc_command)
    else:
        inhibit_boost()
        publish(recovery_or_stop_command)
        publish_gear_if_requested()

Recovery state中にnormal controlとrecovery controlを同一周期で両方publishしない。
現実装はNormal MPC solveとpath error計算を継続し、Recoveryがownerの周期だけその出力を
破棄してRecovery / SafeStop commandへ置換する。

legacy `use_bug_acc`経路を使用する構成でも、Recovery arbitrationがcontrol ownerに
なった周期はlegacy boost accelerationを無効化する。

## Static map / footprint安全判定

現行Mapはreference waypointから左右境界までの幅を計算する用途が中心であり、
車体矩形のreverse swept collision checkerではない。
またMap::w2mは範囲外座標をmap端へclampするため、Recovery安全判定へそのまま使わない。

runtime checkerには次を実装した。

- world座標からgridへの変換で範囲外を明示的にreject。
- unknown / invalid / out-of-mapをoccupied。
- pose基準のfront_extent、rear_extent、left_extent、right_extent。
- margin込みの向き付き矩形rasterization。
- rollout間を補間したswept footprint。
- initial contact cellsと新規contact cellsの区別。
- 初期contact patchの固定1-cell haloと、直前patchの同一・8近傍cellを併用した連続遷移。

初期接触poseの扱い:

- t=0の接触だけで全候補をrejectしない。
- 接触cellは明示Occupiedだけを許し、unknown / invalidはfail-closedとする。
- 初期接触cellの全体がrear-axle pose基準より前方にある場合だけStraight reverse候補とする。
- 各sampleで接触数を増やさず、現在cellが初期patchの固定halo内かつ直前patchと同一または
  8近傍であることを必須にし、patchのchain migrationを禁止する。
- 一定距離以内にinitial contactを解消する。
- 一度接触が0になった後の再接触と、離れたwall patchへの接触は即rejectする。
- rollout評価と実後退中のruntime監視で同じ`evaluate_contact_transition`を使用する。
- `swept_step_m`はmap resolution以下を必須とし、1 sampleでwall patchを飛び越えない。
- 初期接触を持つ候補はStraightだけを許可し、yawが変わるLeft / Rightは方向付き
  penetration metricを導入するまでfail-closedとする。

trajectory editorのtrajectory_clearance.pyとtest fixtureは概念を参照できるが、
runtimeでPython GUIコードを呼び出さない。

## V2X後方安全

既存V2X trackerのactive vehicle、速度推定、timeout、position jumpを再利用できるが、
次の制限を明示する。

- 自車IDと自車除外は未確定。
- 空IDの複数車を同一対象にまとめる可能性がある。
- V2Xが後方全車を必ず含む保証は未確定。
- latencyとprediction誤差がある。

初期版では次をhard conditionとする。

- V2X messageが設定timeout以内。
- self filterが成立している。
- reverse swept corridorと膨張済み他車予測が交差しない。
- rear information completeと判断できない場合はReverseCreepへ進まない。

SIMでunknown時の限定creepを検討する場合も、初期安全版とは別flagにし、
実車へ流用しない。

## 直進後退MVP

初期MVPではReverseStraightだけを扱った。2026-07-13拡張後もStraightを最優先とするが、
Straightが不可能で初期footprintがclearな場合はLeft / Rightを選択できる。

暫定config候補:

    stuck_recovery:
      reverse_actuation_enabled: true
      reverse_acceleration_sign: 1.0
      reverse_stop_acceleration_mps2: -0.8
      verified_reverse_stop_deceleration_mps2: 0.4
      reverse_control_latency_sec: 0.2
      maneuver:
        max_reverse_distance_m: 0.8
        max_reverse_duration_sec: 2.0
        max_reverse_speed_mps: 0.8
        reverse_acceleration_magnitude_mps2: 0.5
        max_reverse_pose_step_m: 0.05
        max_attempts: 1
      gear:
        report_timeout_sec: 0.5
        stop_confirm_sec: 0.3
      rejoin:
        speed_limit_mps: 1.0
        max_lateral_error_m: 0.5
        max_heading_error_rad: 0.35

数値は2026-07-12のローカルAWSIM単車校正と運営チャットの低速・短時間方針に基づく
P1 SIM限定値であり、2026公式上限ではない。実車へ転用しない。

## 3プリミティブ拡張

pure APIでは次の3プリミティブを同じ静的安全規則で評価できる。

- ReverseStraight。
- ReverseLeft。
- ReverseRight。

各candidateを負のsigned velocityを用いるkinematic bicycle modelで短距離rolloutする。
planning model内の負速度は候補軌跡計算専用であり、Normal MPCへ渡さない。

現runtimeは3候補をStraight、Left、Rightの順で評価し、最初のfeasible候補を
static footprint、V2X、Boost、gearのhard conditionへ接続して実commandへ変換する。
選択候補はRecovery episode中に固定する。

3候補をruntimeへ統合する際の評価順:

1. 入力有限性と上限。
2. static map full footprint。
3. out-of-map / unknown。
4. initial contactからの離脱性。
5. rear V2X predicted occupancy。
6. 終端poseからのforward rejoin可能性。
7. hard condition通過後のscore。

score候補:

- minimum static clearance。
- initial contactからの離脱量。
- reference pathまでの距離。
- path heading error。
- reverse distance。
- steering change。

異なる単位を直接加算せず正規化する。同点時はStraight、現在操舵に近い側、
固定side orderなど決定的なtie-breakを定義する。

## LowSpeedRejoin

Drive gear確認後に次を行う。

- 現在poseからreference pathへ再投影。
- MPC current_control、prediction、previous steering、fallback speed、
  solver countersをreset。
- V2X behavior state、OvertakeLine、target ID、pass side、
  LowSpeedAvoidance target lockをreset。
- acceleration / steering low-pass historyを現在値または停止値で初期化。
- Boost latchを再armしない。
- heading / lateral errorが閾値以内になるまで低速上限を適用。

現実装は現在poseから既存のnearest path projectionを使う。Recovery開始前wpと局所連続性を
使った周回の誤枝抑制は未実装である。逆向きのheadingで通常速度へ戻さない判定は
lateral / heading errorとhold時間で行う。

## Failure handling

| Failure | Action |
|---|---|
| odometry stale / non-finite | 既存failsafe、Recovery停止 |
| gear report timeout | SafeStop |
| rear情報unknown / stale | WaitForClear、timeout後SafeStop |
| static candidateなし | SafeStop |
| reverse中のV2X接近 | 即停止 |
| reverse中の新規壁接触 | 即停止、attempt消費 |
| max distance / duration | StopBeforeDrive |
| DRIVE report timeout | SafeStop |
| rejoin timeout | SafeStop |
| operator stop / Finish | HoldStop、session終了処理 |

/admin/awsim/reset、teleport、/set_initial_pose連打をfailure fallbackに使用しない。

## Parameter rollout

初期設定:

    stuck_recovery:
      enabled: false
      shadow_mode: true
      simulation_only: true

段階導入:

1. disabledで既存回帰。
2. shadow onlyで全Domainの誤検知計測。
3. controlled scenarioでD1だけ実制御。
4. 正面接触、後方clear条件でのみ有効。
5. dev2 / dev3で後方車ありを検証。
6. 公式確認と証跡後にonline用既定値を判断。

実車では別の明示flagを必要とし、simulation_onlyを自動で解除しない。

## テスト設計

### Pure C++ unit test

- detector timer、reset、hysteresis。
- deliberate stop除外。
- AWSIM settle wait。
- gear state transitionとtimeout。
- distance / duration / attempt upper limit。
- session reset。
- rear unknown / unsafe。
- RecoveryActionの排他性。
- feature disabled互換。

### Footprint unit test

- axis-aligned / rotated rectangle。
- map boundary。
- unknown cell。
- swept collision between safe endpoints。
- initial contactから離れる候補。
- initial contactを悪化させる候補。
- new obstacle contact。
- Straight / Left / Rightへ共通のstatic safety規則。

### Node integration

- GearCommand / GearReport型とQoS。
- 1周期1つのcontrol owner。
- gear report前にdrive commandを出さない。
- Recovery中のBoost抑止。
- Normal復帰時のhistory reset。
- config key省略時disabled。
- V2X predicted rear crossingとfreshness（未実装のnode integration test）。

### AWSIM

- wall recoveryだけで復帰するケース。
- wall recovery後も正面スタックするケース。
- 左右角接触。
- 後方壁。
- 後方車静止。
- 後方車接近。
- Follow / SafetyBrake停止。
- gear report欠落または遅延。
- race Finish / manual stop。
- first hairpinと1周の通常回帰。
- dev2 / dev3の後続車安全性。

## 文書更新

実装時に次を更新する。

- docs/interface/participant-interface.md: gear command / status公式契約。
- docs/spec/mpc-integration.md: StuckRecovery、state、priority、config。
- multi_purpose_mpc_ros/README.md: 有効化、ログ、制約、検証方法。
- docs/spec/open-questions.md: 自律後退の公式確認事項。
- docs/spec/safety-gates.md: スタック検知と後方安全の証跡。

## 公式チャットへ確認する事項

2026-07-12の運営チャット回答では、技術的な実装は可としつつ、競技利用は慎重に扱い、
低速・短時間・後方clearを満たすスタック復帰に限定し、戦略的な後退は避けるよう案内された。
本実装はこの運用制約を採用する。ただし公開ルールに数値上限や正式許可が明記された
わけではないため、以下の詳細確認は残す。

1. SIM予選で参加者コードがREVERSEを用いて自律復帰してよいか。
2. 後退距離、速度、時間、回数に上限またはペナルティがあるか。
3. REVERSE時のAckermannControlCommand.longitudinal.accelerationの符号。
4. gear_cmdの再送要否、gear_statusの更新周期、切替遅延、timeout推奨値。
5. online SIM予選でgear_statusが必ず配信されるか。
6. 公式collision / wall recovery active状態を取得できるtopicがあるか。
7. 自律後退が逆走、蛇行、妨害と判定される条件。
8. Recovery中のcheckpoint、lap count、collision penaltyの扱い。
9. SIM決勝・実機決勝のmanual recoveryと自律Recoveryの優先関係。

## 2026-07-12 検証結果

- `make autoware-build`: 成功。
- `test_stuck_recovery_core`: 33 / 33成功。
- `test_recovery_footprint`: 19 / 19成功。
- package全体test: Recovery新規testは全て成功。既存
  `final_ver3/traj_mincurv.csv`の重複終端fixture期待1件だけ失敗。
- `git diff --check`: 成功。
- `config.yaml` YAML parse: 成功。
- AWSIM単車校正: Reverse report遅延約0.035 s、Drive report遅延約0.015 s、
  command-to-effect約0.140 s。`+0.5 m/s^2`で後退し、`-0.8 m/s^2`で約0.154 s後に停止、
  平均停止減速度は約0.628 m/s^2。
- 3台クリーン起動: Domain 1 / 2はActive、Domain 3はdisabled、V2Xは各Domain 2 entryを確認。

## 2026-07-13 追加設計

### Evidence-free detector fallback

通常solverが正常で、既存のeligibility、forward intent、speed、pose displacement、path progressを
全て満たすがwall / collision証拠だけがない場合、`evidence_free_duration_sec`までSuspectedを
維持し、到達後にConfirmedとする。solver fallbackはこの経路を使用せず、従来どおりcurrent
wall footprint証拠を必須とする。V2X Follow / SafetyBrake / LowSpeedAvoidance等はnode adapterが
`deliberate_stop`として除外する。

### Recoverable clearance SafeStop

`clearance_wait_timed_out`だけをrecoverable SafeStopとする。rear static、fresh / complete V2X、
rear corridor clearが`safe_stop_clear_confirm_sec`継続した後に`CHECK_CLEARANCE`へ戻す。
attempt limit、gear timeout、不正gear、odometry、solver、control interrupt、collision worseningは
latchedのままとし、session reset以外で解除しない。

### Runtime primitive selection

rolloutはStraight、Left、Rightを順に評価し、最初のfeasible候補をepisode中latchedする。
Left / Rightは`reverse_steering_angle_rad`の符号で区別し、選択角をpure FSMの
`RecoveryInput.reverse_steering_tire_angle_rad`から`RecoveryAction`へ渡す。runtime監視と
command arbitrationは同じ角度を使う。選択rolloutの中心横変位をV2X corridorへ加算し、
操舵バック中も後方車両をhard rejectする。

初期contact patchが存在するLeft / Rightは既存pure safety規則によりrejectされる。これは
物理壁とmapが一致しないがfootprintはclearな今回のケースを救済しつつ、接触中の旋回で
penetrationを悪化させないための制約である。

### Following vehicle reserve

新しいV2X topicやmessage型は導入しない。`v2x_start_grid_grace_time: 5.0`、既存の
LowSpeedAvoidance、`v2x_safety_brake_distance: 6.0`で先頭停止車への接近を抑え、dev3ログで
最低距離と回避遷移を確認する。これで不足する場合だけ車両間Recovery intentを別仕様とする。

### 2026-07-13 Front / Rear wall direction recovery

`classify_nearby_wall`は通常footprintへ`wall_direction_search_margin_m`だけ追加した検索矩形内の
occupied / unknown cellを車体座標へ変換し、最短側をFront / Rear / Left / Rightへ分類する。
複数側が`wall_direction_ambiguity_m`以内、角接触、map外、近傍wallなしはMixed / Unknownとして
駆動しない。検索marginは方向推定専用で、collision clearanceを緩和しない。

- Front: ReverseStraight / ReverseLeft / ReverseRightを従来順で評価する。
- Rear: ForwardStraightだけを評価し、Drive report確認後に短距離前進する。
- Left / Right / Mixed / Unknown: Phase 12時点ではSafeStop。Side / Mixedは後続Phase 13で
  改善証明付き段階離脱へ拡張し、Unknownは引き続きSafeStopとする。

ForwardStraightも同じswept-footprintとcontact transitionを使う。初期接触がrear側だけであること、
候補終端までにcontactがclearになること、前方へ拡張したV2X corridorがcomplete / freshであることを
必須とする。上限は0.6 m / 1.5 s / 0.8 m/s、駆動0.5 m/s^2、escape 0.30 mである。

Reverse gear reportが先に届きV2X completenessが次周期になる場合は、`WAIT_REVERSE_REPORT`で
停止commandを維持する。`clearance_wait_timeout_sec`内にclearになった場合だけReverseManeuverへ
入り、継続して不完全なら停止確認後Driveへ戻す。

### 2026-07-13 Side / Mixed stepwise escape

`wall=mixed`、`current_contacts=144`、`maneuver_direction_unknown`でP1がlatched SafeStopとなった
実走結果を受け、Side / Mixedに限って段階候補評価を追加する。Front / Rearの従来候補は
`RequireClear`と固定initial haloを維持する。

Side / MixedではForwardStraight / ForwardLeft / ForwardRight / ReverseStraight /
ReverseLeft / ReverseRightを0.20 mだけ予測する。各swept sampleは明示Occupiedだけを許し、
contact数がステップ初期値を超えず、各current cellがprevious patchの8近傍へ連続することを
必須とする。終端contactが初期値から5%以上減らない候補は`contact_not_improved`でrejectする。
contact減少最大を選び、同点はForward、Straight、Left、Rightの順とする。

実commandも0.20 mで停止し、実測contactが減った場合だけ`STOP_AND_REASSESS`から次の
`CHECK_CLEARANCE`へ戻る。実測改善なしはForwardなら即SafeStop、Reverseなら停止・Drive確認後に
SafeStopとする。最大3ステップでclearしない場合も`escape_step_limit_reached`で停止する。
各候補には従来どおり方向別V2X corridor、Boost inactive、fresh gear reportを適用する。

### 2026-07-13 自動検証結果

- `make autoware-build`: 成功。
- `test_stuck_recovery_core`: 37 tests成功。
- `test_recovery_footprint`: 19 tests成功。
- package全16 test中15成功。失敗1件は本変更前から記録済みの
  `PathCoreCircular.RemovesOneEndpointFromConfiguredFinalVer3Trajectory`で、入力trajectoryの
  duplicate endpoint期待と現データが一致しないfixtureである。
- config YAML parse、`git diff --check`: 成功。
- 変更したpure core / test / headerは`ament_uncrustify`に合格した。
  `mpc_controller_cpp.cpp`全体は既存箇所を含むstyle divergenceがあるため、広範囲の機械整形は
  本変更へ含めなかった。
- P1は`Confirmed -> WAIT_AWSIM_RECOVERY`へ入り、AWSIM標準wall recoveryで動きが戻った
  ケースでは`awsim_recovery_resolved`として独自Reverseを発動せず通常制御へ戻った。
- 最終の前方接触 / 固定initial halo / corner-motion強化はpure testとbuildで確認し、
  最終binaryの独自Reverse実走は未検証として残した。
- 3台・360秒走行: P1はStart待機、Follow / SafetyBrake由来のdeliberate stop、
  長時間solver fallbackを経験したが、wall evidenceがないためRecoveryは0回だった。
  `solver_fallback_missing_wall_evidence`でfail-closedになることをログで確認した。
- 再ビルド後のP1 / P2起動でActive設定の読込に成功し、gear publisher QoSが
  Reliable / KeepLast(1) / VolatileであることをROS graphから確認した。
- AWSIM標準補正でも解消しない正面壁スタックの独自Reverse end-to-end、40 Hz deadline、
  Left / Right実走、RVizは未検証または未実装。

### 2026-07-13 Phase 14: Reverse中V2X欠落と再判定遷移の修正

`output/20260713-063758/d1/autoware.log`では、P1が`ReverseRight`を選択して実際にReverseへ入り、
map contactを182から178へ減らした。一方、ReverseManeuver中の1周期だけ
`corridor_complete=false`となり、従来FSMは即座に`STOP_BEFORE_DRIVE`へ移った。さらにDrive report
受信周期を常にLowSpeedRejoin開始とみなしたため、継続中のsolver fallbackを`solver_unsafe`として
先にSafeStopし、本来の`STOP_AND_REASSESS`へ到達できなかった。

ReverseManeuver中にstatic / V2X corridorが不完全またはblockedになった場合は、駆動commandを
即座に止めて`WAIT_REVERSE_REPORT`へ戻す。Reverse gearを維持したまま
`clearance_wait_timeout_sec`以内にcompleteかつclearへ戻れば同じmaneuverを再開し、継続する場合だけ
停止確認後Driveへ戻す。再開時は停止中のpose driftを距離へ加算せず、累積移動距離、初期contact、
直前contactを初期化しない。一方、step完了後に次候補を評価する場合は、Drive reportと同じ周期で
`CHECK_CLEARANCE`まで進んでも新しいstepとして距離・contact基準を更新する。

stepwise Reverseで`reassess_after_drive`または`safe_stop_after_drive`が予約されているDrive reportは
LowSpeedRejoin開始ではないため、normal MPC solverの正常性を要求しない。Drive確認後に予定どおり
再判定またはSafeStopへ進める。通常Reverse離脱からLowSpeedRejoinへ入る場合は従来どおりsolver正常を
必須とする。

検証結果は`make autoware-build`成功、`test_stuck_recovery_core` 44件成功、
`test_recovery_footprint` 24件成功。package全体は既知のtrajectory endpoint fixture 1件だけ失敗した。
修正後の同一Side / Mixed実走は未確認である。

### 2026-07-13 Phase 15: Side候補がReverse要求前に中断する問題

`output/20260713-080636/d1/autoware.log`では、P1はSide / Mixed接触に対して
`ForwardRight`、続いて`ReverseLeft`を正常に選択していた。しかし`WAIT_AWSIM_RECOVERY`中に、
速度制限を持たないside vehicle由来の`Follow`が`deliberate_stop=true`へ集約され、coreがこれを
`hard_stop_requested`へ変換して`control_interrupted`のlatched SafeStopへ移った。このため
Reverse gear requestまで到達しなかった。

`deliberate_stop`は通常状態でRecovery誤検知を防ぐeligibility入力とし、Followは実front vehicleが
ある場合だけ対象とする。side vehicleの存在だけでは意図的停止とみなさない。開始済みRecoveryでは
通常behaviorの一時状態をimplicit hard stopへ変換せず、adapterから明示された
`RecoveryInput.hard_stop_requested`だけをoperator / external hard stopとして維持する。実際の
Forward / Reverse駆動は、従来どおり方向別static swept footprintとfresh / completeなV2X corridorが
clearの場合だけ許可する。

同ログではAWSIM標準補正による小さなpose変化でdetector windowがresetされ、map contactが5 cell
残っていても`awsim_recovery_resolved`へ戻る事象も確認した。`WAIT_AWSIM_RECOVERY`から通常制御へ
戻る条件に現在footprint clearを追加し、contactが残る場合は補正待機後に`STOP_AND_CONFIRM`へ進める。

検証結果は`make autoware-build`成功、`test_stuck_recovery_core` 47件成功、
`test_recovery_footprint` 24件成功。明示hard stopがlatched SafeStopになる既存安全経路も新規testで
固定した。package全体は既知のtrajectory endpoint fixture 1件だけ失敗した。修正後の同一
Side / Mixed実走は未確認である。

### 2026-07-13 Phase 16: Reverse距離化とRear復帰待ち短縮

`output/20260713-081653/d1/autoware.log`では、`ReverseLeft`候補がcontact 222→207の予測で成立し、
AWSIM待機後も217→202、217→200、216→200と繰り返し成立していた。しかしV2X completenessが
周期ごとに揺れ、`WAIT_FOR_CLEAR`から一度`CHECK_CLEARANCE`へ移した次周期にstatic候補が
`contact_worsened`へ変わったため、Reverse gear要求前に`maneuver_direction_unknown`でSafeStopした。

`WAIT_FOR_CLEAR`でstatic / V2X clearanceが成立した場合は、同じ入力snapshotで
`update_check_clearance`まで実行し、Reverse gear要求へ進める。これによりclearだった周期と
gear要求の間に別周期を挟まない。

競技上の実態としてRear接触以外は前進中の衝突からの離脱であるため、Side / Mixedの候補を
ReverseStraight / ReverseLeft / ReverseRightへ限定する。0.20 m×最大3回の細切れ操作をやめ、
0.40 m後退後に停止・再評価し、まだcontactが減少していてclearでなければもう0.40 mまで許す。
合計上限は0.80 mで、各候補のswept-footprint、実contact改善、V2X corridor、gear report、速度・
時間上限は維持する。ForwardStraightはRear分類時だけ使用する。gear要求前には0.40 m終端の
5%以上改善を要求し、開始後は残距離ごとの追加改善を要求せず、contact非増加・新規contactなしを
監視して0.40 mまで継続する。終点の実改善判定は維持する。

Rear復帰を含む初動短縮のため、wall evidenceとpose / path無進捗を必須としたまま
`stationary_duration_sec`を1.2→0.4秒、AWSIM標準補正優先待ちを1.2→1.0秒、停止確認を
0.3→0.2秒へ変更した。AWSIM補正の優先自体は維持し、Rearでは補正待ち後にDriveのまま
ForwardStraightへ進む。

検証結果は`make autoware-build`成功、`test_stuck_recovery_core` 47件成功、
`test_recovery_footprint` 25件成功。package全体は既知のtrajectory endpoint fixture 1件だけ
失敗した。変更後のSide / Rear実走は未確認である。

### 2026-07-13 Phase 17: map不一致時にP1がConfirmedへ進まない問題

`output/20260713-083842/d1/autoware.log`では、P1はwp_id 187で長時間0 m/sだったが、occupancy map上は
`wall=none`、`current_contacts=0`だった。通常MPCのtarget再構築により`normal_u[0]`が周期ごとに0と
非0を繰り返し、detectorは`Suspected / missing_corroborating_evidence`へ入っても次周期の
`no_forward_intent`で観測timerをresetした。そのためRecovery FSMへ一度も制御権が渡らなかった。

detectorの`requested_forward_speed_mps`はreference path速度要求と`normal_u[0]`の最大値とし、
正常solverでpathが前進を要求している限りstableなintentを維持する。証拠なし継続時間を
3.0→1.5秒へ短縮する。

物理壁とmapが不一致でも方向を生成できるよう、validなmap上で現在footprintがclear、後方0.8 mの
ReverseStraight swept rolloutがclearの場合に限りfallback候補を作る。core側のevidence-free
Confirmed、正常solver、無進捗、fresh / completeなV2X後方clear、fresh Reverse gear reportを
全て維持し、実commandは`escape_distance_m=0.30`で停止する。map invalid / out-of-map / unknown、
solver fallback、V2X不完全では動かない。

検証結果は`make autoware-build`成功、`test_stuck_recovery_core` 47件成功、
`test_recovery_footprint` 25件成功。変更後の同一P1停止シナリオは未確認である。

### 2026-07-13 Phase 18: dev3全車停止の候補再選択と方向fallback

`output/20260713-084700`ではP2が約5.46 m/sから接触し、OSQP fallbackが約1900回まで継続した。
P2は一度`ReverseStraight`候補を得たが、1.0秒のAWSIM補正待機中にmap contactが190→194へ変化し、
補正前に固定した候補だけを再評価して`contact_worsened`、`maneuver_direction_unknown`の
SafeStopへ入った。`STOP_AND_CONFIRM`から`CHECK_CLEARANCE`へ入る境界でcandidate latch、移動距離、
contact baselineを現在snapshotへ更新し、実駆動前にStraight / Left / Rightを選び直す。

P1はLeft wall、wall distance 0、map contact 0でConfirmedまで進んだが、Side候補生成がmap contact
ありの場合だけだったため`static=invalid_grid`の既定診断を残して方向不明となった。現在footprintが
clearなLeft / Right / MixedでもReverse候補とForwardStraight候補のstatic swept rolloutを評価する。
通常はReverseを優先する。fresh / completeなV2XでReverse corridorだけが塞がれ、ForwardStraightの
map rolloutとforward corridorがclearの場合に限りForwardへ切り替える。後方に留まる他車は直進前進で
距離が増えるためforward corridorの新規衝突対象から除外し、前方・横並び・将来前方へ入る他車は
従来どおりinflated radiusでrejectする。候補固定後の走行中に方向は切り替えない。

P3は設定どおりRecovery無効で、P1を1.59 m前方に検出してSafetyBrakeした。これはノード異常ではなく
P1 / P2の停止に伴う後続安全停止である。上記方向fallbackは、この停止車列でP1が後退できない場合の
限定的なデッドロック解除である。

検証結果は`make autoware-build`成功、`test_stuck_recovery_core` 47件成功、
`test_recovery_footprint` 25件成功。変更後のdev3 end-to-endは未確認である。

### 2026-07-13 Phase 19: Reverse未実行のAWSIM / V2X前段修正

`output/20260713-091107/d1/autoware.log`ではReverse候補は繰り返し生成されたが、gear reportは
Driveだけで、ShiftToReverse / RequestReverse / ReverseCreepは0回だった。最初の2 episodeは
evidence-free Confirmed後、AWSIM由来とみられる約0.15 mのpose変化だけでdetectorがMovingとなり、
map footprintが開始時からclearだったため`awsim_recovery_resolved`へ誤遷移した。

Recovery episode開始時にwall / collision / map contact証拠の有無をlatchする。証拠ありepisodeは
従来どおりfootprint clearでAWSIM補正成功を認める。evidence-free episodeはpose変化だけでは認めず、
reference path progressが`max_progress_delta_m`以上ある場合だけ成功とする。進捗がなければ
STOP_AND_CONFIRMへ進める。

3回目はCHECK_CLEARANCEまで進んだが`rear_information_incomplete`でtimeoutした。V2Xは2台を受信して
いた一方、約1 Hzの更新間に正常走行車が固定`position_jump_threshold=5 m`以上移動し、corridor側が
position jumpとしてrejectしていた。jump許容距離を
`max(position_jump_threshold, v2x_v_max_safety * message_dt)`へ変更し、時間差に対して物理的に
成立する移動を受理する。非有限・逆行stamp、許容速度超過、message不完全は引き続きfail-closedとする。

検証結果は`make autoware-build`成功、`test_stuck_recovery_core` 48件成功、
`test_recovery_footprint` 25件成功。実際のReverse gear reportと負速度はdev3再実行待ちである。

### 2026-07-13 Phase 20: 2.0 m escapeとRejoin完了条件の固定

`output/20260713-092501/d1/autoware.log`ではP1が初めてReverse gearとReverseCreepへ入ったが、
約0.10秒後にcorridor completenessがfalseとなり、実移動約0.01 mでDriveへ戻った。その後、
map footprint clearと開始時点から閾値内だった`e_y` / `e_psi`だけで約0.35秒後に
`rejoin_complete`となり、実際にはwp117から進まず再度スタックした。

Front / Sideは1 maneuverではなくRecovery episode全体の実移動2.0 mをescape条件とする。
Side / Mixedの0.40 m段階離脱ではstep距離だけをresetし、episode距離は保持する。停止予約を含む
hard上限は3.0 m、最大8 step、contact clear後の非stepwise継続を含め最大2 attemptsとする。
Rearは従来のForward 0.30 mを維持し、方向別keyへ分離する。

Reverse中にV2X / Boost完全性が欠落した場合は直ちに駆動を止めるが、情報欠落だけではDriveへ
戻さずReverse gearのまま待つ。completeなsnapshotへ戻れば同一maneuverを距離・contact baselineを
維持して再開する。completeなstatic / vehicle blockageが継続した場合だけDriveへ戻す。

Drive reportを受けてもescape latchが成立していなければ`escape_not_confirmed` SafeStopとし、
LowSpeedRejoinへ入れない。LowSpeedRejoin中もV2X completeを必須とし、欠落中は停止保持する。
完了はescape latch、map footprint clear、V2X complete、`|e_y| <= 0.5 m`、
`|e_psi| <= 0.35 rad`の0.3秒継続を必須とする。

0.8 m/sはescape終了条件ではなく速度レギュレータとする。上限到達周期はReverseのまま停止加速度を
出し、上限未満へ戻れば同じmaneuverを再開する。これにより一定加速度で約0.64 mしか進まず
終了する経路をなくす。

ログはstateだけでなくreason変化でも出し、Boost inactive、V2X message completeness、maneuver /
episode距離、停止予約、escape target / 成立、rejoin safe、`e_y`、`e_psi`を記録する。

検証結果は`make autoware-build`成功、`test_stuck_recovery_core` 51件成功。package全体16 target中
15 target成功で、失敗1件は既知のtrajectory duplicate endpoint fixtureである。2.0 m実後退と
復帰後path progressはdev3再実行待ちである。
