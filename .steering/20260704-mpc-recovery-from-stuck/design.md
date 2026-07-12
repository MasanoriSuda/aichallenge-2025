# MPC Stuck Recovery Design

作成日: 2026-07-12
更新日: 2026-07-12
状態: Implementation Complete / P1-P2 AWSIM Scenario Verification Pending

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
4. runtime footprint checkerと3後退プリミティブ。現在はStraightだけを実制御へ
   統合し、Left / Rightはpure評価APIまでとする。
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
- Active時もStraightのstatic swept footprint、fresh V2X rear corridor、freshかつ
  inactiveなBoost、freshで一致したGearReportのhard conditionを順に満たす。
- Recoveryがcommand ownerの周期は通常MPCの`u / acceleration`を破棄し、
  同じC++ nodeの既存publisherからRecoveryまたはSafeStopだけをpublishする。
- 後退は最大0.8 m / 2.0 s / 0.8 m/s / 1 attemptとし、速度上限はcoreと
  command adapterの二重防護で停止側へ倒す。
- Left / Rightの後退自転車modelとstatic safetyはテスト可能なAPIとして実装したが、
  runtime候補選択、実操舵command、RViz表示は未実装である。
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

最初の実制御はReverseStraightだけを扱う。

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

現runtimeはこのうちStraightだけをstatic footprint、V2X、Boost、gearのhard conditionへ
接続して実commandへ変換する。Left / Rightのruntime選択は未実装である。

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
  Left / Right実制御、RVizは未検証または未実装。
