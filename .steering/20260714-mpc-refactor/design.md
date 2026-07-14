# MPC 疎結合化 設計

- 作成日: 2026-07-14
- 最終更新: 2026-07-15（補正レビュー反映）
- 状態: Draft
- 前提: 開始条件は `Contract/Safety Floor`、`Scoped Solver Baseline`、`Scoped Path Baseline`、`Full Baseline v1`、`Final Full Verification` の名称に統一する。各 Phase は開始条件を満たし、変更対象に対応する verification slice を前後比較する

## 1. 現状認識

現行 `mpc_controller_cpp.cpp` は約 9,000 行の単一 translation unit で、ROS I/O、設定解決、base path、V2X、behavior FSM、local reference、QP 構築、OSQP、制御後処理、Recovery、gear、Boost、fail-safe、publish が同じ object graph と mutable state を共有している。

特に次の境界が曖昧である。

- `MPC::init_problem()` が behavior 判断、reference 生成、QP 構築を同時に行う。
- solver failure が同周期の fallback command と、次周期の追越し Recovery feedback の両方を暗黙に変更する。
- `ReferencePath` が immutable な経路と周期ごとの速度・制約を同じ public vector で持つ。
- ROS node の 1 周期処理が複数の `update_*()` と `get_control()` の呼出順に依存する。
- nominal MPC、Recovery、fail-safe、Boost、gear の最終優先順位がコード順に埋め込まれている。

最初から node/package を分割すると、ROS timing、QoS、launch、提出物まで同時に変わる。そのため、まず同一 process・同一 executor・同一 publisher の中で pure C++ 境界を作る。

## 2. 設計原則

1. **外側を維持し、内側を置き換える。** `mpc_controller_cpp` を互換 façade とする。
2. **1 Phase = 1 責務。** 構造変更と制御ロジック変更を混ぜない。
3. **入力を snapshot にする。** callback が書き換える state を core が直接読まない。
4. **副作用を端へ寄せる。** subscribe/parameter/time/log/publish は ROS adapter に閉じる。
5. **判断を値として渡す。** behavior、solver、arbitration の reason/status を隠さない。
6. **単一の最終 publisher。** core component は ROS publish しない。
7. **不変条件を型と constructor で守る。** 長さ、単位、frame、有限値、左右境界を検証する。
8. **旧経路との二重運用を残さない。** test-only oracle comparison は許可するが production の長期 dual path は作らない。
9. **検証範囲は変更対象に比例させる。** 無関係な AWSIM scenario の不調だけを理由に pure component の mechanical extraction を止めず、変更した責務に必要な fixture と `Contract/Safety Floor` は省略しない。

## 3. 目標 dependency

```text
ROS callbacks / flat YAML / environment
                  |
                  v
        MpcControllerCpp (compatibility facade)
        - RosInputAdapter / ConfigCompatibilityLoader
        - CycleAdapter / CycleSnapshotIdentityProvider
        - SessionState / V2xSnapshotBuilder / BasePathStore
                  |
                  v
      CycleInput + SnapshotIdentity + owned immutable snapshots
                  |
                  +--------------------------+-----------------------------+
                  |                          |                             |
                  v                          v                             v
     BasePathView + V2xSnapshot  BasePathView + V2xSnapshot   Session/config
                  |                          |                             |
                  v                          v                             v
       FrontRiskClassifier         CorridorProposalBuilder    OperationalLimitResolver
                  |                          |                             |
                  v                          v                             v
       FrontRiskAssessment         CorridorProposalSet          OperationalLimits
                  |                          |
                  +------------+-------------+
                               v
                         BehaviorInput
                               |
                               v
                    RaceBehaviorPlanner
                               |
                               v
                      BehaviorSelection
                               |
                               +-------------------+
                               |                   |
                               v                   |
                  OvertakeLinePlanner              |
                      ^        |                   |
                      |        v                   |
 OvertakeExecutionFeedback(t-1)  OvertakeLineDecision
                                        |          |
 CorridorProposalSet -------------------+----------+
                                                   |
                                                   v
                                  LocalCorridorPlanner::commit
                                                   |
                                                   v
                                            CorridorPlan
                                                   |
 BasePathView + OperationalLimits + BehaviorSelection
                                                   |
                                                   v
                                       LocalReferenceBuilder
                                                               |
                                                               v
                                                       ReferenceHorizon
                                                               |
                                                               v
                                                       MpcProblemBuilder
                                                               |
                                                               v
                                                          MpcProblem
                                                               |
                                                               v
                                                            QpSolver
                                                               |
                                                               v
                                                        MpcSolveResult
                              preflight/problem/conversion failure ------+
                                                                       |
                                                                       v
                                                 MpcExecutionAdapter::execute
                                                                       |
                                                                       v
                                                        MpcExecutionStepResult
                              +-------------------+-------------------+----------------------+
                              |                   |                   |                      |
                              v                   v                   v                      v
                 MpcExecutionOutcome   raw nominal control   FallbackCandidate   SolverExecutionFeedback(t)
                      (trace)                    |              (same cycle)          (store for next cycle)
                                                 v                   |
                                    ControlPostProcessor             |
                                                 |                   |
                                                 v                   |
                                      NominalMpcCandidate            |
                                                 +---------+---------+
                                              |
        Recovery/Boost/Gear candidates -------+---- PrevalidatedSafeStop
                                              |
                                              v
                                      SafetySupervisor
                                              |
                                              v
                                      EligibleCandidates
                                              |
                                              v
                                       CommandArbiter
                                              |
                                              v
                                     SelectedCandidate
                                              |
                                              v
                                  FinalCommandValidator
                              +-------+---------------+----------------+
                              |                       |                |
                              | valid                 | invalid once   | SafeStop invalid
                              v                       v                v
                        FinalCommand        PrevalidatedSafeStop   FatalSafetyFault
                              |                       |                |
                              +-----------+-----------+   context-matched GuaranteedTerminalStop only
                                          |                            |
                                          v                            |
                       RosOutputAdapter (sole publisher) <-------------+
```

`CorridorProposalBuilder` は同じ `CycleInput` から左右/低速回避候補を一度だけ作る pure component とする。`RaceBehaviorPlanner` は proposal の summary を使って状態と pass side を選び、`LocalCorridorPlanner::commit` は選択済み proposal に continuity/side lock を適用する。`commit` で V2X 再射影、free interval 再計算、別 proposal の生成を行ってはならない。`CycleInput`、`CorridorProposalSet`、`BehaviorSelection`、`CorridorPlan` の `SnapshotIdentity` と proposal ID は exact match を必須とし、不一致は invalid input として SafeStop 経路へ送る。

`MpcExecutionAdapter::execute()` は raw attempt と previous state から `MpcExecutionStepResult` を一度だけ返す。normalized fact の `MpcExecutionOutcome`、同周期の raw nominal control / `FallbackCandidate`、次周期用の `SolverExecutionFeedback` を別 field にし、Outcome を adapter へ再入力しない。周期末の唯一の commit site は `MpcExecutionAdapter::project_overtake_feedback()` を一度だけ呼び、solver 固有情報を除いた `OvertakeExecutionFeedback` を次周期の `OvertakeLinePlanner` または corridor continuity owner だけへ渡す。commit-site owner は Phase 2B-5 の shadow coordinator、Phase 2B-6 の compatibility façade、Phase 3 の one-cycle orchestrator の順に明示移管し、二重 call site を残さない。現行は failure 後に同周期の behavior/reference を再生成しないため、2-pass pipeline や component 間の暗黙な呼戻しは導入しない。

`RecoveryPolicy`、`BoostPolicy`、`GearPolicy` は候補/action を返す pure policy とし、独立 publisher にはしない。`SafetySupervisor`、`CommandArbiter`、`FinalCommandValidator` は別 component とし、最終 publish は常に既存 `RosOutputAdapter` 一つから行う。

全 Phase で既存 node/executable、40 Hz timer、`SingleThreadedExecutor`、launch/config key、topic/service/type/QoS、Domain、publisher ownership を変えない。特に `/control/command/control_cmd`、`/localization/kinematic_state`、`/planning/scenario_planning/trajectory`、`/set_initial_pose` と AWSIM/V2X contract は外部 I/O invariant である。`autostart_orchestrator_node` が `/admin/awsim/start` に触れないこと、`awsim_state_manager_node` が `/awsim/state` を消費しないこと、`admin_start_once: true`、状態文字列、result schema、`output/latest/` / UID ownership も負方向・成果物 invariant として維持する。Boost に `/awsim/boost_cmd` / `Bool` / Domain 0 admin topic を、gear/Recovery に `/admin/awsim/reset` / cross-domain / teleport / respawn を代用せず、submit tar はリポジトリ直下の build context 内に置く。

## 4. Component responsibilities

### 4.1 `MpcControllerCpp`

- 既存 node/executable/topic/parameter/launch の互換 façade
- Phase 3 までは callback 更新候補を façade の既存 pending state に保持する。Phase 3 以降、path/V2X/session の ROS callback は raw input を各 domain owner へ渡し、owner が validation 済み immutable candidate/ref と accepted version を発行する。config callback は complete config candidate を cycle adapter へ渡す。active aggregate、config epoch increment、周期先頭 swap は cycle adapter 一つに移す
- flat YAML と dynamic parameter を compatibility loader で解決し、各 component 専用 typed config に変換
- 40 Hz timer と `SingleThreadedExecutor` を維持
- diagnostics/debug topic と最終 command の ROS publish
- core に ROS message を渡さず、core の値型を ROS message に変換
- 公式 `/awsim/control_mode_request_topic` の新規 pub/sub を追加せず、legacy 内部 enable input と別責務のまま扱う

### 4.1.1 `CycleSnapshotIdentityProvider`

- Phase 2B の先頭で façade 内に抽出し、`begin_cycle()` で cycle ID と ROS/steady cycle time を timer callback 冒頭に一度だけ採取する。`seal_identity()` は active snapshot commit 後の version/source stamp を受ける
- `BasePathStore`、config epoch、V2X epoch、session epoch の各 owner が供給する version と source stamp を一つの `SnapshotIdentity` に束ねるだけとし、callback state、YAML、環境変数を自ら読まない
- Phase 2B-0 では façade が config/session/V2X input epoch を保持し、accepted update/event のときだけ進める。Phase 2B-1 で V2X epoch ownership を `V2xSnapshotBuilder` へ一度だけ移し、二重 increment を残さない
- Full `CycleInput -> CycleOutput` や config compatibility mapping の整理は導入せず、Phase 3 がこの provider と既存 version owner を一周期 API / snapshot commit へ統合し、Phase 5 が同じ commit API 背後の flat→typed mapping だけを整理する

### 4.2 `BasePathStore`

- CSV/topic 由来の base path、補間、平滑化、曲率、base velocity、track width を保持
- base path は load/update 単位の immutable snapshot とし、`shared_ptr<const BasePath>` と単調増加する version を返す
- 周期ごとの速度上書き、V2X corridor、overtake offset を保持しない
- 現行 CSV schema と circular/open semantics を維持

### 4.3 `V2xSnapshotBuilder`

- `v2x_msgs` を core 用の有限値・時刻付き snapshot に変換
- vehicle ID、freshness、source stamp、観測順を明示
- ID 欠落や重複を隠さず validation result として残す
- gap/behavior 戦術は担当しない

### 4.4 `FrontRiskClassifier`

- ego projection、base path curvature、`V2xSnapshot` から front/side target、distance、relative speed、required deceleration、TTC、risk level、SafetyBrake request を計算
- `FrontRiskAssessment` を返す pure component とし、V2X tracker history、Behavior state、corridor target、QP を変更しない
- missing/stale/invalid track と curve guard の判定理由を隠さず返す

### 4.5 `CorridorProposalBuilder`

- `BasePathView`、ego projection、`V2xSnapshot`、時刻、base corridor から、`Base`、`FollowPreposition`、`LowSpeedAvoidance`、`OvertakeLeft/Right`、`OvertakeFallbackLeft/Right` の `CorridorProposalSet` を一度だけ生成
- horizon 点への他車予測、occupied/free interval、gap 幅、到達可能性、候補ごとの bounds/target、reject reason を担当
- behavior state、state hold、overtake phase、solver failure、side lock を知らない
- proposal 生成中に tracker/continuity state を変更しない pure component とする

### 4.6 `RaceBehaviorPlanner`

- Cruise、Follow、Overtake、LowSpeedAvoidance、SafetyBrake の現在の状態遷移
- `CorridorProposalSet` の feasibility summary と front-risk assessment から target vehicle、pass side、desired speed、stop request を選択
- 完成済み corridor vector を生成せず、選んだ proposal ID と policy を `BehaviorSelection` に残す
- V2X ROS message、publisher、Domain env、QP matrix を知らない
- ego projection、`FrontRiskAssessment`、`CorridorProposalSet` summary、時刻、previous `BehaviorPlannerState` を `BehaviorInput` で受ける。raw path/V2X、solver feedback は直接受けない

### 4.7 `OvertakeLinePlanner`

- `BehaviorSelection`、前周期の `OvertakeExecutionFeedback`、target continuity、現在の lateral state から `Idle/FollowPrepare/ShiftOut/Pass/Return/Recovery` phase を更新。戦術上の target/pass-side 選択は再決定せず、選択済み target/side に対する phase/continuity lock だけを所有する
- selected proposal の bounds/target metadata と base-path lateral geometry だけから、phase に対応する horizon target array/active mask を `OvertakeLineDecision` に生成する。V2X/gap/free interval は再計算しない
- solver failure threshold 到達は次周期だけ反映し、Recovery phase/速度制限/target transition request を返す
- QP、OSQP、ROS I/O を知らず、同周期の再 solve や reference rebuild を要求しない

### 4.8 `LocalCorridorPlanner`

- `BehaviorSelection`、`OvertakeLineDecision` と同じ `CorridorProposalSet` から選択済み `CorridorPlan` を commit
- side/target continuity と overtake line target を適用するが、車両再予測、free interval 再計算、代替 proposal の生成はしない
- proposal ID と `SnapshotIdentity` の一致を検証し、不一致を invalid result として返す
- base path、behavior、proposal の state を直接 mutation しない

### 4.9 `OperationalLimitResolver`

- Domain 別 `v_max`、Start 後の時間窓、`ref_vel.yaml` section、base speed 上限を現在の順序で解決
- `BehaviorSelection` の追従/制動速度とは別の `OperationalLimits` を返す
- ROS parameter、環境変数、base path vector を直接変更しない

### 4.10 `LocalReferenceBuilder`

- base path、`BehaviorSelection`、`CorridorPlan`、`OperationalLimits` から、その周期だけの horizon を作る
- target velocity、lateral target、左右 corridor、dynamic constraint を現行の優先順で合成
- base path を mutation しない
- V2X projection、gap 選択、behavior state machine、solver を知らない

### 4.11 `MpcProblemBuilder`

- vehicle state、model/config、完成済み `ReferenceHorizon` から `P/A/q/l/u` を作る
- V2X、lap、vehicle ID、overtake phase、ROS time を知らない
- solver を呼ばず、sparse problem と metadata を返す

### 4.12 `OsqpBackend / QpSolver / ReferenceSpeedProfileOptimizer`

- `OsqpBackend` は matrix/vector-in、result-out の stateless component とし、online MPC と reference speed profile の両方から利用する
- `QpSolver` は `MpcProblem` を backend 入力へ変換し、OSQP の解、status、iteration、constraint violation を `MpcSolveResult` として返す
- `ReferenceSpeedProfileOptimizer` は path speed QP を構築して同じ backend を使い、初期 path と topic 更新 path の現行 failure semantics を維持
- accepted status、settings、validation は現行 semantics を維持
- behavior state や failure counter を直接変更しない

### 4.13 `MpcExecutionAdapter`

- horizon/preflight failure、problem-build exception、`MpcSolveResult` を discriminated `MpcExecutionAttempt` として `execute()` に受け、solver-to-control conversion とその MPC-internal non-finite failure まで `MpcExecutionOutcome` へ正規化する
- fallback deceleration/speed evolution、failure threshold、accepted execution category だけを持つ `MpcExecutionConfig` を受ける
- previous `SolverExecutionState`、同周期に solve 前に確定した `OvertakeLineDecision.phase`、vehicle speed、fallback config から、Outcome、raw nominal control または同周期 fallback candidate、`SolverExecutionFeedback`、next state を別 field にした `MpcExecutionStepResult` を一度だけ返す
- target architecture では failure counter、overtake 中の failure counter、fallback speed、last result category の唯一 owner とする。Phase 2B-5 shadow 中は shadow `SolverExecutionState` だけを所有し、legacy production state の ownership は Phase 2B-6 cutover まで変更しない。`QpSolver`、`RaceBehaviorPlanner`、`OvertakeLinePlanner` はこれらを直接変更しない
- arbitration 前に実行し、arbitration result を feedback の生成元にしない。同周期 callback、reference rebuild、second solve は要求しない
- `project_overtake_feedback()` は `SolverExecutionFeedback` から solver 固有情報を除く唯一の projection 関数とする。Phase 2B-5 shadow coordinator、2B-6 compatibility façade、Phase 3 one-cycle orchestrator のうち、その時点の唯一の cycle-tail owner だけが一度呼ぶ

### 4.14 `ControlPostProcessor`

- solver solution から nominal acceleration/steering candidate を作る
- acceleration conversion/filter、steering gain/filter/limit の現行順序を担当
- publish、Boost、Recovery、gear の判断はしない

raw command と最終 command を別の観測点として維持する。steering gain/filter/limit の適用順は Phase 0 trace から固定し、raw 側の limit を final 側へ機械的に流用しない。

### 4.15 `SafetySupervisor`

- stale/incomplete/non-finite input、solver failure、control disabled、stop request、Recovery/gear/Boost の system-level inhibit と mandatory stop を判定
- Nominal/Fallback/Recovery/SafeStop 候補ごとの eligibility と reject reason を返す
- source の優先順位を選ばず、ROS publish も行わない
- fatal condition を latch し、以後の nominal/Boost/gear action を許可しない

### 4.16 `CommandArbiter`

- `SafetySupervisor` が許可した候補から Phase 0 で固定した優先順位に従って一つを選択
- 優先順位、selected source、inhibited reason を `ArbitrationDecision` として返す
- 候補値の有限値/range/rate 検証や ROS publish は行わない

### 4.17 `FinalCommandValidator`

- selected command の finite、steering angle/rate、acceleration、gear/Boost invariant を最終確認
- `HardSafetyCommandValidator` を再利用し、SafeStop の事前検証と final validation で hard-limit 判定実装を二重化しない
- selected command が不正な場合は、周期開始時に独立検証済みの同じ `PrevalidatedSafeStop` へ一度だけ置換する
- SafeStop を再帰的に生成・再 arbitration しない。prebuilt SafeStop も成立しない場合は `FatalSafetyFault` event を `FatalSafeStopValidationFailure` reason で返す
- validator 自身は state を mutation しない。one-cycle orchestrator が event を周期末に一度だけ `SafetySupervisorState` へ commit し、fatal latch の唯一 owner は `SafetySupervisor` とする
- ROS publish は行わない

### 4.18 `SafeStopFactory / RecoveryPolicy / BoostPolicy / GearPolicy`

- startup 時に forward/reverse/unknown gear context を含む `ValidatedHardSafetyLimits` を検査し、その immutable 値だけを受ける SafeStop template/factory と pure `HardSafetyCommandValidator` を構築する。通常 config や reject 済み candidate を生成元にしない
- `SafeStopFactory` は各周期の nominal pipeline より前に、検証済み template、current validated gear context、直前に実際に publish された gain 適用後 final steering だけから `PrevalidatedSafeStop` を一度だけ作り、`HardSafetyCommandValidator` で事前検証する
- policy は自身の input/state から候補/action と reason を返し、他 policy や publisher を呼ばない
- startup の hard-safety limit / SafeStop template validation が成立しない場合は走行開始前に起動失敗とする。startup 時点だけで context-specific な `GuaranteedTerminalStop` が成立したとはみなさない
- runtime の `FatalSafetyFault` では nominal/Recovery/Boost/gear を停止する。`GuaranteedTerminalStop` は直前 final command identity と validated gear-context version に対する有効性を別 safety contract で証明し、current context と exact match する場合だけ publish 候補にできる。過去の SafeStop を無条件再利用せず、保証済み stop が存在しない場合は invalid command を合成せず fatal error で停止する

### 4.19 Test-only `LegacyReplayHarness`

- production node の private state snapshot を保存しない
- constructor/reset の clean state から、明示時刻付き input/event prefix を順番に replay する
- callback 順、timer step、parameter update、V2X/session event を fixture schema で表現する
- legacy 1-cycle seam から behavior/reference/QP/solver/raw/final output を収集する
- 同じ fixture の反復で正規化 output が決定的であることを確認する
- production launch/runtime path には組み込まない

各 component は抽出 Phase で必要最小限の component-local typed config を導入する。巨大な `MpcConfig` / `Config` 全体を pure component に渡さない。Phase 5 はこれらの型を新設する Phase ではなく、flat YAML / dynamic parameter から既存の typed config 群を組み立てる compatibility loader を整理する Phase とする。

## 5. Value objects

### 5.1 Snapshot ownership と identity

`BasePathSnapshotRef` と `ConfigSnapshotRef` は `shared_ptr<const T>` と単調増加 version を持つ。`V2xSnapshot` と session snapshot は cycle ごとの値 snapshot と version を持つ。

```cpp
struct SnapshotIdentity
{
  uint64_t cycle_id;
  uint64_t path_version;
  uint64_t config_version;
  uint64_t v2x_version;
  uint64_t session_version;
  int64_t cycle_ros_time_ns;
  int64_t cycle_steady_time_ns;
  std::optional<int64_t> path_source_time_ns;
  std::optional<int64_t> v2x_source_time_ns;
};
```

ownership/lifetime 規則:

- ROS callback は active object を直接変更しない。Phase 3 より前は façade の既存 pending state を更新する。Phase 3 以降、path/V2X/session の raw input は各 domain owner が検証し、validation 済み immutable candidate/ref だけを cycle adapter へ渡す。config callback だけは complete config candidate を cycle adapter へ渡し、adapter が schema/coherence を検証する。
- Phase 3 以降の pending/active **aggregate refs**、config epoch、config snapshot commit は cycle adapter だけが所有し、façade 側の旧 increment/swap site は削除する。path/V2X/session の domain validation、accepted-version increment、last-known-good は各既存 owner に残す。
- `SingleThreadedExecutor` の制御 timer callback 先頭では、(1) `begin_cycle()` で cycle ID/ROS・steady time を採取、(2) 各 owner が発行済みの validation 済み immutable candidate/ref と、schema/coherence が成立する complete config candidate を source ごとに commit、(3) active path/config/V2X/session の version/source stamp を読取、(4) `seal_identity()` と `CycleInput` を確定、の順に一度だけ実行する。周期途中の交換や identity 再生成は行わない。
- invalid path/V2X/session candidate はその owner の last-known-good だけを維持する。invalid complete config candidate は cycle adapter が reject し、last-known-good config snapshot/version を維持する。どちらも他 source の valid pending update を rollback せず、全 source を一括成功/失敗させる global transaction は導入しない。
- construction 時に `SnapshotIdentity` の各 version と対応する active snapshot version を exact 比較し、特に `identity.config_version == ControllerConfigSnapshot.version` を必須にする。
- `CycleInput` が snapshot owner を周期終了まで保持し、`BasePathView` などの非 owning view はその owner より長生きしない。
- path/config/V2X が同時更新でなくても、周期先頭に取得した version 組を一つの `SnapshotIdentity` として固定する。
- 現行の `SingleThreadedExecutor` では callback と timer の並行 mutation を前提にしない。将来 worker thread を導入する場合だけ C++17 互換の `atomic_load/store(shared_ptr)` または明示 mutex を使い、外部 I/O/timing を別変更として再評価する。
- `CycleOutput`、proposal、selection、corridor、fixture/trace は同じ `SnapshotIdentity` を保持する。

### 5.2 `CycleInput`

- ROS time と monotonic time
- vehicle state と freshness
- owned immutable base path/config snapshot
- V2X snapshot
- `awsim_vehicle_state`
- `local_controller_enabled`
- `local_stop_requested`
- session state
- previous `OvertakeExecutionFeedback` / arbitration feedback。raw `SolverExecutionFeedback` は cycle input へ公開しない
- Domain/vehicle の解決済み context
- `SnapshotIdentity`

周期途中の parameter callback はこの object を変更せず、次の周期で新しい config snapshot として反映する。複数 parameter の更新は一つの config version として commit し、部分更新を core に見せない。

`local_controller_enabled` は現行 legacy `/control/control_mode_request_topic` 由来、AWSIM engage は公式 `/awsim/control_mode_request_topic` 由来で責務が異なる。二つを remap、統合、相互代用しない。

### 5.3 `FrontRiskAssessment`

- front/side target ID と classification reason
- longitudinal/lateral distance、relative speed、required deceleration、TTC
- risk level、SafetyBrake request、validity/reject reason
- originating `SnapshotIdentity`

### 5.4 `BehaviorInput`

- ego state と base path 上の projection/progress
- `FrontRiskAssessment`
- `CorridorProposalSet` の feasibility/reachability summary
- ROS/monotonic time
- previous `BehaviorPlannerState`
- `SnapshotIdentity`

### 5.5 `CorridorProposalSet`

- set ID、各 candidate の immutable proposal ID、`SnapshotIdentity`
- base waypoint ID、horizon size、created ROS/steady time
- `Base`、`FollowPreposition`、`LowSpeedAvoidance`、`OvertakeLeft/Right`、`OvertakeFallbackLeft/Right` 候補ごとの feasibility、reject reason、pass side
- horizon ごとの candidate lower/upper/target と active mask
- gap 幅、準備距離/時間、必要横加速度など Behavior が使う assessment summary

proposal は一周期に一度だけ生成する。`Base` proposal は常に明示 ID を持ち、Cruise/SafetyBrake/候補不要時にも `selected_proposal_id` を optional や magic value にしない。assessment は summary だけを読み、commit は選択済み候補を再計算せず参照する。

### 5.6 `BehaviorSelection`

- state と reason
- target vehicle ID、selected proposal ID、pass side
- desired speed と speed limit
- overtake/low-speed policy と SafetyBrake/停止 request
- proposal と一致する `SnapshotIdentity`
- diagnostics

完成済み corridor vector と overtake line の全 target 列は持たない。stale odometry、local stop request、invalid solver output など system-level ForcedStop は `BehaviorSelection` に入れず、`SafetySupervisor` が判断する。

### 5.7 `OvertakeLineDecision / CorridorPlan`

`OvertakeLineDecision` は selected proposal ID、`SnapshotIdentity`、phase、target/side continuity、Recovery request、velocity limit、horizon target array/active mask、理由を持つ。`CorridorPlan` は commit 済みの selected proposal ID、`SnapshotIdentity`、horizon ごとの lower/upper/target、active mask、speed limit、reject reason を持つ。

`LocalCorridorPlanner::commit` は proposal/selection/decision の identity と selected proposal ID を exact 比較する。不一致、欠落、サイズ不整合を別候補の再探索で隠さない。

### 5.8 `OperationalLimits`

- Domain/session/section 解決後の速度上限
- 適用理由と有効期間
- 必要な acceleration/lateral limits
- originating `SnapshotIdentity`

behavior 由来の target speed と設定/運用由来の上限を別 field に保ち、`LocalReferenceBuilder` で現行の順序どおり合成する。

### 5.9 `ReferenceHorizon`

各 sample に、少なくとも次を同じ index で持たせる。

- `s_m`, `x_m`, `y_m`, `yaw_rad`, `curvature_radpm`
- `speed_mps`
- `lateral_lower_m`, `lateral_upper_m`, `target_lateral_m`

artifact 全体に `SnapshotIdentity`、selected proposal ID、base waypoint ID を持たせる。

全 field の sample 数一致、有限値、frame、単位、`lower <= upper` を生成時に検証する。`target` が corridor 内であることを必須 invariant にするかは Phase 0 の現行値から判断し、リファクタ時に新しい clamp を暗黙追加しない。現行 builder が必要とする sample 数も Phase 0 fixture から固定する。

### 5.10 `MpcProblem`

- sparse `P`, `A`
- `q`, `l`, `u`
- dimensions、horizon/model metadata
- originating `SnapshotIdentity`、selected proposal ID、base waypoint ID

疎行列は内部の triplet 挿入順や Eigen storage order を oracle にしない。比較時に `(row, column, value)` へ展開して row/column で canonical sort し、次を分けて判定する。

- exact: dimensions、nonzero 座標集合、upper/lower triangular policy、変数 ordering、constraint row semantics
- tolerance: canonical 座標に対応する value と `q/l/u`

### 5.11 `MpcSolveResult / MpcExecutionAttempt / MpcExecutionStepResult / MpcExecutionOutcome / SolverExecutionFeedback / OvertakeExecutionFeedback`

- `MpcSolveResult`: normalized/raw status、solution/control sequence、iteration、objective、最大制約違反、validity、failure reason、originating `SnapshotIdentity`
- `MpcExecutionAttempt`: `PreflightFailure` / `ProblemBuildFailure` / `SolverResult(MpcSolveResult)` の discriminated variant と originating `SnapshotIdentity`。candidate、counter、fallback state は持たない
- `MpcExecutionOutcome`: success/failure、`Preflight` / `ProblemBuild` / `SolverSetup` / `SolverStatus` / `ControlConversion` / `MpcInternalNonFinite` category、raw solve result、originating cycle ID/`SnapshotIdentity`。candidate と mutable state は持たない
- `MpcExecutionStepResult`: `MpcExecutionOutcome`、success 時の raw nominal control または failure 時の同周期 fallback candidate、`SolverExecutionFeedback`、next `SolverExecutionState`。相互排他 field と identity を construction 時に検証する
- `SolverExecutionFeedback`: originating cycle ID/`SnapshotIdentity`、success/failure、連続 failure count、overtake 中の failure count/threshold edge、fallback source/reason
- `OvertakeExecutionFeedback`: `previous_plan_failed`、`recovery_requested`、originating cycle ID/`SnapshotIdentity` だけを持つ戦術向け projection。OSQP status、solution、raw counter は持たない

`MpcExecutionAdapter::execute()` が arbitration 前に raw attempt、previous `SolverExecutionState`、同周期の `OvertakeLineDecision.phase` から `MpcExecutionStepResult(t)` を一度だけ生成する。cycle `t` の behavior/reference は変更しない。その Phase の唯一の cycle-tail owner は周期末に `project_overtake_feedback()` を一度だけ呼び、cycle `t+1` の `OvertakeLinePlanner` または corridor continuity owner だけが読む。`RaceBehaviorPlanner` は raw solver feedback を受けない。

### 5.12 `ControlCandidate / SafetyAssessment / ArbitrationDecision / FinalCommandResult`

- raw/filtered control と validity
- source (`NominalMpc`, `Fallback`, `Recovery`, `SafeStop`)
- Boost/gear action
- safety eligibility、mandatory stop と reject reason
- selected source、arbitration reason と inhibited reason
- final command、SafeStop substitution の有無、fatal terminal reason
- `SnapshotIdentity` と cycle ID

## 6. State and feedback

すべてを stateless にする必要はない。FSM、filter、failure counter、Boost start-once、Recovery phase は stateful である。ただし state は component ごとに分け、所有者を一つにし、周期の先頭と末尾で state transition を明示する。

| State | Owner | 主な内容 |
|---|---|---|
| `CycleIdentityState` | `CycleSnapshotIdentityProvider` | next cycle ID。ROS/steady cycle time は各周期一度だけ採取し、他 component は再生成しない |
| `ConfigEpochState` | Phase 3 cycle adapter（それまでは compatibility façade） | accepted config update ごとの単調 version。Phase 3 cutover で owner を一度だけ移し、immutable config snapshot と同じ version を使う |
| `SessionState` | compatibility façade | AWSIM session/start epoch、control enable、odometry/session latch |
| `V2xTrackerState` | `V2xSnapshotBuilder` | vehicle history、source/receipt stamp、ID/complete validation、accepted snapshot version |
| `BehaviorPlannerState` | `RaceBehaviorPlanner` | Cruise/Follow/Overtake/LowSpeedAvoidance/SafetyBrake、hold timestamp、target selection |
| `OvertakeLinePlannerState` | `OvertakeLinePlanner` | 選択済み target/side に対する phase/continuity lock、phase start、rear-clear confirmation、Recovery latch。戦術上の target/pass-side 選択は持たない |
| `LocalCorridorPlannerState` | `LocalCorridorPlanner` | selected side/target continuity。V2X track historyは持たない |
| `MpcControlHistoryState` | one-cycle orchestrator / problem-builder boundary | previous control horizon、previous steering、last solved waypoint、prediction linearization history |
| `SolverExecutionState` | solver execution/fallback adapter | consecutive failure、overtake failure count、fallback speed、last result category |
| `PostProcessorState` | `ControlPostProcessor` | acceleration/steering filter の previous value |
| `BoostPolicyState` | `BoostPolicy` | start-once、re-arm、inhibit latch |
| `RecoveryPolicyState` | `RecoveryPolicy` | Recovery phase、attempt、distance/evidence、fault latch |
| `GearPolicyState` | `GearPolicy` | requested/reported gear、transition/timeout state |
| `SafetySupervisorState` | `SafetySupervisor` | mandatory-stop/fatal latch、current `GuaranteedTerminalStop` identity/gear-context validity |

`ControllerState` は façade/orchestrator がこれらを束ねるための aggregate に限る。各 component は `ControllerState` 全体ではなく、自身の previous substate と明示的 input/feedback だけを受け、自身の next substate だけを返す。cross-component state の参照や mutable alias を禁止する。

`FinalCommandValidator` は `FatalSafetyFault` event を返すだけで latch を所有しない。one-cycle orchestrator は final result を確定した後、同 event を `SafetySupervisorState` の next state に一度だけ commit する。次周期以降の terminal stop / inhibit 判断は `SafetySupervisor` が行い、validator、arbiter、policy が fatal latch を直接変更しない。

現行 `solve_osqp()` は周期ごとに OSQP workspace を作るため solver warm start state はない。旧 control sequence は QP linearization/rate constraint の入力であり、`MpcWarmStartState` とは呼ばず `MpcControlHistoryState` とする。将来の OSQP warm start 導入は性能・数値挙動変更として別計画にする。

```text
previous component states + CycleInput
  -> component decisions/results + next component states
  -> CycleOutput + SnapshotIdentity
```

### 6.1 Solver failure の周期 semantics

static code reading が示す現行 semantics は「同周期 fallback、追越し Recovery は次周期」である。Phase 0 の D-11 fixture で順序を確認し、差があれば Full Baseline v1 承認前に本節を更新する。

```text
cycle t:
  Behavior/Proposal/Reference/QP
    -> preflight/problem/solve/control-conversion failure
    -> solve 前に確定済みの OvertakeLineDecision.phase と raw attempt を execute() へ渡す
    -> MpcExecutionStepResult が Outcome/FallbackCandidate/Feedback/next state を一度だけ返す
    -> FallbackCandidate を同周期の arbitration へ渡す
    -> next SolverExecutionState と SolverExecutionFeedback(t) を保存
    -> 周期末に project_overtake_feedback() を一度だけ呼ぶ
    -> OvertakeExecutionFeedback(t) を次周期用に保存
    -> Behavior/Proposal/Reference は再実行しない

cycle t+1:
  OvertakeExecutionFeedback(t)
    -> OvertakeLinePlanner
    -> threshold 到達時に OvertakeLine Recovery / target suppression
```

failure threshold に達しても同周期に `RaceBehaviorPlanner`、`LocalCorridorPlanner`、`LocalReferenceBuilder` を呼び戻さず、second solve も行わない。`BehaviorPlannerState` と `OvertakeLinePlannerState` を混同せず、現行の solver failure が変更するのは next-cycle の `OvertakeExecutionFeedback` として表現する。Phase 0 deterministic fixture ではこの既知 semantics を再確認し、未確定の設計分岐として扱わない。

## 7. Current arbitration characterization

Phase 4C で優先順位を再設計してはならない。Phase 0 で現行 `control()` と publish 前後を調べ、canonical runtime、deterministic synthetic fixture、dormant pure-rule test を使い分けて次の競合を固定する。

| 競合 | evidence class | 固定する観測 |
|---|---|---|
| stale odometry vs nominal MPC | deterministic fault + live proxy | 最終 source、brake/steer、reason category |
| solver failure vs Overtake | deterministic cycle | 同周期 fallback、次周期 OvertakeLine feedback、failure count の順序 |
| control disabled/stop vs Boost | deterministic cycle | command と Boost inhibit/event 順序 |
| Recovery vs normal MPC | dormant pure-rule（canonical runtime は N/A） | takeover 条件、gear、最終 source |
| stale gear report vs Reverse | dormant pure-rule（canonical runtime は N/A） | Reverse 許可/拒否と safe fallback |
| non-finite postprocess | deterministic fault | 検出位置と最終 safe command |
| shutdown while Reverse | dormant pure-rule（canonical runtime は N/A） | gear/command 終了 sequence |

canonical config で Recovery/gear は disabled である。これを有効化した test-only simulation は補助 evidence にできるが、canonical runtime 等価性は主張しない。この表が期待値または正当な N/A で埋まるまでは、`CommandArbiter` の実装に着手しない。

## 8. Migration phases and staged baselines

### 8.1 Start-condition and verification policy

開始条件の名称と順序は全計画文書で次に統一する。`Contract/Safety Floor` は Phase 1〜5 のすべての構造リファクタに常時適用し、後段 baseline で代替しない。Phase 0b だけは、Contract Conformance lane で external-contract RED を既存正本へ適合させた後、Safety lane で確定済み oracle と登録済み hard-safety RED を是正する ordered remediation entry を経て、Floor 全項目を green にする。

| 開始条件 / 完了判定 | 適用範囲 | 必須 evidence |
|---|---|---|
| `Contract/Safety Floor` | Phase 1〜5 の全構造リファクタ | external contract exact、sole publisher、H-01〜H-08 に FAIL/UNRESOLVED なし、対象 identity、外部 I/O/timer/executor 不変 |
| `Scoped Solver Baseline` | Phase 1 開始前 | Online MPC と Reference Speed Profile の canonical QP/result、raw/fallback control、feasible/infeasible/non-finite/constraint violation、各 caller の failure semantics |
| `Scoped Path Baseline` | Phase 2A 開始前 | CSV/topic/circular/open、補間/平滑化/曲率/幅/base speed、valid/invalid update、last-known-good path、snapshot identity |
| `Full Baseline v1` | Phase 2B-0〜Phase 5 開始前 | 全 deterministic fixture、V2X/Behavior/corridor/reference/arbitration、Domain 1〜4、gate/eval/timing、全 identity |
| `Final Full Verification` | Phase 5 DoD 後 | Full Baseline v1 と同じ matrix、submit tar、eval image、`make eval`、最終 identity の再取得 |

baseline は比較元の承認済み evidence であり、各変更後に再実行する範囲は次の **verification slice** と呼ぶ。slice は新しい gate 名でも baseline version でもない。

| Verification slice | 主な適用 Phase | 比較対象 |
|---|---|---|
| Solver/QP slice | Phase 1 | Online MPC / Reference Speed Profile、canonical QP/result、各 caller の failure semantics |
| Solver Execution/Fault slice | Phase 2B-5〜2B-6 | Online MPC の preflight/problem-build/conversion/non-finite、同周期 fallback、counter/state owner、次周期 feedback |
| Path slice | Phase 2A | base path、trajectory update、last-known-good path、dynamic view |
| V2X/Behavior slice | Phase 2B-0〜2B-6 | cycle/snapshot identity、freshness/ID/history、front-risk、proposal、Behavior/OvertakeLine state、dev4 |
| Corridor/Reference slice | Phase 2C-1〜2C-2 | proposal/selection/commit identity、bounds/target、reference 全 field、no-recompute invariant |
| Problem slice | Phase 2D | canonical sparse structure、`P/A/q/l/u`、solver/behavior sequence |
| Cycle slice | Phase 3 | input-prefix replay、state ownership、snapshot boundary swap、40 Hz/timing envelope |
| Safety/Arbitration slice | Phase 4A〜4C | postprocess、fault/arbitration 競合、one-shot SafeStop、fatal fault、Boost/gear/Recovery、final invariant |
| Config/Release slice | Phase 5 | compatibility loader、CLI/launch/package、submit/eval preparation |

既存 external-contract RED または hard-safety RED は `Contract/Safety Floor` を block し、scoped baseline や verification slice の waiver にはできない。変更範囲が複数 slice にまたがる場合は必要 slice の和集合を再実行する。Phase 1/2A の scoped artifact は Full Baseline v1 へ自動昇格しない。

### Phase 0: Staged characterization / remediation

Phase 0a と observation step では制御ロジック、責務境界、外部契約を変更しない。Phase 0a で既存 topic/log だけの black-box evidence を取得し、続く observation step で recorder、比較器、test-only/no-op trace seam と `LegacyReplayHarness` を追加する。canonical production では seam を無効にする。Phase 0b は ordered remediation とし、まず Contract Conformance lane で既存 `docs/interface/` の exact target への実装適合だけを行って external-contract RED を 0 件にする。その後の Safety lane だけが、確定済み safety oracle と登録済み hard-safety RED に対する修正を行う。両 lane は独立 commit/review、before/after test、rollback を entry 条件とする。

bootstrap は次の 3 比較に分ける。

1. original v0 と seam compiled/disabled: contract exact、live envelope 非悪化
2. seam disabled と enabled: deterministic fixture の decision/output 一致
3. seam enabled の serialization/I/O overhead: 記録のみ。production performance gate には使わない

- `Scoped Solver Baseline` から順に、後続 Phase が必要とする baseline artifact、比較器、tolerance をレビュー・承認する
- `Scoped Solver Baseline` には online MPC だけでなく、startup/topic path update で同じ `solve_osqp()` を使う speed-profile fixture と failure semantics を含める
- config/document/contract drift を分類する
- 既存契約への適合または外部契約を変えない安全修正だけを `Phase 0b` の独立修正にする
- Candidate v0 の fault-time `publish_failsafe_command()` は R-13 を満たさないため、startup `ValidatedHardSafetyLimits`、pure `HardSafetyCommandValidator`、prebuilt `PrevalidatedSafeStop`、context-matched `GuaranteedTerminalStop`、one-shot validation、`FatalSafetyFault` を Phase 0b の独立安全修正として先に確立する
- observation seam/Phase 0b を含む確定 commit から、各 slice をその Phase の entry 前に再取得する
- 全 baseline artifact と Full matrix が揃った時点で aggregate を `Full Baseline v1` として承認する

### Phase 1: `OsqpBackend / QpSolver` extraction

Start condition: `Contract/Safety Floor + Scoped Solver Baseline`。

- 既存 `solve_osqp()` を generic matrix-in/result-out の `OsqpBackend` に mechanical move
- online `QpSolver` と `ReferenceSpeedProfileOptimizer` の両 call site を同じ backend へ接続
- `OsqpBackendConfig`、`QpSolverConfig`、`ReferenceSpeedProfileConfig` の必要最小 typed config を導入
- QP の作り方、OSQP settings/accepted status、validation、fallback/failure count、startup/path-update failure semantics を変更しない

Exit verification (Solver/QP slice):

- dimensions/nonzero coordinates/triangular policy/ordering/row semantics は canonical exact
- numeric `P/A/q/l/u` は canonical coordinate ごとの baseline tolerance 内
- online と speed-profile の accepted status、objective、最大違反、first control/prediction/output が baseline 内。full solution vector は補助比較とし、非一意解の要素一致を合格条件にしない
- build/package test/characterization replay が成功し、外部 I/O は不変

### Phase 2A: `BasePathStore` read-only boundary

Start condition: `Contract/Safety Floor + Phase 1 DoD + Scoped Path Baseline`。

- public mutable vector への直接書込みを adapter/store の内側へ閉じる
- immutable base path snapshot と 1 周期の dynamic view を分ける
- `BasePathConfig` を導入し、Phase 1 の `ReferenceSpeedProfileConfig` を path loader から明示的に渡す
- CSV/topic/circular/open の現行 semantics と valid/invalid update rollback を維持
- `shared_ptr<const BasePath>` owner、lifetime、version を実装し、consumer の direct mutation を禁止

Exit verification: Path slice が新 API で green、base path snapshot が周期前後で不変、外部 trajectory producer/consumer は不変。

### Phase 2B-0: `CycleSnapshotIdentityProvider` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2A DoD`。事前比較 fixture は Full baseline の cycle/snapshot identity trace。

- timer callback 冒頭で cycle ID と ROS/steady cycle time を一度だけ取得し、Phase 2A の path version と façade が持つ config/V2X/session epoch・source stamp を束ねる
- config/session/V2X epoch は accepted update/event だけで単調増加させ、同一周期中に変更しない。値の resolver、V2X validation、session FSM はこの component へ移さない
- `SnapshotIdentity` を shadow artifact と後続 component へ渡す最小 seam だけを導入し、full `CycleInput` と pending→active snapshot commit は Phase 3、flat→typed compatibility mapping の整理は Phase 5 に残す

Exit verification: 同じ clean initial state と event prefix で identity sequence が baseline trace と一致し、一周期内の全 shadow artifact が同じ identity を持つ。外部 I/O、timer、executor は不変。

### Phase 2B-1: `V2xSnapshotBuilder` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2B-0 DoD`。事前比較 fixture は V2X/Behavior slice の freshness/ID/history。

Phase 2B-1〜2B-5 の新 component は、各 subphase では test-only shadow path として legacy production cycle と並行比較する。shadow state は独立に保持し、command publish、legacy FSM/corridor state、ROS graph を変更しない。Phase 2B-6 の production cutover は in-process hot switch にせず、node restart と clean session/reset boundary から新 chain を初期化して同じ input prefix を replay する。shadow/legacy state の copy や途中昇格は行わない。

Phase 2B-6 では、現行の反映用再計算だけを明示する一時的な `LegacyCorridorCommitAdapter` を境界に置く。`apply(const LegacyCorridorCommitRequest &, const LegacyCorridorState &)` は同じ `CorridorProposalSet` / `BehaviorSelection` / `OvertakeLineDecision` / `SnapshotIdentity` と immutable view を受け、現行 `update_last_target=true` 相当の application pass だけを実行して corridor と next legacy corridor state を返す。legacy Behavior/OvertakeLine/evaluation pass、tracker、solver counter は呼ばず、mutable state は返却する next corridor state だけに限定する。adapter は Phase 2C-1 で削除し、それまでは現行 semantics を維持するための migration seam として扱う。

- ROS message adapter、track history、freshness/ID/duplicate/position-jump validation を抽出
- `V2xTrackerConfig` と `V2xTrackerState` を導入
- façade の V2X input epoch ownership を builder へ一度だけ移し、accepted snapshot update ごとに一回だけ version を進める
- gap、front-risk、Behavior、Recovery corridor を snapshot builder から除く

Exit verification: 同じ event prefix から同じ active track/validation/version が得られ、Domain 1〜4 の V2X input contract が不変。

### Phase 2B-2: Front classification / risk extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2B-1 DoD`。事前比較 fixture は V2X/Behavior slice の front/side/danger/risk。

- ego/path projection、front/side classification、required decel/TTC/risk level を pure component に抽出
- `FrontRiskConfig` を導入し、Behavior state と corridor target を持たせない

Exit verification: front target、distance/speed、risk level、SafetyBrake request の境界値と sequence が baseline exact/tolerance 内。

### Phase 2B-3: `CorridorProposalBuilder` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2B-2 DoD`。事前比較 fixture は V2X/Behavior slice と Corridor/Reference slice の proposal 部分。

- V2X projection、occupied/free interval、左右/低速候補、reachability assessment を pure extraction
- `CorridorProposalConfig` を導入
- proposal を一周期一回だけ作り、生成中に tracker/side-lock state を変更しない

Exit verification: proposal ID/identity、候補ごとの feasibility/reject reason/bounds/target と reachable metrics が baseline 内。

### Phase 2B-4: `RaceBehaviorPlanner` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2B-3 DoD`。事前比較 fixture は V2X/Behavior slice の Behavior sequence と承認済み proposal schema。

- `CorridorProposalSet` summary を入力に Cruise/Follow/Overtake/LowSpeedAvoidance/SafetyBrake selection を mechanical extraction
- `RaceBehaviorConfig` と `BehaviorPlannerState` を導入
- horizon vector、gap 再計算、QP、ROS I/O を planner から除く

Exit verification: state/category、target ID、selected proposal/pass side、speed/stop request、hold semantics が baseline と一致。

### Phase 2B-5: `OvertakeLinePlanner` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2B-4 DoD`。事前比較 fixture は V2X/Behavior slice の overtake-line sequence と Solver Execution/Fault slice の failure/fallback matrix。

- `MpcExecutionConfig`、`MpcExecutionAttempt`、`MpcExecutionAdapter`、`MpcExecutionStepResult`、shadow `SolverExecutionState` を抽出し、preflight/problem/solve/control-conversion failure、fallback、counter を shadow chain 内の一つの owner に集約する。legacy production owner はまだ変更しない
- target/side lock、ShiftOut/Pass/Return/Recovery と continuity を抽出
- `OvertakeLineConfig` と `OvertakeLinePlannerState` を導入
- `execute()` には solve 前に確定した同周期の `OvertakeLineDecision.phase` を渡し、Outcome/candidate/feedback/next state を一回で返す。周期末の `project_overtake_feedback()` 呼出しは一回に限定する
- Phase 2B-5 の shadow では shadow coordinator だけが projection を呼び、production feedback/state へ書き込まない
- `OvertakeExecutionFeedback(t-1)` のみを読み、same-cycle callback、reference rebuild、second solve を導入しない

Exit verification: B-07 の preflight/problem-build/conversion/non-finite を含む solver failure cycle は shadow fallback、次周期に Recovery/target suppression となる順序と全 phase sequence が baseline と一致。production command/state は不変。

### Phase 2B-6: Selection handoff / integration verification

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2B-5 DoD`。事前比較 fixture は V2X/Behavior slice 全体、Solver Execution/Fault slice の fault matrix、Corridor/Reference slice の selection handoff 部分。

- `CorridorProposalSet`、`BehaviorSelection`、`OvertakeLineDecision` と各 next state を別 artifact として同じ cycle/identity に結び付ける
- selected proposal ID、target、pass side、`SnapshotIdentity` を後続 corridor 適用へ明示的に渡す
- cutover candidate は node restart + clean session/reset boundary から開始し、shadow/legacy state を production state へ copy しない。同じ initial state/input prefix の replay で初期 state と遷移を照合する
- production cutover と同時に cycle-tail projection の唯一 owner を shadow coordinator から compatibility façade へ移し、shadow call site を production binary/path に残さない
- shadow chain が baseline 内であることを確認後、`MpcExecutionAdapter` と V2X/FrontRisk/proposal/Behavior/OvertakeLine decision を production caller へ切り替える。同じ subphase で legacy failure counter/fallback mutation を外し、`SolverExecutionState` を唯一の production owner にする
- corridor 反映は前記 signature の一時的な `LegacyCorridorCommitAdapter` に限定し、現行 `update_last_target=true` 相当の application pass、state owner、output を明示する。legacy evaluation/Behavior/OvertakeLine を再実行せず、新 proposal と legacy planner の二重 state mutation を許さない
- `LocalCorridorPlanner::commit` の抽出と no-recompute invariant は Phase 2C-1 に残す
- `OvertakeExecutionFeedback` が次周期だけに渡り、同周期 callback/再 solve がないことを統合 fixture で確認する

Exit verification: Phase 2B の全 state owner、transition 回数、selection handoff、B-07 failure/fallback sequence、Domain 1〜4 V2X event sequence が Full Baseline v1 内。

### Phase 2C-1: `LocalCorridorPlanner::commit` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2B DoD`。事前比較 fixture は Corridor/Reference slice の proposal/selection/commit。

- selected proposal と overtake-line decision を `CorridorPlan` に commit
- `LocalCorridorConfig` と `LocalCorridorPlannerState` を導入
- commit 内の V2X projection/free interval/proposal 再計算を禁止
- proposal/selection/decision/cycle の identity と proposal ID を exact 検証
- `LegacyCorridorCommitAdapter` を `LocalCorridorPlanner::commit` に置換し、同じ subphase 内で adapter と legacy の evaluation/反映再計算 path を削除する。runtime dual path を残さない

Exit verification: bounds/target/speed limit、side/target continuity、identity mismatch failure が green。

### Phase 2C-2: `OperationalLimitResolver / LocalReferenceBuilder` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2C-1 DoD`。事前比較 fixture は Corridor/Reference slice の reference 部分。

- Domain/Start window/ref_vel section の速度解決を `OperationalLimitResolver` に抽出
- `OperationalLimitConfig`、`LocalReferenceConfig` を導入
- base path、Behavior、CorridorPlan、OperationalLimits を現行順で `ReferenceHorizon` へ合成
- base path の周期 mutation を廃止

Exit verification: horizon の全 field が baseline 内、base path snapshot は不変、builder は V2X/FSM/solver を参照しない。

### Phase 2D: `MpcProblemBuilder` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2C DoD`。事前比較 fixture は Problem slice。

- 完成済み reference と vehicle/model/config だけから QP を生成
- `MpcProblemConfig` を導入
- V2X/FSM/lap/Domain/env/time、solver invocation、solver feedback side effect を除く

Exit verification: canonical sparse `P/A`、`q/l/u`、dimensions と solver/behavior 周期 sequence が baseline と一致。

### Phase 3: One-cycle API

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 2D DoD`。事前比較 fixture は Cycle slice。

- `update_current_speed`、`update_v_max`、path mutation、`get_control` の順序依存を `CycleInput -> CycleOutput` にまとめる
- Phase 2B-0 の `CycleSnapshotIdentityProvider` と既存 epoch owner を再利用し、pending snapshot を `SingleThreadedExecutor` の周期先頭で一度だけ交換して full `CycleInput` に統合する。identity/version の別生成経路を作らない
- 周期先頭の順序を `begin_cycle() -> validation 済み domain refs と complete config candidate の commit -> active version/source-stamp read -> seal_identity()/CycleInput` に固定し、config snapshot と identity の version mismatch を construction error にする
- path/V2X/session の domain validation、accepted-version increment、last-known-good は各 owner に残し、cycle adapter は再検証しない。invalid source だけを保持し、他 source の valid update を rollback する global transaction を作らない
- cycle-tail projection call を compatibility façade から one-cycle orchestrator の `step()` commit へ機械的に移し、同じ subphase で façade 側 call site を削除する
- `CycleAdapterConfig`、component-state aggregate、Phase 2D までに導入済みの component-local typed config を束ねる immutable `ControllerConfigSnapshot` を導入する。snapshot schema、config epoch、pending→active commit の owner は Phase 3 で確定し、component へ aggregate 全体を渡さない。Phase 4 は同じ owner/version のまま自身の typed-config field だけを schema へ追加する
- flat YAML / Domain override / dynamic parameter から typed config 値を作る既存 mapping と precedence は変更せず、その pure mapping 整理を Phase 5 に残す
- `awsim_vehicle_state`、legacy `local_controller_enabled`、`local_stop_requested` を別 field にし、二つの control-mode topic を統合しない

Exit verification: input-prefix replay の output/next substate/identity と更新境界 event 順が一致し、active snapshot と identity の全 version が exact match する。40 Hz、SingleThreadedExecutor、topic/QoS/外部 I/O は不変。

### Phase 4A: `ControlPostProcessor` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 3 DoD`。事前比較 fixture は Safety/Arbitration slice の nominal/postprocess 部分。

- acceleration conversion/filter、steering gain/filter/rate/angle の現行順序を抽出
- `ControlPostProcessorConfig` と `PostProcessorState` を導入
- `ControllerConfigSnapshot` を同じ config epoch/commit owner のまま `ControlPostProcessorConfig` field で拡張する
- fallback/Recovery/Boost/gear/arbitration を持たせない

Exit verification: raw/filtered candidate と previous filter state が baseline 内、final safety limit を先取りして挙動変更しない。

### Phase 4B: `SafetySupervisor` and policies extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 4A DoD`。事前比較 fixture は Safety/Arbitration slice の fault/Recovery/Boost/gear 部分。

- stale/non-finite/solver failure/control disabled/stop の inhibit と mandatory stop を抽出
- `SafetySupervisorConfig`、`SafeStopConfig`、`RecoveryPolicyConfig`、`BoostPolicyConfig`、`GearPolicyConfig` と各 state を導入
- `ControllerConfigSnapshot` を同じ config epoch/commit owner のまま Phase 4B の typed-config fields で拡張する
- Phase 0b で承認済みの `HardSafetyCommandValidator` を抽出し、`PrevalidatedSafeStop` を nominal pipeline 前に一度だけ生成・事前検証する
- `SafetySupervisorState` を fatal latch の唯一 owner とし、context-matched `GuaranteedTerminalStop` の identity/gear validity を保持する
- policy は候補/action を返し、publisher と source priority を持たない

Exit verification: fault fixture、Boost inhibit、Recovery/gear dormant pure-rule、one-shot SafeStop、fatal fault が green。

### Phase 4C: `CommandArbiter / FinalCommandValidator` extraction

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 4B DoD`。事前比較 fixture は Safety/Arbitration slice の arbitration/final-command 部分。

- `CommandArbiterConfig` と、required-field / one-shot substitution flow だけを持つ `FinalCommandValidatorConfig` を導入する。authoritative hard limit は `ValidatedHardSafetyLimits` から構築した共有 `HardSafetyCommandValidator` だけが所有し、config 間で複製しない
- `ControllerConfigSnapshot` を同じ config epoch/commit owner のまま Phase 4C の typed-config fields で拡張する
- eligible candidate の優先順位選択と final invariant 検証を別関数/componentにする
- invalid selected command は prebuilt SafeStop へ一度だけ置換し、再帰 arbitration/fallback を禁止
- `/control/command/control_cmd` publish は既存 adapter 一つに保つ

Exit verification:

- 全競合 test と final invariant が green
- gate scenarios/dev4 の event sequence、publisher ownership、behavior envelope が baseline/contract 内
- disabled Recovery/gear の canonical runtime は N/A、dormant pure-rule は PASS

gain 適用後 limit や R-13 の SafeStop/fatal semantics など Phase 0 で安全問題と判断されたものは、Phase 4 に紛れて直さず Phase 0b の意図的修正として先に baseline 化する。Phase 4A〜4C はその承認済み挙動の mechanical extraction に限定する。

### Phase 5: Config/tools/package boundary

Start condition: `Contract/Safety Floor + Full Baseline v1 + Phase 4C DoD`。事前比較 fixture は Config/Release slice と、変更が及んだ他 slice の和集合。

- 既に導入済みの component-local typed config 群を flat YAML/Domain override/dynamic parameter から組み立て、Phase 3 の snapshot-candidate API へ渡す pure compatibility mapping を整理する。snapshot schema/version、pending→active commit、周期境界 swap は所有しない
- key/default/Domain override/source precedence を維持し、巨大 config を pure component へ再導入しない
- pure library の CMake target、offline trajectory tool、runtime dependency を整理
- package 分割が必要なら `aichallenge_submit/` 内で行い、互換 entry を残す
- `mpc-integration.md` と必要な architecture 文書を更新

Phase 5 DoD 後に `Final Full Verification` を実行し、build/test、提出 tar、eval image、`make eval`、既存 CLI/config/launch、全外部 I/O/identity を Full Baseline v1 と同じ matrix で確認する。

legacy 削除、ROS node 分割、制御性能改善、OSQP warm start は Phase 5 完了後の別計画とする。

## 9. Test strategy

### 9.1 Pure unit tests

- value object validation と単位/サイズ invariant
- online MPC / speed-profile 両方の solver status/validation/failure semantics
- V2X freshness/ID validation
- proposal assessment -> Behavior selection -> corridor commit と identity mismatch
- commit が proposal を再計算しないこと、Behavior が horizon vectorへ依存しないこと
- component-local previous state から next state への遷移
- behavior/overtake-line state sequence と next-cycle `OvertakeExecutionFeedback`
- local reference boundaries と base snapshot immutability
- arbitration conflicts、Boost/gear/Recovery、one-shot SafeStop、fatal terminal rules

### 9.2 Characterization replay

- legacy/current implementation が吐いた normalized cycle fixture を入力
- `LegacyReplayHarness` は clean reset から input/event prefix を再生し、private state snapshot に依存しない
- 同じ fixture を現行実装で 2 回以上 replay し、正規化 output の決定性を先に確認する
- exact field と tolerant numeric field を分離
- sparse matrix は canonical `(row,column,value)` に変換し、dimensions/coordinate set/ordering/row semantics の exact と value tolerance を分離する
- insertion order、Eigen storage order、PID、絶対 timestamp を golden にしない
- proposal/selection/commit/output の cycle ID、proposal ID、`SnapshotIdentity` を exact 比較する
- test-only legacy oracle との shadow comparison は移行中だけ許可
- production binary に長期 dual path や runtime switch を残さない
- 走行 trace は wall-clock ではなく path progress/waypoint/event を基準に整列する
- 連続量は同一 image/host の反復から中央値と MAD を取り、hard safety limit と baseline envelope の両方で判定する

### 9.3 ROS integration

- node/param/topic/QoS/sole publisher
- Domain 1 と Domain 1〜4
- 40 Hz と timing distribution
- launch route と canonical config 解決
- pending snapshot が timer 周期先頭で一度だけ反映され、同一周期内の identity が変わらないこと
- Phase 前後で外部 endpoint/type/direction/QoS/owner が不変であること

### 9.4 Evaluation ladder

```text
approved change-scoped baseline slice
  -> unit + characterization
  -> make autoware-build
  -> package test
  -> affected make dev smoke / deterministic fault replay
  -> affected gate1/2/3 only
  -> make dev4 (V2X/Domain-sensitive Phase)
  -> Phase 5 DoD
  -> Final Full Verification (same Full Baseline v1 matrix + submission/eval)
```

## 10. Change and rollback policy

- 一つの Phase 2B/2C/4 subphase を含む各変更 Phase を独立 commit/PR にする。
- Phase 内でも mechanical move、API 導入、caller 切替、旧コード削除を review 可能な commit に分ける。
- baseline mismatch が出た場合、tolerance を先に広げず、最初に入力、state transition、unit/frame、呼出順を調べる。
- Phase gate を通るまで旧 production path を削除しない。切替後は旧 production path を同じ Phase 内で削除する。
- rollback はその Phase の commit revert で前の green baseline に戻せる粒度を保つ。
