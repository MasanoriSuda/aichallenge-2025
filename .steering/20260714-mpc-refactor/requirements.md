# MPC 疎結合化 要求仕様

- 作成日: 2026-07-14
- 最終更新: 2026-07-15（補正レビュー反映）
- 状態: Draft
- 対象: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`
- 入力資料: `point-out-by-chatgpt-pro.md`、`point-out-v1.md`、2026-07-15 補正レビュー、現行コード、現行 launch/config、`docs/spec/`、`docs/interface/`

## 1. 目的

現行 MPC の外部契約と観測可能な挙動を先に固定し、その基準と比較しながら責務を段階的に分離する。

この作業で優先する順序は次のとおりとする。

1. 安全性と評価インターフェース互換性
2. 現行挙動の再現性
3. 変更差分の説明可能性
4. 内部構造の疎結合化
5. 将来の controller、経路生成、V2X 戦術の差し替えやすさ

「凍結」は、現行コードを無条件に正解として永久保存する意味ではない。まず実挙動を `Baseline candidate v0` として記録し、既知の不整合や安全上の疑義を分類する。必要な修正を独立差分として行った後に、リファクタ全体の比較対象となる `Full Baseline v1` を確定する。`Baseline v1` という名称は Full 承認済み基準だけに使い、途中の限定 fixture を `Baseline v1` と呼ばない。

## 2. 正本と基準の扱い

### 2.1 外部契約の正本

- `docs/interface/participant-interface.md`
- `docs/interface/evaluation-interface.md`
- リポジトリ直下の `AGENTS.md` に列挙された現行互換契約

内部リファクタを理由に、これらの契約を変更しない。契約変更が必要になった場合は本計画から切り離し、先に `docs/interface/`、影響範囲、移行方針を更新する。

### 2.2 挙動基準の正本

`Baseline candidate v0` では、記録した commit、launch 経路、config、resource、実行環境から得られた実挙動を観測対象とする。文書と実装が食い違う場合はどちらかを暗黙に採用せず、`baseline.md` の差分台帳で判断する。

### 2.3 2026 仕様の扱い

公式確認できていない事項は、2025 由来の現行仕様または暫定仕様として扱う。ローカルの `make gate1` / `gate2` / `gate3` と 2026 公式評価項目が同一であるとは仮定しない。

### 2.4 段階ゲートと baseline 名称

検証の幅は変更対象に応じて段階化するが、外部契約と hard-safety の合格条件は段階化によって弱めない。各段階は次の意味を持つ。

これは旧レビューの「Minimum / Full」二分を、その後に承認された変更対象別 baseline へ置き換える決定である。ただし Minimum を軽くする意味ではなく、H-01〜H-08 と external contract は `Contract/Safety Floor` として常時必須にする。現時点の `baseline.md` は H-01〜H-08 が `UNRESOLVED` のため、Phase 1 をまだ開始できない状態を意図的に表す。

1. **Contract/Safety Floor**
   - Phase 1〜5 の production path を変更する全構造リファクタの共通前提。
   - `docs/interface/` と `AGENTS.md` の外部契約を static exact oracle とし、対象 runtime の endpoint/type/direction/Domain/owner、current QoS compatibility、単一 publisher を確認する。
   - 正方向 endpoint だけでなく、`autostart_orchestrator_node` が `/admin/awsim/start` に触れないこと、`awsim_state_manager_node` が `/awsim/state` を消費しないこと、`admin_start_once: true`、状態文字列、result schema 主要キー、`output/latest/` / UID ownership の負方向・成果物 invariant も exact 確認する。
   - Boost は車両 Domain の `/awsim/status` / `/awsim/cmd` と `Float32MultiArray` だけを使い、`/awsim/boost_cmd`、`Bool`、Domain 0 の `/admin/awsim/*` で代用しない。gear/Recovery は `/admin/awsim/reset`、クロスドメイン転送、teleport/respawn で代用しない。提出 tar はリポジトリ直下の Docker build context 内の相対 path に置く。
   - H-01〜H-08 の signal、単位、authoritative criterion、観測方法を確定し、`FAIL` / `UNRESOLVED`、external-contract RED、hard-safety RED を残さない。
   - scoped baseline はこの Floor を免除、延期、waiver しない。変更範囲が ROS adapter、launch/config、最終 arbitration、評価基盤へ広がった場合は、該当する Full 検証へ即時昇格する。
   - 唯一の前段例外は Phase 0b の ordered remediation とする。まず **Contract Conformance lane** で、static oracle が検出した external-contract RED を既存 `docs/interface/` へ実装適合させる修正だけを、exact target・test・rollback 付きで許可する。契約文書、endpoint/type/Domain/schema 自体は変えない。external-contract RED が 0 件になった後だけ **Safety lane** へ進み、H-01〜H-08 の判定基準・観測方法を確定して登録済み hard-safety RED を是正する。各 lane は独立 commit/review とし、修正後に Floor 全項目を green にできなければ scoped baseline と Phase 1 へ進まない。
2. **Scoped Solver Baseline**
   - Phase 1 の `OsqpBackend / QpSolver` mechanical extraction だけに使用する限定 oracle。
   - QP、solver、同周期 fallback、raw/final command と代表 Cruise/fault fixture を固定する。
   - Full baseline ではなく、Phase 2B 以降の behavior/reference/arbitration 等価性の根拠には使わない。
3. **Scoped Path Baseline**
   - Phase 2A の `BasePathStore` read-only 化だけに使用する限定 oracle。
   - base path、Scoped Solver で承認済みの resolved speed artifact との結合、trajectory update、周期 dynamic view の値と last-known-good path 保持を固定する。optimizer/QP 自体は再定義しない。
   - Solver fixture の代用や Full baseline への自動昇格を認めない。
4. **Full Baseline v1**
   - この名称を使用できる唯一の baseline。
   - Contract/Safety Floor、全 deterministic fixture、V2X/behavior/reference/arbitration、Domain 1〜4、gate、eval、timing、artifact identity を含み、Phase 2B-0 以降の開始条件とする。
5. **Final Full Verification**
   - 最終 candidate に対し Full Baseline v1 と同じ契約・安全・回帰 matrix、提出 tar、eval image、`make eval` を再実行する完了判定。
   - 新しい baseline version ではなく、Full Baseline v1 に対する最終検証結果として保存する。

標準順序は `Contract/Safety Floor -> Scoped Solver Baseline -> Phase 1 -> Scoped Path Baseline -> Phase 2A -> Full Baseline v1 -> Phase 2B-0 -> Phase 2B-1 以降 -> Final Full Verification` とする。Phase 0b の意図的修正が入った場合は、その修正後の clean commit と binary/image/tool identity を新しい anchor とし、影響する scoped/full artifact を再取得する。

## 3. スコープ

### 3.1 対象

- 現行挙動、設定、実行環境、ROS graph の baseline 化
- QP solver wrapper の分離
- base path と 1 周期の reference horizon の分離
- V2X/追従/追い越し判断と QP 構築の分離
- 1 制御周期を表す明示的な input/output API の導入
- 制御後処理、Recovery、Boost、gear、fail-safe の最終調停責務の集約
- pure C++ component の単体・characterization test の追加
- runtime と offline tool の依存整理

### 3.2 非目標

少なくとも Phase 0〜4 では、次を行わない。

- ROS node の物理分割、新規内部 topic/message の導入
- 制御性能改善、controller tuning、V2X 戦術変更
- dormant な Recovery/path constraint 機能の有効化
- config 既定値、CSV schema、経路補間・平滑化・幅計算の変更
- control rate、executor、QoS、node/executable 名の変更
- map/trajectory の再生成
- legacy Boost の削除
- `aichallenge_system/`、評価 FSM、result JSON、`output/latest/` の変更
- 実車向け速度・操舵・制動設定の変更
- 未確定事項の 2026 公式契約への昇格

性能改善、安全修正、legacy 削除、ROS node 分割は、必要性と検証方法を別途定義した独立タスクとする。

## 4. 固定する外部契約

全 Phase で以下を維持する。

1. Domain 0 は AWSIM と `awsim_state_manager_node`、車両は Domain 1..N とする。
2. クロスドメイン通信は `/v2x/vehicle_positions` と `v2x_msgs` を使用し、`domain_bridge` を追加しない。
3. `/admin/awsim/start`、`/admin/awsim/reset`、`/admin/awsim/state` の名前・型・責務を変えない。
4. `/awsim/state` と `/awsim/control_mode_request_topic` の名前・型・状態文字列を変えない。
5. `aichallenge_submit.launch.xml`、既定 `control_method=mpc`、契約済み 5 値（`mpc`、`pure_pursuit`、`tiny_lidar_net`、`pilot_net`、`joycon`）の launch 分岐を維持する。
6. `/localization/kinematic_state` と `/planning/scenario_planning/trajectory` を含む participant stack の安定データフロー契約を維持する。
7. `/control/command/control_cmd` を最終制御出力とし、publisher owner を一つに保つ。
8. `/set_initial_pose` service を評価起動ハンドシェイクとして維持する。
9. 現行 participant contract にある `/awsim/status`、`/awsim/state`、`/awsim/cmd`、gear topic の名前・型・方向を変えない。
10. 提出 tar、result JSON、`output/latest/`、`HOST_UID` / `HOST_GID` の契約を変えない。
11. Boost に `/awsim/boost_cmd`、`Bool`、Domain 0 の `/admin/awsim/*` を代用しない。
12. gear/Recovery に `/admin/awsim/reset`、クロスドメイン転送、teleport/respawn を代用しない。
13. 提出 tar はリポジトリ直下の Docker build context 内に置き、build context 外の path を渡さない。

## 5. 機能要求

### R-01: Baseline manifest

commit、dirty state、Autoware/AWSIM launch route、launch 引数、設定/resource hash、Domain 別解決値、Docker image、実行 binary、ROS graph、topic type/QoS/publisher 数を記録できること。Domain 別 effective config は別 parser で再実装せず、production と同じ C++ resolver から出力する。

現行 interface compatibility の対象は Domain 1〜4 とする。4 台時の競技挙動が 2026 公式評価対象かどうかは未確定でも、Domain 分離、fallback 設定、topic/type/owner は確認する。

各 artifact は anchor commit、source/config/resource hash、binary/image/tool identity と `SnapshotIdentity` を持つ。Full evidence の取得を後段に分ける場合も、変更後実装だけを自身の baseline にせず、Phase 0b 後の凍結済み pre-refactor anchor から再取得または比較できること。

### R-02: 再現可能な入力

単体 fixture または記録済み周期入力から、少なくとも次を再生できること。

- 単車 Cruise
- V2X Follow / Overtake
- stale/no-gap/欠落 ID を含む V2X 境界条件
- solver failure と連続 failure
- stale odometry、非有限値、stop request
- Boost start-once と再 arm
- Recovery disabled 時の非介入

Recovery enabled や path constraint など canonical runtime で無効な機能は、実走等価性を主張せず、pure unit test で現在の規則を固定する。

固定 fixture は test-only `LegacyReplayHarness` で再生する。private state の完全 snapshot は保存せず、constructor/reset の clean state から explicit ROS/steady timestamp を持つ input/event prefix を順番に適用して FSM、filter、V2X history を再構成する。同じ fixture を現行実装で 2 回以上 replay し、正規化 output が同じになることを Scoped Solver Baseline、Scoped Path Baseline、Full Baseline v1 の条件とする。

`OnlineMpc` と `ReferenceSpeedProfile` は別 fixture family・別 artifact とし、artifact schema に `ProblemKind::OnlineMpc` / `ProblemKind::ReferenceSpeedProfile` の discriminant を必須にする。`OnlineMpc` は周期 input、reference horizon、QP、solver、command を記録し、`ReferenceSpeedProfile` は base path、Domain/start window/ref-velocity section と解決済み速度列を記録する。一方を他方の golden や identity の代用にしない。generic `OsqpBackend` 自体は problem kind で分岐しない。

Scoped Path Baseline では trajectory update の成功と失敗を別 case にする。valid update は新しい path snapshot/hash/version の採用を、invalid/incomplete update は拒否理由と直前の valid path snapshot/hash/version の保持を固定する。失敗時に部分更新、empty path への置換、暗黙の初期 CSV 再読込を行わない。

### R-03: 比較可能な出力

少なくとも次を比較対象とする。

- behavior state、reason、対象車両、pass side、phase
- reference horizon の座標、姿勢、曲率、速度、左右境界、横位置目標
- QP の次元、疎行列構造、`P/A/q/l/u`
- solver status、解、最大制約違反、failure count
- raw command、後処理後 command、最終 command
- fail-safe、Recovery、Boost、gear の決定と理由
- prediction と主要 diagnostics

solver の full solution vector は補助比較として保存する。primary comparison は accepted status、objective、最大制約違反、first control、predicted trajectory とし、非一意解で full vector の要素一致を合格条件にしない。

### R-04: 意図的差分の管理

baseline と異なる変更は、次の情報が揃わない限り受け入れない。

- 変更前後の差分
- 変更理由と安全・評価影響
- 対応する test
- reviewer の判断
- baseline version の更新要否

既知問題の修正と構造リファクタを同じ commit/Phase に混在させない。Phase 0b で許可するのは、実装を既存 interface contract に適合させる修正と、外部契約を変えない安全修正だけとする。topic/type/Domain/control method/result schema 自体の変更は本計画外とする。

### R-05: 明示的な依存

core component は、ROS message、publisher、logger、環境変数、暗黙の `now()`、他 component の mutable public member を直接参照しない。時刻、Domain/vehicle 情報、V2X snapshot、previous-cycle feedback、config は input として渡す。

現行は OSQP workspace / primal-dual warm start を保持していない。previous control sequence、previous steering、last solved waypoint、prediction linearization history は `MpcControlHistoryState` とし、failure counter、overtake failure counter、fallback speed、last category を持つ `SolverExecutionState` と混在させない。`MpcWarmStartState` という名称や将来の solver workspace 再利用を先取りしない。

### R-06: 互換 façade

既存の `mpc_controller_cpp` executable と node、launch、flat YAML key を当面の互換 façade として維持し、内部の型付き component に変換する。

### R-07: 一つの最終出力経路

MPC、Recovery、SafeStop が別々に `/control/command/control_cmd` を publish してはならない。最終調停、有限値確認、limit 確認を通過した command だけを既存 ROS adapter が publish する。

### R-08: 1 周期 API

`update_*()` の呼出順で結果が変わる API を、明示的な `CycleInput -> CycleOutput` に置き換える。同じ input と同じ初期 state に対して同じ output を返すこと。

### R-09: 段階ゲート

各 Phase は独立して build/test/characterization comparison を通過し、前 Phase の DoD と対象 baseline の開始条件が満たされるまで次へ進まない。Phase 1 は Scoped Solver Baseline、Phase 2A は Phase 1 DoD と Scoped Path Baseline、Phase 2B-0 以降は Full Baseline v1 を開始条件とする。Contract/Safety Floor は Phase 1〜5 に継続適用し、Phase 0b は 2.4 の限定 remediation entry に従う。各 Phase は原則として独立 commit または PR とする。

### R-10: 文書同期

実装・運用が変わった Phase で `docs/spec/mpc-integration.md` を更新する。node/package tree が変わる場合は `docs/spec/architecture.md` も更新する。外部契約は原則変更しない。

### R-11: Corridor の proposal / selection / commit

corridor 計算は少なくとも proposal、selection、commit の 3 観測点に分ける。同じ `SnapshotIdentity`、同じ component state、同じ config に対して、候補集合と reject reason、選択 side/target、committed corridor が決定的であること。

proposal set は少なくとも `Base`、`FollowPreposition`、`LowSpeedAvoidance`、`OvertakeLeft/Right`、`OvertakeFallbackLeft/Right` を表現し、各候補に proposal ID、feasibility/reject reason、side、bounds、target、speed limit を持たせる。Cruise/SafetyBrake など corridor 変更が不要な state も明示 `Base` proposal を選択し、optional ID や magic sentinel で commit validation を迂回しない。Overtake line の phase 別 horizon target は selected proposal の bounds/metadata から作り、V2X/gap/free interval を再計算しない。

commit は 1 周期につき高々 1 回とし、committed corridor はその周期中 immutable とする。`LocalReferenceBuilder`、QP builder、solver failure path は同じ committed artifact を参照し、候補生成、side 選択、corridor commit を再計算しない。再計算が必要な input 変更は、新しい `SnapshotIdentity` を持つ次周期として扱う。

`SnapshotIdentity` は少なくとも path/config/V2X/session の version、cycle identity、ROS/steady cycle time、比較に必要な source timestamp を区別できること。proposal はさらに base waypoint ID、horizon size、生成時刻を持ち、selection、OvertakeLine decision、commit、reference、QP、command の artifact に引き継がれること。

この identity を Phase 3 まで先送りしない。Phase 2B の最初に `CycleSnapshotIdentityProvider` を抽出し、周期先頭で cycle ID / ROS・steady time と、既存 owner が供給する path/config/V2X/session epoch を一度だけ束ねる。Phase 2B/2C component はこの値を受け取る。

Phase 3 は provider を二段階 API として再利用し、`cycle ID/time 採取 -> accepted pending refs と complete config candidate の commit -> active snapshot の version/source stamp 読取 -> SnapshotIdentity と CycleInput の seal` の順序を唯一の cycle adapter に固定する。Path/V2X/session は各 owner が domain validation、accepted-version increment、last-known-good を担当し、cycle adapter は validation 済み immutable candidate/ref の選択・集約だけを行う。config だけは cycle adapter が complete `ControllerConfigSnapshot` candidate の schema/coherence を検証し、単一 version として commit する。invalid complete config は last-known-good config/version を維持し、valid domain refs の commit を妨げない。一つの invalid source が他 source の valid update まで rollback する全体 transaction は導入しない。

Phase 3 以降、ROS callback/façade は raw input を対応する domain owner、または config candidate を cycle adapter へ渡すだけで、active aggregate、config epoch increment、周期境界 swap を持たない。`SnapshotIdentity.config_version == active ControllerConfigSnapshot.version` と全 active snapshot version の一致を construction 時と更新境界 fixture で exact 検証する。

### R-12: Solver failure の同周期処理と次周期 feedback

solver failure の**同周期処理**は fallback または SafeStop candidate の選択であり、同周期の behavior/corridor への暗黙再入、corridor 再 commit、無制限な second solve を行わない。

preflight failure、problem-build failure、または `MpcSolveResult` のいずれかを同じ identity に結び付けた discriminated `MpcExecutionAttempt` とし、`MpcExecutionAdapter::execute()` へ一度だけ渡す。adapter 内で solver setup/status、solver-to-control conversion とその MPC-internal non-finite まで分類する。戻り値 `MpcExecutionStepResult` は、正規化された事実だけを持つ `MpcExecutionOutcome`、success 時の raw nominal control または failure 時の同周期 fallback candidate、`SolverExecutionFeedback`、next `SolverExecutionState` を別 field で返す。`MpcExecutionOutcome` 自身に candidate や mutable state を持たせない。

`MpcExecutionAdapter` を failure counter、overtake failure counter、fallback speed、last category の唯一 owner とする。failure の分類には previous state ではなく、solve より前に確定した同周期の `OvertakeLineDecision.phase` を明示 input とし、ShiftOut 開始周期を一周期ずらさない。

Overtake の実行可否や failure count へ反映する**次周期処理**は、周期末の唯一の commit site が `MpcExecutionAdapter::project_overtake_feedback()` をちょうど一度呼び、`SolverExecutionFeedback` から solver 固有情報を除いた `OvertakeExecutionFeedback` へ変換する。同周期 fallback とは別の値・state transition・artifact とし、次周期の `OvertakeLinePlanner` または corridor continuity owner だけが消費する。API を抽出する Phase 2B-5 の shadow 中は shadow coordinator、Phase 2B-6 の production cutover 後は compatibility façade、Phase 3 以降は one-cycle orchestrator が commit site を所有し、移管と同じ subphase で旧 call site を削除する。`RaceBehaviorPlanner` へ raw `MpcSolveResult` / `MpcExecutionOutcome` / `SolverExecutionFeedback` を直接渡さない。Behavior 側に通知が必要になった場合も、solver 固有値を含まない狭い戦術上の summary を別契約として定義する。fixture は同周期 command source と次周期の OvertakeLine/corridor transition を別 field で比較する。

### R-13: FinalCommandValidator の one-shot fail-closed

起動時に forward/reverse/unknown gear context を含む `ValidatedHardSafetyLimits` を独立検証し、その値だけから SafeStop template/factory と pure `HardSafetyCommandValidator` を構築する。各周期では nominal pipeline より前に、検証済み limit、現在の validated gear context、直前に実際に publish された gain 適用後 final steering だけから `PrevalidatedSafeStop` を用意し、同じ `HardSafetyCommandValidator` で事前検証する。通常 config や reject 済み candidate を生成元にしない。

最終 candidate は `FinalCommandValidator` が一度検証する。candidate が non-finite、hard limit/rate/gear 条件違反、または不完全な場合、planner/solver/arbitration へ再入せず、その周期の同じ `PrevalidatedSafeStop` へ一度だけ置換して再検証する。

SafeStop が検証に成功した場合はその周期の唯一の final command とし、Boost/Recovery/通常 gear action を抑止する。SafeStop 自体が検証不能または不正な場合、`FinalCommandValidator` は再帰的 fallback を行わず `FatalSafetyFault` event と reason `FatalSafeStopValidationFailure` を返し、`SafetySupervisor` が fatal latch の唯一 owner として周期末に commit する。

fatal 時に publish 候補にできる `GuaranteedTerminalStop` は、通常の `PrevalidatedSafeStop` や「過去に valid だった stop」の単なる再利用ではない。直前の final command identity と validated gear-context version に対する有効性を別 safety contract で証明し、その identity が current context と exact match する場合だけ既存 ROS adapter から使用できる。保証済み terminal stop が存在しない、または context が一致しない場合は command を合成・再利用せず fatal error として停止する。以後は通常 command、Boost、Recovery、gear action を出さない。

現行 `Baseline candidate v0` はこの semantics を満たさないため、Phase 4 の構造抽出に挙動変更を混ぜない。差分を hard-safety oracle で確定し、Phase 0b の「外部契約を変えない安全修正」として独立実装・検証した後、その commit を Full Baseline v1 の anchor にする。Phase 4 は承認済み semantics の mechanical extraction に限定する。

## 6. 非機能要求

### 6.1 決定性と許容差

- enum、flag、配列サイズ、publisher 数などは原則 exact match とする。疎行列は dimensions、triangle policy、variable/constraint ordering、重複座標を production 規則で集約した後の正規化 `(row, col)` 座標集合を exact とし、triplet 挿入順や Eigen/OSQP の内部格納順を exact 条件にしない。
- contract oracle、deterministic cycle fixture、live-run envelope を別 schema/判定にする。
- 浮動小数値は baseline の同一条件を最低 3 回測定し、工学的下限、solver epsilon、quantization floor、観測 jitter を確認してから絶対/相対 tolerance を決める。
- solver full solution vector は補助比較とし、primary comparison の accepted status、objective、最大制約違反、first control、predicted trajectory を満たしているかを先に判定する。
- timestamp、ログ順序、実行時間をそのまま golden にしない。比較前に正規化する。
- 理由のない緩い tolerance や、差分を隠すための丸めを禁止する。

### 6.2 制御周期

40 Hz の設定を維持する。実行時間 budget は未確認の公式値を作らず、Full Baseline v1 の測定分布から定める。平均値だけでなく p95/p99、deadline miss、solver iteration/status を記録する。

### 6.3 安全性

- stale/incomplete/non-finite input から通常 command を生成しない。
- forced stop、Recovery、fault 中の Boost 抑止を確認する。
- 最終 command で steering angle/rate、acceleration、gear 条件が成立することを確認する。
- 実車検証は本計画の対象外とし、シミュレータと評価検証の完了前に進めない。
- external-contract RED と hard-safety RED は baseline waiver の対象にせず、解消するまで Contract/Safety Floor、Scoped Solver Baseline、Scoped Path Baseline、Full Baseline v1、次 Phase のいずれも承認しない。
- hard-safety oracle は signal、単位、閾値、根拠、観測方法、`PASS / FAIL / UNRESOLVED` を持つ。`UNRESOLVED` は Contract/Safety Floor と Full Baseline v1 を block する。
- scoped baseline でも Contract/Safety Floor は必須であり、H-01〜H-08 の `FAIL / UNRESOLVED` を `PENDING`、`KNOWN_RED`、対象外へ読み替えて通過させない。Full は scenario coverage を拡張する段階であって、安全判定を後から有効にする段階ではない。
- `FinalCommandValidator` の one-shot `PrevalidatedSafeStop` と `FatalSafetyFault` / `FatalSafeStopValidationFailure` の両分岐を deterministic fixture で検証する。

### 6.4 成果物

rosbag/MCAP、build/install/log、`output/` はコミットしない。コミット対象は小さな manifest、比較器、fixture、test、文書に限定する。

## 7. 全体 Definition of Done

- Contract/Safety Floor、Scoped Solver Baseline、Scoped Path Baseline が用途どおり承認され、Full Baseline v1 の静的 manifest、runtime evidence、fixture、tolerance がレビュー済みである。
- observation seam を含む最終 clean commit と binary/image/tar identity から baseline が再取得されている。
- 各 component の責務と input/output が `design.md` と一致する。
- 外部契約、launch entry、config key、単一 publisher ownership が維持されている。
- 全 Phase の unit/characterization/integration gate が成功している。
- `make autoware-build`、対象 package test、必要な gate、多車両確認が成功している。
- Final Full Verification で Full Baseline v1 の全 matrix、提出 tar の構造、eval image build、`make eval` を再確認している。
- 実行できなかった検証と残存リスクが明記されている。
