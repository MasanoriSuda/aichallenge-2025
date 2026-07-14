# MPC 疎結合化 Task List

- 作成日: 2026-07-14
- 最終更新: 2026-07-15（補正レビュー反映）
- 状態: Planning / Phase 0 未完了
- 原則: `Contract/Safety Floor`、対象 staged baseline、直前 Phase の Definition of Done を満たすまで次へ進まない

## 0. Planning

- [x] `git status --short` で既存変更を確認した
- [x] `AGENTS.md` と正本 interface 文書を確認した
- [x] `docs/spec/architecture.md`、`mpc-integration.md`、`safety-gates.md` を確認した
- [x] canonical launch route と config 解決経路を確認した
- [x] `mpc_controller_cpp.cpp` の主要 coupling point を確認した
- [x] core/config/launch/resource の主要 hash を取得した
- [x] `requirements.md` を作成した
- [x] `baseline.md` を作成した
- [x] `design.md` を作成した
- [x] Phase ごとの task/DoD を本書に作成した
- [ ] 計画レビューで scope、順序、Full Baseline v1 の承認者を確定する

## 全 Phase 共通条件

### 常時 `Contract/Safety Floor`

Phase 1〜5 の各 Phase と各 subphase は、変更対象の大小にかかわらず次を満たす。

- [ ] `baseline.md` の contract oracle と endpoint/type/direction/Domain/owner が exact match する
- [ ] current QoS compatibility oracle と active endpoint 数に未説明の差がない
- [ ] `/control/command/control_cmd` の publisher owner が一つである
- [ ] external-contract RED と hard-safety RED が 0 件である
- [ ] H-01〜H-08 に `FAIL / UNRESOLVED` が 0 件であり、未確定値を PASS 扱いしていない
- [ ] config/feature の enabled/disabled、40 Hz、`SingleThreadedExecutor` が意図せず変わっていない
- [ ] ROS node/executable、launch、topic/service/type、Domain、提出/result/output 契約を変更していない
- [ ] 変更 component と ROS adapter の間に新しい publisher、subscriber、暗黙 `now()`、env 参照を追加していない
- [ ] 外部契約差分が出た場合は実装を停止し、本計画外の interface migration として扱う

### 変更対象別 verification slice

各 Phase/subphase は変更 surface を先に宣言し、その surface に対応する fixture と live scenario だけを必須再実行する。未変更 surface の既存 green evidence は同じ baseline identity のまま再利用できるが、依存境界を越えた差分が出た場合は verification slice の範囲を拡大する。

- [ ] 変更対象の component、config、state、fixture、live scenario を Phase 開始時に列挙する
- [ ] 変更対象の deterministic fixture と fault fixture が green である
- [ ] 変更対象に対応する B-02〜B-09 の scenario が Phase 指定どおり green である
- [ ] `make autoware-build` と対象 package test が成功する
- [ ] 各 component は抽出した Phase から component-local typed config だけを受け、aggregate `Config` / `MpcConfig`、YAML node、env を直接参照しない
- [ ] `docs/spec/mpc-integration.md` の該当責務・構造・検証結果を同じ Phase で更新する
- [ ] node/package tree を変えない Phase では `docs/spec/architecture.md` を変更しない

### Staged baseline の順序と同一性

1. Phase 1〜5 の全構造リファクタで **Contract/Safety Floor** を維持する。Phase 0b は限定 remediation entry を通す。
2. Phase 1 前に **Scoped Solver Baseline** を承認する。
3. Phase 2A 前に Phase 1 DoD と **Scoped Path Baseline** を承認する。
4. Phase 2B-0 前に **Full Baseline v1** を承認する。
5. Phase 5 DoD 後に **Final Full Verification** を実行する。

- [ ] 全 staged baseline は同じ baseline source commit/image/config/resource/schema identity を参照する
- [ ] Phase 1/2A の candidate evidence で baseline reference fixture/hashを上書きせず、candidate側成果物として別保存する
- [ ] Phase 0b の intentional delta がある場合だけ承認済み identity/version へ更新し、全 staged baseline の参照先を同時に更新する

## 1. Phase 0 — 現行挙動の凍結

### 1.1 Static manifest

- [x] branch/commit/dirty state を記録する
- [x] canonical Autoware/AWSIM の dev/gate/eval launch route を記録する
- [x] launch/config/map/trajectory/AWSIM の主要 hash を記録する
- [x] Domain 1〜4 の静的 config 差と fallback source を記録する
- [ ] Docker image ID/digest を記録する
- [ ] compiler、OSQP、ROS distro、build option を記録する
- [ ] install 済み `mpc_controller_cpp` binary hash を記録する
- [ ] 実 run と同じ env/project の Compose config と、起動後 container inspect を run ごとに記録する
- [ ] unset `VEHICLE_ID`、unused launch `vehicle_id=default`、V2X message ID を別概念として記録する
- [ ] Phase 0 完了直前に最終 clean commit と全 source/config/resource/tool/schema hash を再取得する

### 1.2 Contract/runtime surface

- [ ] `aichallenge_submit.launch.xml`、既定 `mpc`、契約済み 5 control method の include 経路を静的確認する
- [ ] 同じ C++ config resolver から Domain 1〜4 の `effective-config.json` を保存する
- [ ] Domain 1〜4 の宣言済み dynamic ROS parameter dump を別 layer として保存する
- [ ] Domain 0 と Domain 1〜4 の publisher/subscriber、topic type、QoS、owner を oracle と照合する
- [ ] `/control/command/control_cmd` の publisher owner が一つであることを確認する
- [ ] `/set_initial_pose` と AWSIM admin/state contract に影響がないことを確認する
- [ ] `/admin/awsim/start` の bidirectional direction、`admin_start_once: true`、trigger `waitstart,ready`、`/admin/awsim/state` と `/awsim/state` の状態文字列を exact 比較する
- [ ] `autostart_orchestrator_node` が `/admin/awsim/start` を pub/sub せず、`awsim_state_manager_node` が `/awsim/state` を subscribe しない負方向 ownership を graph/static oracle で確認する
- [ ] Boost が `/awsim/boost_cmd` / `Bool` / Domain 0 admin topic を、gear/Recovery が `/admin/awsim/reset` / cross-domain / teleport / respawn を代用していないことを確認する
- [ ] submit tar の指定 path がリポジトリ直下の Docker build context 内にあることを確認する
- [ ] result summary v2 / details v3 の主要キー・型、`output/latest/` の実ディレクトリと内部リンク、`HOST_UID/HOST_GID` ownership を exact 比較する
- [ ] debug/optional endpoint の active/inactive 状態を保存する
- [ ] control/odometry/trajectory の実測 topic hz を保存する

### 1.3 Known-difference triage

- [ ] D-01 `rl_train` を非契約開発 option のまま隔離するか削除するか決める
- [ ] D-02 V2X enabled と文書の不一致を解消する
- [ ] D-03 Recovery disabled と文書の記述範囲を解消する
- [ ] D-04 publisher 不在の legacy local-control topic を維持/削除するか決め、公式 topic と統合しない
- [ ] D-05 `use_sim_time` と simulation safety gate の扱いを決める
- [ ] D-06 canonical/legacy launch の適用範囲を明記する
- [ ] D-07 flat config の互換維持範囲を確定する
- [ ] D-08 empty V2X ID の挙動を再現し、維持/修正を決める
- [ ] D-09 gain 適用後の steering angle/rate を計測し、安全 invariant を判定する
- [ ] D-10 base path 速度 profile の周期上書きを fixture で確認する
- [ ] D-11 solver failure -> behavior/recovery feedback の周期順序を確認する
- [ ] D-12 fault 時生成の現行 `publish_failsafe_command()` と、startup 検証済み limit / one-shot final validation / fatal reason を要求する R-13 の差分を確認し、Phase 0b 安全修正の scope を確定する
- [ ] D-02/D-03/D-04 の判断を `docs/spec/mpc-integration.md` に反映し、外部契約は変更しない

### 1.4 Hard-safety oracle

- [ ] H-01〜H-08 の signal、単位、閾値、根拠、観測方法を走行取得前に確定する
- [ ] final steering angle/slew の authoritative limit を確認し、raw config 値で代用しない
- [ ] stale response の開始点、safe source、許容 control interval を定義する
- [ ] final validation reject 時の one-shot `PrevalidatedSafeStop` と、SafeStop 自体が不正な場合の `FatalSafetyFault` semantics を確定する
- [ ] `FatalSafetyFault` 時の reason `FatalSafeStopValidationFailure`、terminal command、Boost/Recovery/gear 抑止、latch、ログ、終了条件を非再帰の safety oracle として定義する
- [ ] `SafetySupervisor` を fatal latch の唯一 owner とし、validator は event を返すだけであることを oracle にする
- [ ] `GuaranteedTerminalStop` は直前 final command identity と validated gear-context version の exact match を必須とし、過去の SafeStop を無条件再利用しない
- [ ] official result の collision/penalty と local proxy を別判定にする
- [ ] `PASS / FAIL / UNRESOLVED` を実装し、UNRESOLVED を blocker にする

### 1.5 Characterization harness

- [ ] Phase 0a: instrumentation 前に existing topic/log だけの black-box evidence を取得する
- [ ] Phase 0 専用の限定 topic recorder と起動/停止手順を作る
- [ ] recorder-ready barrier、topic 別 QoS override、必須 message completeness check を実装する
- [ ] MCAP/raw log から小型 manifest/events/metrics/trace を生成する
- [ ] observation step: 制御判断を変えない test-only/no-op trace sink と 1 周期 seam を設計する
- [ ] clean constructor/reset から timestamp 付き input/event prefix を再生する `LegacyReplayHarness` を実装する
- [ ] private FSM/filter/V2X state の snapshot を fixture に保存しない
- [ ] 同じ C++ resolver を使う `effective-config.json` trace を実装する
- [ ] original v0 と seam compiled/disabled の contract exact・live envelope 非悪化を確認する
- [ ] seam disabled/enabled で deterministic decision/output が一致することを確認する
- [ ] seam enabled の timing overhead は記録のみとし、production performance gate に使わない
- [ ] timestamp、log order、DDS discovery order の正規化規則を実装する
- [ ] path progress/waypoint/event による走行 trace の整列を実装する
- [ ] contract / deterministic cycle / live-run の 3 schema を分ける
- [ ] base path/reference horizon fixture を作る
- [ ] sparse matrix を shape、三角格納方針、変数順、constraint row block、重複集約後の `(row,col)` 集合へ canonicalize する比較器を作る
- [ ] triplet挿入順、Eigen/CSC内部格納順を比較せず、canonical index pattern は exact、値は field 別 tolerance で比較する
- [ ] Online MPC 用 `P/A/q/l/u`、solver result/failure/fallback sequence fixture を作る
- [ ] Reference Speed Profile 用 `P/A/q/l/u`、生成 `v_ref`、solver failure fixture を別 schema で作る
- [ ] solver は accepted status を exact、objective/max violation/first control/prediction を主比較し、full solution は補助 tolerance 比較にする
- [ ] behavior sequence fixture を作る
- [ ] raw/postprocessed/final command fixture を作る
- [ ] arbitration fixture を canonical runtime、deterministic synthetic、dormant pure-rule に分ける
- [ ] 現行 fail-safe、validation reject、shutdown の command/action/latch/control-history sequence fixture を作る
- [ ] Recovery/gear disabled の canonical runtime は N/A とし、pure-rule fixture で規則を固定する
- [ ] 同一 image/host で最低 3 回反復し、engineering/solver/quantization floor と反復差から tolerance を決める
- [ ] baseline envelope と hard safety limit を別判定として実装する
- [ ] raw command と final command の期待値/invariant を別々に定義する
- [ ] comparison report が最初の差分位置と reason を表示できるようにする
- [ ] observation seam を含む確定 commit から black-box/cycle fixture を再取得する
- [ ] 同じ fixture を legacy harness で 2 回以上 replay し、正規化 output の決定性を確認する

### 1.6 Scenario capture

- [ ] B-01 対象 package の既存 unit test を実行する
- [ ] 全 live run に clean-down、ready、開始/終了 anchor、最大 timeout、flush、completeness、cleanup を定義する
- [ ] initial-pose、node、fresh odom、reference、sole publisher、期待 vehicle 数、startup fatal の valid-run barrier を実装する
- [ ] invalid/incomplete/infra unavailable run を反復数に含めず、`INVALID / BLOCKED` とする
- [ ] B-02 `make dev ROS_DOMAIN_ID=1` で Domain 1 baseline を取得する
- [ ] B-03 `make gate1 ROS_DOMAIN_ID=1` の baseline を取得する
- [ ] B-04 `make gate2 ROS_DOMAIN_ID=1` の baseline を取得する
- [ ] B-05 `make gate3 ROS_DOMAIN_ID=1` の baseline を取得する
- [ ] B-06 `make dev4` で Domain 1〜4、fallback、V2X baseline を取得する
- [ ] B-07 stale/non-finite/solver failure/stop request の replay fixture を取得する
- [ ] B-08 submit tar 作成/hash -> eval image build/identity -> `make eval` の順で baseline を取得する
- [ ] B-09 一時 config/test launch で trajectory topic mode の valid/invalid update を replay する

### 1.7 Phase 0b — 意図的修正（triage で必要な項目。現行で未充足の R-13 は必須）

- [ ] Phase 0b の各修正を Contract Conformance lane または Safety lane に分類し、同じ commit/PR に混ぜない
- [ ] Contract Conformance lane は static oracle が検出した external-contract RED を既存 `docs/interface/` の exact target へ実装適合させる修正だけに限定し、契約文書、endpoint/type/Domain/schema 自体を変更しない
- [ ] Contract Conformance lane の before/after exact oracle を通し、external-contract RED を 0 件にしてから Safety lane へ進む
- [ ] Safety lane entry として H-01〜H-08 の signal、単位、authoritative criterion、観測方法が確定している
- [ ] 未解消 hard-safety RED、対象 code path、是正内容、reviewer、before/after fixture を D 台帳へ登録し、未登録の構造変更を混ぜない
- [ ] 修正失敗・回帰 mismatch 時に Baseline candidate v0 へ戻せる rollback 手順と停止条件を承認する
- [ ] 各修正を構造リファクタと別 commit/PR にする
- [ ] 修正を「既存 interface contract への適合」または「外部契約を変えない安全修正」に限定する
- [ ] topic/type/Domain/control method/result schema 自体の変更を本 Phase に入れない
- [ ] 変更理由、安全/評価影響、before/after、test を記録する
- [ ] 修正後に該当 scenario と回帰 matrix を再実行する
- [ ] Baseline candidate v0 から Full Baseline v1 anchor への intentional delta を保存する
- [ ] D-12 で未充足と確認した R-13 を、`ValidatedHardSafetyLimits`、`HardSafetyCommandValidator`、prebuilt `PrevalidatedSafeStop`、context-matched `GuaranteedTerminalStop`、one-shot validation、`FatalSafetyFault` として Phase 4 に持ち越さず、この Phase の独立安全修正として実装・検証する

### Scoped Solver Baseline（Phase 1 開始条件）

- [ ] baseline source commit、config/resource、compiler、OSQP、binary/image identity が固定されている
- [ ] `Contract/Safety Floor` と、solver input validation・non-finite・constraint violation・fallback に関係する hard-safety oracle が確定している
- [ ] Online MPC fixture が canonical `P/A/q/l/u`、accepted status、objective、max violation、first control、prediction、failure/fallback sequence を持つ
- [ ] Reference Speed Profile fixture が Online MPC と別に canonical `P/A/q/l/u`、生成 `v_ref`、return/statusを持つ
- [ ] 各 solver artifact に `ProblemKind::OnlineMpc` / `ProblemKind::ReferenceSpeedProfile` の discriminant があり、generic backend の runtime 分岐には使っていない
- [ ] Reference Speed Profile の `N < 2`、invalid input、OSQP failure時の `compute_speed_profile()` と各 caller の startup failure / previous-state保持 semantics を固定する
- [ ] Online MPC / Reference Speed Profile の各fixtureを同じ初期状態から2回以上replayし、正規化結果が決定的である
- [ ] D-12 の判断と必要な Phase 0b 安全修正が完了し、同じ承認済み anchor から solver artifact を取得している
- [ ] B-01、B-02 solver smoke、B-07 solver/final-command fault replay が PASS である
- [ ] solver surface に未説明の baseline mismatch、external-contract RED、hard-safety RED がない
- [ ] Scoped Solver Baseline の reviewer/承認者と参照 artifact hash が記録されている

### Scoped Path Baseline（Phase 2A 開始条件）

- [ ] Phase 1 DoD と常時 `Contract/Safety Floor` が green である
- [ ] CSV/topic、circular/open、補間、平滑化、曲率、base speed、左右幅の path fixture が固定されている
- [ ] valid trajectory update の採用 path/version/hash と、invalid update 拒否後の last-valid path/version/hash 保持をfixture化する
- [ ] B-09 が Phase 2A の事前条件として PASS し、一時config/test launchがcanonical configを変更していない
- [ ] B-02 path smoke と trajectory producer / consumer contract が PASS である
- [ ] path surface に未説明の baseline mismatch、external-contract RED、hard-safety RED がない
- [ ] Scoped Path Baseline の reviewer/承認者と参照 artifact hash が記録されている

### Full Baseline v1（Phase 2B-0 開始条件）

- [ ] static manifest、runtime evidence、全fixture、tolerance が揃っている
- [ ] D-01〜D-12 が許可された分類と根拠を持つ
- [ ] external-contract RED と hard-safety RED が 0 件である
- [ ] H-01〜H-08 に FAIL/UNRESOLVED がない
- [ ] 機能性能上の waiver は owner/issue/期限/scenario/承認者を持つ
- [ ] arbitration は canonical runtime / deterministic synthetic / dormant pure-rule の各欄が PASS または正当な N/A である
- [ ] R-13 の valid / one-shot `PrevalidatedSafeStop` / `FatalSafetyFault` / `FatalSafeStopValidationFailure` fixture が固定されている
- [ ] B-01/B-02/B-07/B-08/B-09 は PASS である
- [ ] B-03〜B-06 は PASS、または非 safety/contract の承認済み KNOWN_RED である
- [ ] N/A は文書で指定した optional cell だけに使われ、実行不能は BLOCKED である
- [ ] `LegacyReplayHarness` の必須fixtureが2回以上同じ正規化outputを返す
- [ ] 最終 baseline source commit、source/config/resource、comparison schema、binary、dev/eval image、submit tar の identity を再取得している
- [ ] Phase 1/2A candidate artifact と baseline reference artifact が分離され、referenceが上書きされていない
- [ ] Full Baseline v1 がレビュー・承認され、Phase 2B-0 開始時点で未説明の mismatch がない

## 2. Phase 1 — OSQP backend / QP Solver 分離

- [ ] `Contract/Safety Floor` と Scoped Solver Baseline が承認済みである
- [ ] generic matrix/vector-in/result-out の `OsqpProblem` / `OsqpResult` と、online 用 `MpcProblem` / `MpcSolveResult` の最小値型を分けて導入する
- [ ] 現行 OSQP settings と accepted status だけを持つ component-local `OsqpBackendConfig`、online validation 用 `QpSolverConfig`、speed-profile 用 `ReferenceSpeedProfileConfig` を導入する
- [ ] 既存 `solve_osqp()` を stateless な `OsqpBackend` に mechanical move する
- [ ] online `QpSolver` と `ReferenceSpeedProfileOptimizer` の両 caller を同じ `OsqpBackend` へ接続する
- [ ] OSQP settings、accepted status、problem 別 validation を現行のまま移す
- [ ] online の behavior/recovery/failure counter は compatibility caller 側にそのまま残し、Phase 2B-5 の `MpcExecutionAdapter` 抽出まで `QpSolver` へ移さない。speed-profile の startup/path-update failure semantics も caller 側に残す
- [ ] feasible case の unit test を追加する
- [ ] infeasible/non-finite/constraint violation の unit test を追加する
- [ ] Online MPC fixtureでstatus/objective/max violation/first control/predictionとfallback sequenceを比較する
- [ ] Reference Speed Profile fixtureで生成 `v_ref`、return/status、caller failure semanticsを比較する
- [ ] Online MPC と Reference Speed Profile の failureを同じfallback semanticsとして誤統合しない
- [ ] canonical sparse shape/index/layoutはexact、numericはPhase 0 tolerance内であり、triplet/CSC内部順に依存しないことを確認する
- [ ] full solutionは補助tolerance比較とし、非一意解でもobjective/constraint/first controlの同値性を優先する
- [ ] ROS/launch/config/topic に差分がないことを確認する
- [ ] `make autoware-build` を実行する
- [ ] 対象 package test を実行する
- [ ] Online MPC / Reference Speed Profile のdeterministic QP/fault fixtureとB-02 smokeを実行する
- [ ] `docs/spec/mpc-integration.md` の solver 境界を更新する

### Phase 1 DoD

- [ ] `OsqpBackend` / `QpSolver` は ROS、V2X、path、Domain、暗黙時刻に依存しない
- [ ] `OsqpBackend` / `QpSolver` は aggregate `Config` / `MpcConfig` を受けず、それぞれの component-local config と明示 problem だけを参照する
- [ ] `ReferenceSpeedProfileOptimizer` は Online MPC の fallback/failure counter を参照せず、startup/path-update の現行 failure semantics を維持する
- [ ] solver extraction 前後の behavior/fallback/failure sequence が一致する
- [ ] Online MPC / Reference Speed Profile の両 fixture と Solver/QP verification slice が green である

## 3. Phase 2A — Base path read-only 化

- [ ] `Contract/Safety Floor`、Phase 1 DoD、Scoped Path Baseline が承認済みである
- [ ] base path と dynamic per-cycle data の現行 write site を列挙する
- [ ] geometry、CSV/topic更新条件、circular/open semanticsだけを持つcomponent-local `BasePathConfig` を導入する
- [ ] `BasePathStore` / immutable snapshot API を導入する
- [ ] `ReferencePath` / `BasePathSnapshot` の waypoint、segment、width、constraint vector をprivate化する
- [ ] `get_waypoint_mutable()` とmutable vectorを返すAPIを削除し、friendによる迂回を追加しない
- [ ] consumerのpublic vector参照をconst view/read APIへ置換する
- [ ] base speed、周期 `v_ref`、dynamic path constraint、corridor targetをbase geometryとは別のdynamic overlayへ分離する
- [ ] base pathの生成/置換をBuilder/Storeだけに限定し、valid updateだけがsnapshot versionを進める
- [ ] invalid trajectory update時はlast-valid snapshot/version/hashを保持し、部分更新しない
- [ ] `SingleThreadedExecutor` の周期境界で通常のmove/assignmentを使い、atomic swapや新しい並行実行を導入しない
- [ ] CSV/topic、circular/open の fixture を追加する
- [ ] 補間、平滑化、曲率、base speed、左右幅を比較する
- [ ] canonical CSV schema/header/行数を維持する
- [ ] trajectory producer を維持する
- [ ] build/package test/characterization を実行する
- [ ] B-02 と B-09 を再実行し、Scoped Path Baseline の last-valid path 保持を candidate でも確認する
- [ ] `docs/spec/mpc-integration.md` の path ownership を更新する

### Phase 2A DoD

- [ ] base vector/mutable getterがcompile-timeに非公開で、consumerが直接変更できない
- [ ] componentはaggregate `Config` / `MpcConfig`を受けず、`BasePathConfig`とimmutable snapshotだけを参照する
- [ ] base snapshotは周期前後で同じversion/hashを保ち、dynamic overlayだけが周期単位で更新される
- [ ] valid/invalid trajectory updateを含むbase pathの全観測値がbaselineと一致する

## 4. Phase 2B — V2X / FrontRisk / Behavior 段階分離

- [ ] `Contract/Safety Floor`、Phase 2A DoD、Full Baseline v1 がレビュー・承認済みである
- [ ] 現行 V2X tracker、FrontRisk、corridor/gap、behavior FSM、OvertakeLine の全input/state/output/write siteを列挙する
- [ ] tracker history、Behavior FSM、corridor target lock、OvertakeLine phase/target lockのownerを重複なしで定義する
- [ ] Phase 2B-1〜2B-5 の新 component は各 subphase で test-only shadow path とし、独立 shadow state で legacy production cycle と比較する。command publish、legacy FSM/corridor state、ROS graph を変更しない
- [ ] Phase 2B-6 の production cutover では現行の反映用再計算だけを行う一時的 `LegacyCorridorCommitAdapter` を使い、Phase 2C-1 で削除する。長期 runtime dual path を残さない

### Phase 2B-0 — `CycleSnapshotIdentityProvider`

- [ ] timer callback 冒頭で cycle ID と ROS/steady cycle time を一度だけ取得する `CycleSnapshotIdentityProvider` を façade 内に抽出する
- [ ] Phase 2A の path version と、façade が保持する config/V2X/session epoch・source stamp を accepted update/event ごとに単調増加させ、一つの `SnapshotIdentity` に束ねる
- [ ] 同一周期の shadow artifact へ同じ identity を渡し、周期途中の再採取や component ごとの identity 再生成を禁止する
- [ ] full `CycleInput`、pending config aggregate、config compatibility loader、executor 変更を先取りしない
- [ ] clean initial state と同じ event prefix から得る identity sequence を Full Baseline v1 の trace と比較する
- [ ] build/package test、identity deterministic fixture、B-02 smoke を実行する
- [ ] `docs/spec/mpc-integration.md` に cycle/snapshot identity owner と increment 条件を反映する

#### Phase 2B-0 DoD

- [ ] cycle/path/config/V2X/session version と ROS/steady/source time の owner・increment 条件が一意である
- [ ] 一周期内の全 shadow artifact が同じ `SnapshotIdentity` を持ち、外部 I/O/timer/executor に差分がない

### Phase 2B-1 — `V2xSnapshotBuilder` / Tracker

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2B-0 DoD が green である
- [ ] freshness、ID、jump、velocity推定だけを持つcomponent-local `V2xTrackerConfig` を導入する
- [ ] ROS message変換をadapterへ閉じ、core値型 `V2xSnapshot` とvalidation resultを作る
- [ ] source/receipt時刻、message order、missing/duplicate/empty ID、invalid sampleを明示する
- [ ] tracker historyとcompleteness stateの唯一のownerを`V2xSnapshotBuilder`にする
- [ ] façade の V2X input epoch ownership を `V2xSnapshotBuilder` へ一度だけ移し、accepted snapshot update 以外で version を進めず、二重 increment を残さない
- [ ] stale、out-of-order、position jump、empty/duplicate ID、Domain 1〜4 message count fixtureを比較する
- [ ] build/package test、V2X deterministic fixture、B-02/B-06 smokeを実行する
- [ ] `docs/spec/mpc-integration.md` のV2X input/snapshot境界を更新する

#### Phase 2B-1 DoD

- [ ] `V2xSnapshotBuilder` はROS publisher/logger、path、behavior、QPを参照しない
- [ ] aggregate `Config` / `MpcConfig`を受けず、snapshot/history/validationがbaselineと一致する

### Phase 2B-2 — `FrontRiskClassifier`

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2B-1 DoD が green である
- [ ] distance、relative speed、required decel、TTC、risk thresholdだけを持つcomponent-local `FrontRiskConfig` を導入する
- [ ] front vehicle classification と `FrontRiskAssessment` / `FrontRiskLevel` 算出を pure extraction する
- [ ] curve guard、moving/stopped front、inside stopping distance、境界値fixtureを追加する
- [ ] proposal計算中にtracker/FSM/corridor stateを変更しないことを確認する
- [ ] build/package test、FrontRisk deterministic fixture、B-02 smokeを実行する
- [ ] `docs/spec/mpc-integration.md` のFrontRisk input/outputを更新する

#### Phase 2B-2 DoD

- [ ] 同じ immutable input/config から同じ `FrontRiskAssessment` が得られる
- [ ] `FrontRiskClassifier` はROS、env、Domain、FSM、QPを参照しない

### Phase 2B-3 — `CorridorProposalBuilder`

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2B-2 DoD が green である
- [ ] gap 幅、vehicle inflation、wall margin、reachability だけを持つ component-local `CorridorProposalConfig` を導入する
- [ ] `Base`、`FollowPreposition`、`LowSpeedAvoidance`、`OvertakeLeft/Right`、`OvertakeFallbackLeft/Right` の `CorridorProposal` と reject reason を side-effect なしで生成する
- [ ] proposal ID、feasibility、side、bounds、target、speed limit、base waypoint ID、horizon size、生成 ROS/steady time、`SnapshotIdentity` を各候補に持たせる
- [ ] Cruise/SafetyBrake/候補不要時も明示 `Base` proposal を選択し、optional ID や magic sentinel で validation を迂回しない
- [ ] proposal段階ではtarget lock、pass side、FSM、base path、constraint vectorを変更しない
- [ ] no-gap、both-side、vehicle-vehicle、multi-front、curve forbidden、reachable/unreachable fixtureを追加する
- [ ] proposalのbounds/target/velocity limit/reasonをlegacy結果と比較する
- [ ] build/package test、corridor proposal fixture、B-02/B-06 smokeを実行する
- [ ] `docs/spec/mpc-integration.md` のcorridor proposal境界を更新する

#### Phase 2B-3 DoD

- [ ] proposal API は immutable `BasePathView` / `V2xSnapshot` / typed config だけを参照する
- [ ] proposal を複数回評価しても component state と結果が変わらない

### Phase 2B-4 — `RaceBehaviorPlanner`

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2B-3 DoD が green である
- [ ] state transition、hold time、target selection だけを持つ component-local `RaceBehaviorConfig` を導入する
- [ ] `BehaviorInput` に ego projection、FrontRisk、`CorridorProposalSet` summary、時刻、previous `BehaviorPlannerState` を明示し、raw `MpcSolveResult` / `MpcExecutionOutcome` / solver feedback を含めない
- [ ] `BehaviorSelection` と proposed next state を返す mechanical extraction を行う
- [ ] `propose()`ではFSM stateを変更せず、周期末の明示`commit()`で一度だけstate transitionする
- [ ] Cruise/Follow/Overtake/LowSpeedAvoidance/SafetyBrakeのproposal/commit sequence fixtureを追加する
- [ ] stale V2X、missing ID、curve forbidden、no-gap の閾値/状態列を比較する
- [ ] build/package test、behavior deterministic fixture、B-02/B-03〜B-06を実行する
- [ ] `docs/spec/mpc-integration.md` のBehavior input/output/state ownerを更新する

#### Phase 2B-4 DoD

- [ ] `RaceBehaviorPlanner` はROS、publisher、logger、env、Domain、QPを参照しない
- [ ] rejected/未採用proposalがFSM stateを変更せず、commit後のstate/category/reasonがbaselineと一致する

### Phase 2B-5 — `OvertakeLinePlanner`

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2B-4 DoD が green である
- [ ] preflight failure / problem-build failure / `MpcSolveResult` を discriminated `MpcExecutionAttempt` として一回で受け、solver setup/status、solver-to-control conversion とその MPC-internal non-finite まで正規化する `MpcExecutionAdapter` と component-local `MpcExecutionConfig` を抽出する
- [ ] `MpcExecutionStepResult` に candidate/state を持たない `MpcExecutionOutcome`、success 時の raw nominal control または failure 時の fallback candidate、`SolverExecutionFeedback`、next state を別 field で持たせる
- [ ] shadow chain 内の failure counter、overtake failure counter、fallback speed、last category の唯一 owner を shadow `SolverExecutionState` にし、solve 前に確定した同周期の `OvertakeLineDecision.phase` を明示 input とする。legacy production owner は Phase 2B-6 まで変更しない
- [ ] adapter の `execute()` は arbitration 前に `MpcExecutionStepResult` を一度だけ返し、arbitration result、ROS、behavior callback を参照しない
- [ ] phase、target lock、reacquire、solver-failure recovery だけを持つ component-local `OvertakeLineConfig` を導入する
- [ ] Phase 2B-5 の shadow coordinator だけが周期末に `MpcExecutionAdapter::project_overtake_feedback()` を一度呼び、cycle `t` の `SolverExecutionFeedback` から solver 固有情報を除いた `OvertakeExecutionFeedback` を shadow の cycle `t+1` 入力にする。production feedback/state へ書き込まない
- [ ] ShiftOut/Pass/Return/Recoveryのproposalとproposed next stateをpure境界へ抽出する
- [ ] selected proposal bounds/target metadata と base-path lateral geometry だけから horizon target array/active mask を `OvertakeLineDecision` に生成し、V2X/gap/free interval を再計算しない
- [ ] `OvertakeLinePlanner` を selected target/side に対する phase/continuity lock の唯一 owner とする。戦術上の target/pass-side 選択は `RaceBehaviorPlanner` の `BehaviorSelection` として current-cycle input に固定し、mutable state を共有しない
- [ ] proposal中はphase/lockを変更せず、明示`commit()`で一度だけ更新する
- [ ] target lost/jump/reacquire/rear-clear/curve/solver failure の proposal/commit fixture を追加する
- [ ] build/package test、OvertakeLine deterministic fixture、B-02/B-03〜B-06 と B-07 の preflight/problem-build/conversion/non-finite fault matrix を実行する
- [ ] `docs/spec/mpc-integration.md` のOvertakeLine state ownerを更新する

#### Phase 2B-5 DoD

- [ ] OvertakeLine proposal/commit前後のtarget ID、pass side、phase、reasonがbaselineと一致する
- [ ] Behavior FSM、corridor target lock、`SolverExecutionState` を `OvertakeLinePlanner` が直接変更しない
- [ ] shadow chain 内では `MpcExecutionAdapter` 以外が shadow failure counter、fallback speed、last category を変更せず、legacy production state は不変である
- [ ] `RaceBehaviorPlanner` に raw `MpcSolveResult` / `MpcExecutionOutcome` / `SolverExecutionFeedback` を渡さず、同周期の behavior/corridor 再実行と second solve がない

### Phase 2B-6 — Selection handoff / integration check

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2B-5 DoD が green である
- [ ] V2X、FrontRisk、`CorridorProposalSet`、`BehaviorSelection`、`OvertakeLineDecision` と各 state transition を別 field で fixture へ保存する
- [ ] state transition の順序・回数と rejected proposal の非介入を legacy cycle sequence と比較する
- [ ] selected proposal ID、target、pass side、`SnapshotIdentity` を明示的に引き渡し、shadow chain が baseline 内であることを確認してから decision chain を production caller へ切り替える
- [ ] production cutover は node restart + clean session/reset boundary に限定し、shadow/legacy state を production state へ copy しない。同じ initial state/input prefix の replay で初期化と遷移を確認する
- [ ] `LegacyCorridorCommitAdapter::apply(const LegacyCorridorCommitRequest &, const LegacyCorridorState &)` に corridor 反映を閉じ、現行 `update_last_target=true` 相当の application pass と next corridor state だけを返す。legacy evaluation/Behavior/OvertakeLine/tracker/solver counter を呼ばず、new/legacy planner の二重 state mutation を許さない
- [ ] Phase 2B では `LocalCorridorPlanner::commit` 抽出と no-recompute 化を先取りしない
- [ ] `OvertakeExecutionFeedback` は明示経路で次周期の `OvertakeLinePlanner` / corridor continuity へ渡し、component 間の暗黙 callback を追加しない
- [ ] `MpcExecutionAdapter` も production caller へ切り替え、同じ subphase で legacy failure counter/fallback mutation を削除して `SolverExecutionState` を唯一の production owner にする
- [ ] production cutover と同時に cycle-tail projection の唯一 owner を shadow coordinator から compatibility façade へ移し、production path に shadow call site を残さない
- [ ] 全component-local typed configのeffective値がlegacy resolver結果と一致する
- [ ] build/package test、V2X/behavior full deterministic fixture、Solver Execution/Fault verification slice、B-02〜B-07を実行する
- [ ] `make dev4` でV2X接続、Domain 4 fallback、状態列、proposal/commit eventを確認する

#### Phase 2B DoD

- [ ] 各 state owner が一つで、proposal は side-effect なし、Behavior/OvertakeLine の state transition は各周期最大一回である
- [ ] shadow の state/category、target speed/offset、proposal/selection、target ID、pass side、phase、SafetyBrake request が legacy baseline と一致する
- [ ] production command/behavior/corridor は明示 adapter 経由で baseline と一致し、旧 decision path と runtime dual 実行していない
- [ ] production の failure counter/fallback speed/last category は `SolverExecutionState` だけが所有し、B-07 の同周期 fallback / 次周期 transition が baseline と一致する
- [ ] Full Baseline v1 comparison と V2X/Behavior verification slice が green である

## 5. Phase 2C — Corridor commit / LocalReferenceBuilder 分離

### Phase 2C-1 — `LocalCorridorPlanner::commit`

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2B DoD が green である
- [ ] continuity margin、side/target lock、identity validation だけを持つ component-local `LocalCorridorConfig` を導入する
- [ ] 同じ `CorridorProposalSet`、`BehaviorSelection`、`OvertakeLineDecision`、previous `LocalCorridorPlannerState` から `CorridorPlan` と next state を返す `commit()` を抽出する
- [ ] proposal ID、path/V2X/config/session version、cycle ID、ROS/steady cycle time、source timestamp、base waypoint ID、horizon size の exact match を必須にする
- [ ] `commit()` は選択済み proposal の validity、side/target continuity だけを処理し、V2X/path の再読込、再射影、free interval 再計算、代替 proposal 生成を行わない
- [ ] commit は一周期に高々一回とし、committed `CorridorPlan` を周期中 immutable にする
- [ ] `LegacyCorridorCommitAdapter` を `LocalCorridorPlanner::commit` に置換し、同じ subphase 内で adapter と legacy の evaluation/反映再計算 path を削除する
- [ ] production runtime に dual path を残さず、proposal -> selection -> OvertakeLine -> commit が唯一の corridor chain である
- [ ] proposal/selection/commit の値、reason、identity、commit count と、identity/proposal mismatch failure fixture を比較する
- [ ] build/package test、corridor deterministic/fault fixture、B-02〜B-06 を実行する
- [ ] `docs/spec/mpc-integration.md` の corridor commit/state owner 境界を更新する

#### Phase 2C-1 DoD

- [ ] `LocalCorridorPlanner::commit` は選択済み proposal を再計算せず、proposal ID/identity 不一致を別候補への fallback で隠さない
- [ ] side/target continuity state の owner が一つで、commit 回数と `CorridorPlan` が baseline と一致する

### Phase 2C-2 — `OperationalLimitResolver / LocalReferenceBuilder`

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2C-1 DoD が green である
- [ ] Domain/start window/ref_vel section の現行速度上限適用順を fixture 化する
- [ ] 速度上限、start window、Domain override の解決値だけを持つ component-local `OperationalLimitConfig` を導入する
- [ ] horizon sample、補間、境界、validation だけを持つ component-local `LocalReferenceConfig` を導入する
- [ ] `OperationalLimits` と `OperationalLimitResolver` を抽出する
- [ ] target speed/lateral/corridor の現行適用順を fixture 化する
- [ ] `ReferenceHorizon` 値型と validation を追加する
- [ ] `LocalReferenceBuilder` に周期 reference 生成を抽出する
- [ ] Phase 2A の immutable base snapshot と dynamic overlay を入力にし、base path への周期速度/constraint mutation を再導入しない
- [ ] `BehaviorSelection` と committed `CorridorPlan` を別入力にし、採用済み decision だけを horizon に反映する
- [ ] `LocalReferenceBuilder` は proposal/selection/commit、V2X/path read を再実行せず、同じ `SnapshotIdentity` の `CorridorPlan` だけを参照する
- [ ] `OperationalLimitResolver` から Domain/env/YAML の直接参照を除き、adapter が解決した typed config と明示 runtime context だけを渡す
- [ ] sample 数、単位、frame、finite、左右境界 invariant test を追加する
- [ ] 単車/V2X/追い越し/制動 horizon を baseline と比較する
- [ ] build/package test/characterization を実行する
- [ ] 影響する B-02〜B-06 を実行する
- [ ] `docs/spec/mpc-integration.md` の operational limit/reference 境界を更新する

### Phase 2C DoD

- [ ] base path snapshot と committed `CorridorPlan` は周期の前後で不変である
- [ ] `OperationalLimitResolver` / `LocalReferenceBuilder` は aggregate `Config` / `MpcConfig`、YAML、env を参照しない
- [ ] rejected/未commit proposal が horizon と dynamic overlay を変更しない
- [ ] proposal/selection/commit/reference の identity が一致し、horizon 全 field が tolerance 内で一致する

## 6. Phase 2D — MpcProblemBuilder 分離

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2C DoD が green である
- [ ] `init_problem()` の QP 構築 input を列挙する
- [ ] horizon、model、cost、constraint、scaling だけを持つ component-local `MpcProblemConfig` を導入する
- [ ] previous control sequence、previous steering、last solved waypoint、prediction linearization history を `MpcControlHistoryState` として明示し、failure counter/fallback speed の `SolverExecutionState` と分離する。`MpcWarmStartState` と呼ばない
- [ ] `MpcProblemBuilder` に matrix/vector 構築を抽出する
- [ ] V2X/FSM/lap/Domain/env/time と aggregate `Config` / `MpcConfig` 依存を builder から除く
- [ ] behavior と solver の failure reason を分離する
- [ ] `P/A/q/l/u` を shape、三角格納方針、変数順、constraint row block、重複集約後の canonical `(row,col)` pattern で比較する
- [ ] canonical shape/index/layout は exact、numeric は field 別 tolerance とし、triplet 挿入順と Eigen/CSC 内部格納順を同値性判定に使わない
- [ ] objective/max violation/first control/prediction を solver 結果の主比較、full solution を補助比較とする Phase 1 schema を維持する
- [ ] solver failure 後の状態列を baseline と比較する
- [ ] build/package test/characterization を実行する
- [ ] 影響する B-02〜B-06 と deterministic QP fixture を実行する
- [ ] `docs/spec/mpc-integration.md` の problem builder 境界を更新する

### Phase 2D DoD

- [ ] 同じ prepared reference から同じ QP が作られる
- [ ] builder は `MpcProblemConfig` と明示 input だけを参照する
- [ ] builder に previous solver result、behavior side effect、solver invocation がない
- [ ] canonical sparse comparison と Problem verification slice が green である

## 7. Phase 3 — One-cycle API

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 2D DoD が green である
- [ ] `CycleInput`、`CycleOutput`、`CycleState` を定義する
- [ ] callback snapshot assembly と cycle-boundary swap だけを持つ component-local `CycleAdapterConfig` を導入する
- [ ] Phase 2B-0 の `CycleSnapshotIdentityProvider` と既存 epoch owner を再利用し、adapter 境界に Phase 2D までに導入済みの component-local typed config を束ねた immutable `ControllerConfigSnapshot` を定義する。config version/identity の別生成経路を作らない
- [ ] flat YAML / Domain override / dynamic parameter から typed config 値を作る既存 mapping と precedence は変更せず、pure mapping の整理を Phase 5 に残す
- [ ] cycle-tail projection call を compatibility façade から one-cycle orchestrator の `step()` commit へ機械的に移し、同じ subphase で façade 側 call site を削除して一周期一回を維持する
- [ ] `awsim_vehicle_state`、`local_controller_enabled`、`local_stop_requested` を別 field にする
- [ ] `/awsim/control_mode_request_topic` と内部 `/control/control_mode_request_topic` を remap/統合しない
- [ ] callback data を周期先頭で immutable snapshot にする
- [ ] dynamic parameter を次周期の完全な `ControllerConfigSnapshot` として一度だけ反映し、周期途中で component config を混在させない
- [ ] snapshot schema、config epoch、complete config candidate の schema/coherence validation、pending→active aggregate commit の唯一 owner を compatibility façade から Phase 3 の cycle adapter へ一度だけ移し、旧 epoch increment/commit site を削除する。Phase 4 は同じ version owner のまま自身の typed-config field だけを追加する
- [ ] Phase 3 以降、path/V2X/session の ROS callback/façade は raw input を各 domain owner へ渡し、validation 済み immutable candidate/ref を cycle adapter が集約する。config callback は complete config candidate を渡すだけとし、active aggregate、config epoch increment、swap を持たない
- [ ] path/V2X/session の domain validation、accepted-version increment、last-known-good は `BasePathStore` / `V2xSnapshotBuilder` / `SessionState` に残し、cycle adapter で重複実装しない
- [ ] timer 冒頭の順序を `begin_cycle() -> validation 済み domain refs と complete config candidate の commit -> active version/source-stamp read -> seal_identity()/CycleInput` に固定する
- [ ] 一つの invalid source はその owner の last-known-good だけを維持し、他 source の valid update を rollback する global transaction を導入しない
- [ ] `SnapshotIdentity` の path/config/V2X/session version と active snapshot version を construction 時に exact 検証し、config 更新境界で `identity.config_version == ControllerConfigSnapshot.version` を確認する deterministic fixture を追加する
- [ ] path invalid + config valid、V2X invalid + path valid の更新境界 fixture で、valid source だけが進み identity が各 active version を正しく表すことを確認する
- [ ] invalid complete config + valid path/V2X の更新境界 fixture で、config は last-known-good snapshot/version を維持し、valid domain refs だけが進み、identity がその混成 active version を正しく表すことを確認する
- [ ] 各 component には aggregate snapshot ではなく自身の typed config slice だけを渡す
- [ ] `update_*()` + `get_control()` の順序依存を一つの `step()` に置換する
- [ ] core 内の暗黙 `now()`、env、subscriber state 参照を除く
- [ ] incomplete/invalid input の status を明示する
- [ ] cycle replay で output/state を baseline と比較する
- [ ] output inter-arrival と、trace seam 取得可能時の timer/solver timing を別々に比較する
- [ ] SingleThreadedExecutor と外部 I/O を維持する
- [ ] build/package test、B-02〜B-07 を実行する
- [ ] `docs/spec/mpc-integration.md` の cycle API/state ownership を更新する

### Phase 3 DoD

- [ ] 同じ initial state/input から同じ output/next state が得られる
- [ ] public core API に caller の順序依存がない
- [ ] 一周期内の全 component が同じ config version を観測し、component は aggregate config を参照しない
- [ ] pending/active aggregate refs、config epoch、周期境界 swap の owner は cycle adapter 一つで、`CycleInput` identity の全 version が対応する active snapshot と一致する。domain candidate の validation/version owner は重複していない
- [ ] Full Baseline v1 comparison と Cycle verification slice が green である

## 8. Phase 4 — PostProcessor / Safety / Arbiter 段階分離

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 3 DoD が green である
- [ ] raw candidate、postprocessed candidate、safety assessment、selected result、final validation、publish result の現行順序と全 write site を列挙する

### Phase 4A — `ControlPostProcessor`

- [ ] gain、filter、rate、既存 clamp、state update だけを持つ component-local `ControlPostProcessorConfig` を導入する
- [ ] Phase 3 の `ControllerConfigSnapshot` を同じ config epoch/commit owner のまま `ControlPostProcessorConfig` field で拡張する
- [ ] acceleration/steering postprocess を `ControlPostProcessor` に mechanical extraction する
- [ ] `PostProcessorInput` / `PostProcessorState` / `PostprocessedCandidate` を明示し、filter/history state の owner を一つにする
- [ ] 現行の gain/filter/clamp 適用順、raw/postprocessed 値、state update を fixture 化する
- [ ] finite/non-finite、angle/rate 境界、reset、停止 candidate の unit test を追加する
- [ ] postprocess 中に arbitration、SafetyBrake、Boost/gear、publish を行わない
- [ ] build/package test、raw/postprocessed deterministic fixture、B-02/B-07 smoke を実行する
- [ ] `docs/spec/mpc-integration.md` の postprocess 境界を更新する

#### Phase 4A DoD

- [ ] `ControlPostProcessor` は `ControlPostProcessorConfig` と明示 input/state だけを参照し、ROS、aggregate config、arbitration state を参照しない
- [ ] raw/postprocessed command と filter state が baseline と一致する

### Phase 4B — `SafetySupervisor` / policies

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 4A DoD が green である
- [ ] stale、non-finite、input completeness、mandatory stop、shutdown 条件だけを持つ component-local `SafetySupervisorConfig` と、SafeStop template/context だけを持つ `SafeStopConfig` を導入する
- [ ] `RecoveryPolicyConfig`、`BoostPolicyConfig`、`GearPolicyConfig` と各 component-local state を導入し、Recovery/Boost/gear を pure policy として mechanical extraction する
- [ ] Phase 3 の `ControllerConfigSnapshot` を同じ config epoch/commit owner のまま Phase 4B の typed-config fields で拡張する
- [ ] `SafetyInput` / `SafetyAssessment` / `RequiredSafetyAction` を導入し、安全判定を `SafetySupervisor` に抽出する
- [ ] `SafetySupervisor` は priority selection、publish、Boost/gear action、postprocess state を変更しない
- [ ] nominal/stale odom、invalid input、solver failure、disabled/stop、shutdown、SafetyBrake request の判定 fixture を追加する
- [ ] Phase 0b で承認済みの startup hard-safety validation と immutable `ValidatedHardSafetyLimits` を mechanical extraction し、FAIL/UNRESOLVED では走行開始しない semantics を変えない
- [ ] Phase 0b で承認済みの SafeStop template/factory を抽出し、`ValidatedHardSafetyLimits` だけから構築して通常 config や reject 済み candidate を参照しない
- [ ] Phase 0b で承認済みの pure `HardSafetyCommandValidator` を抽出し、SafeStop 事前検証と Phase 4C の final validation で共有できる API にする
- [ ] 各周期の nominal pipeline より前に、検証済み template、current validated gear context、直前に publish 済みの gain 適用後 final steering だけから `PrevalidatedSafeStop` を一度構築・検証する承認済み順序を維持する
- [ ] policy は候補/action/reason だけを返し、source priority、ROS publish、他 policy state、fatal latch を変更しない
- [ ] `SafetySupervisorState` を `FatalSafetyFault` latch の唯一 owner とし、context-matched `GuaranteedTerminalStop` の command identity / validated gear-context version を保持する
- [ ] build/package test、safety deterministic/fault fixture、B-02/B-07 smoke を実行する
- [ ] `docs/spec/mpc-integration.md` の safety assessment 境界を更新する

#### Phase 4B DoD

- [ ] 同じ immutable safety input/config から同じ assessment/action/reason が得られる
- [ ] safety 判定に ROS publish、aggregate config、arbitration priority の副作用がない
- [ ] startup safety limit validation と周期先頭の `PrevalidatedSafeStop` 構築が nominal candidate の成否から独立している
- [ ] Recovery/Boost/gear policy の state owner が一つで、canonical disabled runtime と dormant pure-rule fixture が baseline と一致する
- [ ] validator/policy/arbiter は fatal latch を持たず、`SafetySupervisorState` だけが fatal 状態と terminal-stop context validity を保持する

### Phase 4C — `CommandArbiter` / `FinalCommandValidator`

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 4B DoD が green である
- [ ] 現行 arbitration 優先順位表を `design.md` に確定値で反映する
- [ ] candidate priority だけを持つ component-local `CommandArbiterConfig` を導入する
- [ ] required-field と one-shot substitution flow だけを持つ component-local `FinalCommandValidatorConfig` を導入する。authoritative final angle/slew、acceleration、finite、gear coherence は Phase 4B の `ValidatedHardSafetyLimits` / `HardSafetyCommandValidator` だけに保持し、重複設定しない
- [ ] Phase 3 の `ControllerConfigSnapshot` を同じ config epoch/commit owner のまま Phase 4C の typed-config fields で拡張する
- [ ] `ControlCandidate` / `ArbitrationDecision` / `FinalCommandResult` 値型を追加する
- [ ] `CommandArbiter` が候補と `SafetyAssessment` から一つだけを選び、command/gear/Boost/reasonを一結果に集約する
- [ ] `FinalCommandValidator` を全 gain/filter/既存 clamp と arbitration の後に置き、Phase 4B の `HardSafetyCommandValidator` を再利用して違反を暗黙 clamp せず reject-only にする
- [ ] selected result reject 時は通常 command と Boost/Recovery/gear を抑止し、その周期の同じ `PrevalidatedSafeStop` へちょうど一度だけ置換して再検証する。reject 後に SafeStop を生成し直さない
- [ ] SafeStop 自体が reject された場合は再帰 fallback せず `FatalSafetyFault` event を `FatalSafeStopValidationFailure` reason で返し、one-cycle orchestrator が周期末に `SafetySupervisorState` へ一度だけ commit する
- [ ] `FatalSafetyFault` latch 中は normal/Recovery/Overtake command と Boost/gear を再開せず、直前 final command identity / validated gear-context version が exact match する `GuaranteedTerminalStop` だけを publish 候補にする。存在しない場合は invalid command を合成しない
- [ ] stale odom vs nominal、solver fallback vs Overtake、disabled/stop vs Boost test を追加する
- [ ] Recovery vs nominal、stale gear vs Reverse の deterministic/dormant pure-rule test を追加する
- [ ] non-finite postprocess、final validation reject、SafeStop reject、shutdown while Reverse test を追加する
- [ ] steering gain 適用後の authoritative angle/rate invariant を確認する
- [ ] fault/forced stop/Recovery 中の Boost inhibit と Boost edge/start-once/re-arm sequence を比較する
- [ ] arbitration/validation/PrevalidatedSafeStop/`FatalSafetyFault` の candidate、reason、attempt count、latch、command/action sequence を fixture で比較する
- [ ] `/control/command/control_cmd` の唯一の publisher を ROS adapter に限定し、core/policy/validatorから publish しない
- [ ] canonical disabled の Recovery/gear runtime 欄は N/A とし、非canonical test-only simulation を実走等価性に使わない
- [ ] build/package test、B-02〜B-07 を実行する
- [ ] `make dev4` で final publisher ownership と Domain 分離を確認する
- [ ] `docs/spec/mpc-integration.md` の arbitration/final validation/fault ownership を更新する

#### Phase 4C DoD

- [ ] final command/gear/Boost/reason/source/validation result が一つの `FinalCommandResult` に集約されている
- [ ] final reject ごとの SafeStop attempt は最大一回で、SafeStop reject 後は terminal fault 以外の command path に戻らない
- [ ] core/policy/validator は ROS publish せず、publisher owner は一つである
- [ ] safety invariant と baseline arbitration/fault/shutdown sequence が全て green である

## 9. Phase 5 — Config/tools/package 整理

- [ ] `Contract/Safety Floor`、Full Baseline v1、Phase 4C DoD が green である
- [ ] flat YAML key と Domain override の互換 matrix を作る
- [ ] Phase 1〜4 で導入済みの全 component-local typed config と key/default/range/source/owner の対応表を作る
- [ ] compatibility loader を、既存 flat YAML、Domain override、dynamic parameter から typed config candidate を作り、Phase 3 の snapshot-candidate API へ渡す pure mapping に限定する。snapshot version、pending→active commit、周期境界 swap を所有させない
- [ ] legacy の default/fallback/override 解決順、missing/unknown/type/range error semantics を変えない
- [ ] component は loader、YAML node、env、aggregate `ControllerConfigSnapshot` を参照せず、自身の typed config slice だけを受ける
- [ ] Domain 1〜4、fallback、dynamic parameter update の loader unit test と `effective-config.json` exact comparison を追加する
- [ ] loader 出力を Phase 3 の既存 commit API へ渡し、次周期境界で完全な snapshot として反映されることを確認する。Phase 5 に別の update/commit 経路を追加しない
- [ ] pure C++ component の CMake target を整理する
- [ ] runtime と offline trajectory tool の依存を整理する
- [ ] package 分割の必要性を測定結果から判断する
- [ ] 分割する場合はすべて `aichallenge_submit/` 内に置く
- [ ] `mpc_controller_cpp`、`control/mpc.launch.xml`、config path の互換入口を残す
- [ ] `package.xml` と CMake dependency を一致させる
- [ ] offline CLI と canonical CSV を維持する
- [ ] `docs/spec/mpc-integration.md` を更新する
- [ ] node/package tree が変わる場合は `docs/spec/architecture.md` を更新する
- [ ] `make autoware-build` と package test を実行する
- [ ] `./create_submit_file.bash` と tar 最上位を確認する
- [ ] eval image build と `make eval` を実行する

### Phase 5 DoD

- [ ] launch/config/topic/tool/submission の互換性が確認済みである
- [ ] loader の全出力 field が component-local typed config のいずれかへ一意に対応し、未使用/二重解決がない
- [ ] Domain 1〜4 と dynamic parameter の effective config が baseline と一致する
- [ ] `aichallenge_system/` に participant 実装を移していない
- [ ] Config/Release verification slice と文書更新が完了している

### Final Full Verification（Phase 5 DoD 後の完了判定）

- [ ] 常時 `Contract/Safety Floor` の全項目が green である
- [ ] B-01〜B-09 の全 matrix を final candidate で再実行し、必須 cell に FAIL/BLOCKED/UNRESOLVED がない
- [ ] Online MPC、Reference Speed Profile、path、V2X、FrontRisk、corridor proposal/selection/commit、Behavior/Overtake、reference、QP、cycle、postprocess、安全、arbitration、PrevalidatedSafeStop/`FatalSafetyFault` の deterministic/fault fixture が green である
- [ ] sparse QP は canonical shape/index/layout exact と field 別 numeric tolerance の双方が green である
- [ ] hard-safety H-01〜H-08、final angle/slew、stale/non-finite/stop/shutdown、Boost/gear inhibit に FAIL/UNRESOLVED がない
- [ ] `/control/command/control_cmd` は sole publisher のままで、topic/service/type/QoS/Domain/launch/control method/result schema/`output/latest` 契約に未説明の差がない
- [ ] admin start one-shot/state strings、orchestrator/state-manager の禁止 pub/sub、result schema 主要キー、`output/latest/` / `HOST_UID/HOST_GID` の負方向・成果物 invariant が exact である
- [ ] Boost/gear/Recovery の禁止代替 endpoint・cross-domain・teleport/respawn と、submit tar の build-context 内配置が exact である
- [ ] `make autoware-build`、全対象 package test、submit tar 作成、eval image build、`make eval` が成功する
- [ ] submit tar の最上位が `aichallenge_submit/` であり、`aichallenge_system/`、生成物、rosbagを含めていない
- [ ] final source/config/resource/comparison schema/binary/dev・eval image/submit tar の identity と artifact hash を保存する
- [ ] final candidate evidence と Full Baseline v1 reference evidence が分離され、比較結果から最初の差分位置と reason を追跡できる
- [ ] `docs/spec/mpc-integration.md`、必要時のみ `docs/spec/architecture.md`、本 tasklist の実行結果が一致する
- [ ] Final Full Verification の reviewer/承認者、waiver、残課題、artifact retention を記録し、全体 DoD を承認する

## 10. Stop conditions

次のいずれかが起きたら次 Phase へ進まず、原因と判断を記録する。

- baseline input 自体が再現できない
- exact field に未説明の差が出る
- numeric 差が観測 jitter で説明できない
- `/control/command/control_cmd` の publisher が複数になる
- external topic/service/type/Domain/launch/config contract が変わる
- external-contract RED または hard-safety RED が未解消である
- 40 Hz deadline miss が baseline より悪化する
- stale/non-finite/fault 時に通常 command または Boost が出る
- dormant feature の有効/無効が意図せず変わる
- 変更が `aichallenge_system/`、実車、評価 schema に波及する
- baseline 根拠が ephemeral path だけにあり、fixture/hash/retention で追跡できない

## 11. 今回未実行の検証

本 planning 作業ではコード、launch、config を変更していない。Phase 0 の baseline 取得に必要な Docker/ROS/AWSIM 起動、build、package test、gate、V2X、eval は未実行で、`Contract/Safety Floor` も H-01〜H-08 が `UNRESOLVED` のため未成立である。Phase 1〜5 の verification slice と Final Full Verification は、各開始条件が未成立のため未着手であり、Phase 0 の実行項目として扱わない。
