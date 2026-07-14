# MPC 疎結合化 Task List

- 作成日: 2026-07-14
- 状態: Planning / Phase 0 未完了
- 原則: 前 Phase の Definition of Done を満たすまで次へ進まない

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
- [ ] 計画レビューで scope、順序、Baseline v1 の承認者を確定する

## 全 Phase 共通ゲート

Phase 1〜5 は、それぞれ次を満たす。

- [ ] `baseline.md` の contract oracle と endpoint/type/direction/Domain/owner が exact match する
- [ ] current QoS compatibility oracle と active endpoint 数に未説明の差がない
- [ ] `/control/command/control_cmd` の publisher owner が一つである
- [ ] external-contract RED と hard-safety RED が 0 件である
- [ ] config/feature の enabled/disabled が意図せず変わっていない
- [ ] Phase 指定の deterministic fixture と live scenario が green である
- [ ] `make autoware-build` と対象 package test が成功する
- [ ] `docs/spec/mpc-integration.md` の該当責務・構造・検証結果を同じ Phase で更新する
- [ ] node/package tree を変えない Phase では `docs/spec/architecture.md` を変更しない
- [ ] 外部契約差分が出た場合は実装を停止し、本計画外の interface migration として扱う

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
- [ ] D-02/D-03/D-04 の判断を `docs/spec/mpc-integration.md` に反映し、外部契約は変更しない

### 1.4 Hard-safety oracle

- [ ] H-01〜H-08 の signal、単位、閾値、根拠、観測方法を走行取得前に確定する
- [ ] final steering angle/slew の authoritative limit を確認し、raw config 値で代用しない
- [ ] stale response の開始点、safe source、許容 control interval を定義する
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
- [ ] QP `P/A/q/l/u` と sparse structure fixture を作る
- [ ] solver result/failure sequence fixture を作る
- [ ] behavior sequence fixture を作る
- [ ] raw/postprocessed/final command fixture を作る
- [ ] arbitration fixture を canonical runtime、deterministic synthetic、dormant pure-rule に分ける
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

### 1.7 Phase 0b — 意図的修正（必要な場合のみ）

- [ ] 各修正を構造リファクタと別 commit/PR にする
- [ ] 修正を「既存 interface contract への適合」または「外部契約を変えない安全修正」に限定する
- [ ] topic/type/Domain/control method/result schema 自体の変更を本 Phase に入れない
- [ ] 変更理由、安全/評価影響、before/after、test を記録する
- [ ] 修正後に該当 scenario と回帰 matrix を再実行する
- [ ] Baseline candidate v0 から v1 への intentional delta を保存する

### Phase 0 DoD

- [ ] static manifest、runtime evidence、fixture、tolerance が揃っている
- [ ] D-01〜D-11 が許可された分類と根拠を持つ
- [ ] external-contract RED と hard-safety RED が 0 件である
- [ ] H-01〜H-08 に FAIL/UNRESOLVED がない
- [ ] 機能性能上の waiver は owner/issue/期限/scenario/承認者を持つ
- [ ] arbitration は canonical runtime / deterministic synthetic / dormant pure-rule の各欄が PASS または正当な N/A である
- [ ] B-01/B-02/B-07/B-08/B-09 は PASS である
- [ ] B-03〜B-06 は PASS、または非 safety/contract の承認済み KNOWN_RED である
- [ ] N/A は文書で指定した optional cell だけに使われ、実行不能は BLOCKED である
- [ ] `LegacyReplayHarness` の必須 fixture が 2 回以上同じ正規化 output を返す
- [ ] 最終 commit、source/config/resource、comparison schema、binary、dev/eval image、submit tar の identity を再取得している
- [ ] Baseline v1 がレビュー・承認されている
- [ ] Phase 1 開始時点で未説明の baseline mismatch がない

## 2. Phase 1 — QP Solver 分離

- [ ] `MpcProblem` / `MpcSolveResult` の最小値型を導入する
- [ ] 既存 `solve_osqp()` を `QpSolver` に mechanical move する
- [ ] OSQP settings、accepted status、validation を現行のまま移す
- [ ] behavior/recovery/failure counter の side effect を caller 側に残す
- [ ] feasible case の unit test を追加する
- [ ] infeasible/non-finite/constraint violation の unit test を追加する
- [ ] status/solution/max violation を baseline fixture と比較する
- [ ] QP input の structure は exact、numeric は Phase 0 tolerance 内であることを確認する
- [ ] ROS/launch/config/topic に差分がないことを確認する
- [ ] `make autoware-build` を実行する
- [ ] 対象 package test を実行する
- [ ] deterministic QP/fault fixture と B-02 smoke を実行する
- [ ] `docs/spec/mpc-integration.md` の solver 境界を更新する

### Phase 1 DoD

- [ ] `QpSolver` は ROS、V2X、path、Domain、暗黙時刻に依存しない
- [ ] solver extraction 前後の behavior/fallback/failure sequence が一致する
- [ ] Phase 0 comparison が green である

## 3. Phase 2A — Base path read-only 化

- [ ] base path と dynamic per-cycle data の現行 write site を列挙する
- [ ] `BasePathStore` / immutable snapshot API を導入する
- [ ] public mutable vector 参照を read API に置換する
- [ ] CSV/topic、circular/open の fixture を追加する
- [ ] 補間、平滑化、曲率、base speed、左右幅を比較する
- [ ] canonical CSV schema/header/行数を維持する
- [ ] trajectory producer を維持する
- [ ] build/package test/characterization を実行する
- [ ] B-02 と B-09 を実行する
- [ ] `docs/spec/mpc-integration.md` の path ownership を更新する

### Phase 2A DoD

- [ ] consumer が base path vector を直接変更しない
- [ ] base path の全観測値が baseline と一致する

## 4. Phase 2B — BehaviorDecision 分離

- [ ] 現行 V2X/FSM/gap/overtake の全 input/state/output を列挙する
- [ ] ROS message から `V2xSnapshot` への adapter を作る
- [ ] `BehaviorDecision` 値型を追加する
- [ ] ego projection、read-only path/corridor、V2X、時刻、previous feedback を持つ `BehaviorInput` を追加する
- [ ] `RaceBehaviorPlanner` に現行判断を mechanical extraction する
- [ ] 時刻、vehicle/domain、solver feedback を explicit input にする
- [ ] Cruise/Follow/Overtake/LowSpeedAvoidance/SafetyBrake sequence test を追加する
- [ ] stale V2X、missing/duplicate ID、curve forbidden、no-gap test を追加する
- [ ] 閾値、状態遷移、戦術が増減していないことを確認する
- [ ] build/package test/characterization を実行する
- [ ] V2X deterministic fixture と B-03〜B-06 を実行する
- [ ] `make dev4` で V2X 接続、Domain 4 fallback、状態列を確認する
- [ ] `docs/spec/mpc-integration.md` の behavior input/output を更新する

### Phase 2B DoD

- [ ] behavior planner は ROS、publisher、logger、env、QP を参照しない
- [ ] state/category、target speed/offset、corridor、target ID、pass side、phase、SafetyBrake request が baseline と一致する

## 5. Phase 2C — LocalReferenceBuilder 分離

- [ ] Domain/start window/ref_vel section の現行速度上限適用順を fixture 化する
- [ ] `OperationalLimits` と `OperationalLimitResolver` を抽出する
- [ ] target speed/lateral/corridor の現行適用順を fixture 化する
- [ ] `ReferenceHorizon` 値型と validation を追加する
- [ ] `LocalReferenceBuilder` に周期 reference 生成を抽出する
- [ ] base path への周期速度/constraint mutation を除去する
- [ ] sample 数、単位、frame、finite、左右境界 invariant test を追加する
- [ ] 単車/V2X/追い越し/制動 horizon を baseline と比較する
- [ ] build/package test/characterization を実行する
- [ ] 影響する B-02〜B-06 を実行する
- [ ] `docs/spec/mpc-integration.md` の operational limit/reference 境界を更新する

### Phase 2C DoD

- [ ] base path snapshot は周期の前後で不変である
- [ ] horizon 全 field が tolerance 内で一致する

## 6. Phase 2D — MpcProblemBuilder 分離

- [ ] `init_problem()` の QP 構築 input を列挙する
- [ ] `MpcProblemBuilder` に matrix/vector 構築を抽出する
- [ ] V2X/FSM/lap/Domain/env/time 依存を builder から除く
- [ ] behavior と solver の failure reason を分離する
- [ ] `P/A/q/l/u` の structure と numeric を baseline と比較する
- [ ] solver failure 後の状態列を baseline と比較する
- [ ] build/package test/characterization を実行する
- [ ] 影響する B-02〜B-06 と deterministic QP fixture を実行する
- [ ] `docs/spec/mpc-integration.md` の problem builder 境界を更新する

### Phase 2D DoD

- [ ] 同じ prepared reference から同じ QP が作られる
- [ ] builder に previous solver result、behavior side effect、solver invocation がない

## 7. Phase 3 — One-cycle API

- [ ] `CycleInput`、`CycleOutput`、`CycleState` を定義する
- [ ] `awsim_vehicle_state`、`local_controller_enabled`、`local_stop_requested` を別 field にする
- [ ] `/awsim/control_mode_request_topic` と内部 `/control/control_mode_request_topic` を remap/統合しない
- [ ] callback data を周期先頭で immutable snapshot にする
- [ ] dynamic parameter を次周期の config snapshot として反映する
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
- [ ] Phase 0 comparison と timing gate が green である

## 8. Phase 4 — CommandArbiter / SafetySupervisor

- [ ] 現行 arbitration 優先順位表を `design.md` に確定値で反映する
- [ ] `ControlCandidate` / `ArbitrationDecision` 値型を追加する
- [ ] acceleration/steering postprocess を `ControlPostProcessor` に抽出する
- [ ] Recovery/Boost/gear を pure policy として抽出する
- [ ] 最終 arbitration と output validation を一経路にまとめる
- [ ] stale odom vs nominal test を追加する
- [ ] solver fallback vs Overtake test を追加する
- [ ] disabled/stop vs Boost test を追加する
- [ ] Recovery vs nominal の deterministic/dormant pure-rule test を追加する
- [ ] stale gear vs Reverse の deterministic/dormant pure-rule test を追加する
- [ ] non-finite postprocess test を追加する
- [ ] shutdown while Reverse test を追加する
- [ ] steering gain 適用後の angle/rate invariant を確認する
- [ ] fault/forced stop/Recovery 中の Boost inhibit を確認する
- [ ] Boost edge/start-once/re-arm sequence を比較する
- [ ] `/control/command/control_cmd` publisher owner が一つであることを確認する
- [ ] canonical disabled の Recovery/gear runtime 欄は N/A とし、非canonical test-only simulation を実走等価性に使わない
- [ ] build/package test、B-02〜B-07 を実行する
- [ ] `make dev4` で final publisher ownership と Domain 分離を確認する
- [ ] `docs/spec/mpc-integration.md` の arbitration/safety ownership を更新する

### Phase 4 DoD

- [ ] core/policy は ROS publish しない
- [ ] 最終 command/gear/boost action と reason が一つの結果に集約されている
- [ ] safety invariant と baseline arbitration sequence が全て green である

## 9. Phase 5 — Config/tools/package 整理

- [ ] flat YAML key と Domain override の互換 matrix を作る
- [ ] compatibility loader と typed internal config を分離する
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
- [ ] final candidate で B-01〜B-09 の全 matrix を再実行する

### Phase 5 DoD

- [ ] launch/config/topic/tool/submission の互換性が確認済みである
- [ ] `aichallenge_system/` に participant 実装を移していない
- [ ] 全体 DoD と文書更新が完了している

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

本 planning 作業ではコード、launch、config を変更していない。Docker/ROS/AWSIM の起動、build、package test、gate、V2X、eval はまだ実行しておらず、すべて Phase 0 の未完了項目である。
