# MPC 疎結合化 設計

- 作成日: 2026-07-14
- 状態: Draft
- 前提: `requirements.md` と `baseline.md` の Baseline v1 が Phase 1 開始条件

## 1. 現状認識

現行 `mpc_controller_cpp.cpp` は約 9,000 行の単一 translation unit で、ROS I/O、設定解決、base path、V2X、behavior FSM、local reference、QP 構築、OSQP、制御後処理、Recovery、gear、Boost、fail-safe、publish が同じ object graph と mutable state を共有している。

特に次の境界が曖昧である。

- `MPC::init_problem()` が behavior 判断、reference 生成、QP 構築を同時に行う。
- solver failure が behavior/recovery state を直接変更する。
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

## 3. 目標 dependency

```text
ROS callbacks / flat YAML / environment
                  |
                  v
        MpcControllerCpp (compatibility facade)
        - RosInputAdapter / ConfigCompatibilityLoader
        - SessionState / V2xSnapshotBuilder / BasePathStore
                  |
                  v
       CycleInput + BasePathView + V2xSnapshot
                  |                         |
                  +-> BehaviorInput --------+
                              |
                              v
                    RaceBehaviorPlanner -> BehaviorDecision
                                                   |
       Session/config -> OperationalLimitResolver -> OperationalLimits
                                                   |
       BasePathView --------------------------------+
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
                              |                           |
                              +-> PreviousCycleFeedback   +-> ControlPostProcessor
                                  (next cycle only)                    |
                                                                       v
                                                              ControlCandidate
                                            |
                                            v
                      CommandArbiter / SafetySupervisor
                                            |
                                            v
                                  FinalCommand + reason
                                            |
                                            v
                        RosOutputAdapter (sole publisher)
```

`RecoveryPolicy`、`BoostPolicy`、`GearPolicy` は `CommandArbiter / SafetySupervisor` が利用する pure policy とし、独立 publisher にはしない。

## 4. Component responsibilities

### 4.1 `MpcControllerCpp`

- 既存 node/executable/topic/parameter/launch の互換 façade
- callback data を周期境界で snapshot 化
- flat YAML と dynamic parameter を typed config に変換
- 40 Hz timer と `SingleThreadedExecutor` を維持
- diagnostics/debug topic と最終 command の ROS publish
- core に ROS message を渡さず、core の値型を ROS message に変換
- 公式 `/awsim/control_mode_request_topic` の新規 pub/sub を追加せず、legacy 内部 enable input と別責務のまま扱う

### 4.2 `BasePathStore`

- CSV/topic 由来の base path、補間、平滑化、曲率、base velocity、track width を保持
- base path は load/update 単位で immutable snapshot とする
- 周期ごとの速度上書き、V2X corridor、overtake offset を保持しない
- 現行 CSV schema と circular/open semantics を維持

### 4.3 `V2xSnapshotBuilder`

- `v2x_msgs` を core 用の有限値・時刻付き snapshot に変換
- vehicle ID、freshness、source stamp、観測順を明示
- ID 欠落や重複を隠さず validation result として残す
- gap/behavior 戦術は担当しない

### 4.4 `RaceBehaviorPlanner`

- Cruise、Follow、Overtake、LowSpeedAvoidance、SafetyBrake の現在の状態遷移
- target vehicle、pass side、target speed/offset、corridor narrowing の意図を決定
- V2X ROS message、publisher、Domain env、QP matrix を知らない
- ego projection、read-only path/corridor、V2X snapshot、時刻、previous FSM/solver feedback を `BehaviorInput` で受ける

### 4.5 `OperationalLimitResolver`

- Domain 別 `v_max`、Start 後の時間窓、`ref_vel.yaml` section、base speed 上限を現在の順序で解決
- `BehaviorDecision` の追従/制動速度とは別の `OperationalLimits` を返す
- ROS parameter、環境変数、base path vector を直接変更しない

### 4.6 `LocalReferenceBuilder`

- base path と `BehaviorDecision` から、その周期だけの horizon を作る
- target velocity、lateral target、左右 corridor、dynamic constraint をサンプルごとに解決
- base path を mutation しない
- behavior state machine と solver を知らない

### 4.7 `MpcProblemBuilder`

- vehicle state、model/config、完成済み `ReferenceHorizon` から `P/A/q/l/u` を作る
- V2X、lap、vehicle ID、overtake phase、ROS time を知らない
- solver を呼ばず、sparse problem と metadata を返す

### 4.8 `QpSolver`

- `MpcProblem` を受け、OSQP の解、status、iteration、constraint violation を返す
- accepted status、settings、validation は現行 semantics を維持
- behavior state や failure counter を直接変更しない

### 4.9 `ControlPostProcessor`

- solver solution から nominal acceleration/steering candidate を作る
- acceleration conversion/filter、steering gain/filter/limit の現行順序を担当
- publish、Boost、Recovery、gear の判断はしない

raw command と最終 command を別の観測点として維持する。steering gain/filter/limit の適用順は Phase 0 trace から固定し、raw 側の limit を final 側へ機械的に流用しない。

### 4.10 `CommandArbiter / SafetySupervisor`

- control disabled、stop request、invalid/stale input、solver fallback、Recovery、Boost、gear を最終調停
- 最終 command の finite/range/rate invariant を確認
- command、gear/boost action、reason、diagnostics を一つの結果として返す
- ROS publish はしない

### 4.11 Test-only `LegacyReplayHarness`

- production node の private state snapshot を保存しない
- constructor/reset の clean state から、明示時刻付き input/event prefix を順番に replay する
- callback 順、timer step、parameter update、V2X/session event を fixture schema で表現する
- legacy 1-cycle seam から behavior/reference/QP/solver/raw/final output を収集する
- 同じ fixture の反復で正規化 output が決定的であることを確認する
- production launch/runtime path には組み込まない

## 5. Value objects

### 5.1 `CycleInput`

- ROS time と monotonic time
- vehicle state と freshness
- immutable base path/config snapshot
- V2X snapshot
- `awsim_vehicle_state`
- `local_controller_enabled`
- `local_stop_requested`
- session state
- previous solver/arbitration feedback
- Domain/vehicle の解決済み context

周期途中の parameter callback はこの object を変更せず、次の周期で新しい config snapshot として反映する。

`local_controller_enabled` は現行 legacy `/control/control_mode_request_topic` 由来、AWSIM engage は公式 `/awsim/control_mode_request_topic` 由来で責務が異なる。二つを remap、統合、相互代用しない。

### 5.2 `BehaviorInput`

- ego state と base path 上の projection/progress
- read-only path geometry、curvature、corridor view
- V2X snapshot
- ROS/monotonic time
- previous FSM/solver feedback

### 5.3 `BehaviorDecision`

- state と reason
- target vehicle ID、pass side、overtake phase
- desired speed と speed limit
- target lateral offset
- corridor policy/override
- behavior 上の SafetyBrake/速度制限 request
- diagnostics

stale odometry、local stop request、invalid solver output など system-level ForcedStop は `BehaviorDecision` に入れず、`CommandArbiter / SafetySupervisor` が判断する。

### 5.4 `OperationalLimits`

- Domain/session/section 解決後の速度上限
- 適用理由と有効期間
- 必要な acceleration/lateral limits

behavior 由来の target speed と設定/運用由来の上限を別 field に保ち、`LocalReferenceBuilder` で現行の順序どおり合成する。

### 5.5 `ReferenceHorizon`

各 sample に、少なくとも次を同じ index で持たせる。

- `s_m`, `x_m`, `y_m`, `yaw_rad`, `curvature_radpm`
- `speed_mps`
- `lateral_lower_m`, `lateral_upper_m`, `target_lateral_m`

全 field の sample 数一致、有限値、frame、単位、`lower <= upper` を生成時に検証する。`target` が corridor 内であることを必須 invariant にするかは Phase 0 の現行値から判断し、リファクタ時に新しい clamp を暗黙追加しない。現行 builder が必要とする sample 数も Phase 0 fixture から固定する。

### 5.6 `MpcProblem`

- sparse `P`, `A`
- `q`, `l`, `u`
- dimensions、horizon/model metadata

構造比較と数値比較を分けられる表現にする。

### 5.7 `MpcSolveResult`

- normalized status と raw OSQP status
- solution/control sequence
- iteration、objective、最大制約違反
- validity と failure reason

### 5.8 `ControlCandidate` / `ArbitrationDecision`

- raw/filtered control と validity
- source (`NominalMpc`, `Fallback`, `Recovery`, `SafeStop`)
- Boost/gear action
- final command
- arbitration reason と inhibited reason

## 6. State and feedback

すべてを stateless にする必要はない。FSM、filter、failure counter、Boost start-once、Recovery phase は stateful である。ただし、所有者を一つにし、周期の先頭と末尾で state transition を明示する。

```text
previous CycleState + CycleInput
  -> decisions/results
  -> next CycleState + CycleOutput
```

現行の solver failure -> overtake recovery 変更は、同周期か次周期かを Phase 0 trace で確定する。分離後は `MpcSolveResult` または `PreviousCycleFeedback` を介し、同じ周期 semantics を維持する。

## 7. Current arbitration characterization

Phase 4 で優先順位を再設計してはならない。Phase 0 で現行 `control()` と publish 前後を調べ、canonical runtime、deterministic synthetic fixture、dormant pure-rule test を使い分けて次の競合を固定する。

| 競合 | evidence class | 固定する観測 |
|---|---|---|
| stale odometry vs nominal MPC | deterministic fault + live proxy | 最終 source、brake/steer、reason category |
| solver failure vs Overtake | deterministic cycle | fallback、FSM feedback、failure count の順序 |
| control disabled/stop vs Boost | deterministic cycle | command と Boost inhibit/event 順序 |
| Recovery vs normal MPC | dormant pure-rule（canonical runtime は N/A） | takeover 条件、gear、最終 source |
| stale gear report vs Reverse | dormant pure-rule（canonical runtime は N/A） | Reverse 許可/拒否と safe fallback |
| non-finite postprocess | deterministic fault | 検出位置と最終 safe command |
| shutdown while Reverse | dormant pure-rule（canonical runtime は N/A） | gear/command 終了 sequence |

canonical config で Recovery/gear は disabled である。これを有効化した test-only simulation は補助 evidence にできるが、canonical runtime 等価性は主張しない。この表が期待値または正当な N/A で埋まるまでは、`CommandArbiter` の実装に着手しない。

## 8. Migration phases

### Phase 0: Baseline v1

制御ロジック、責務境界、外部契約を変更しない。Phase 0a で既存 topic/log だけの black-box evidence を取得し、続く observation step で baseline recorder、比較器、test-only/no-op trace seam と `LegacyReplayHarness` を追加する。canonical production では seam を無効にする。

bootstrap は次の 3 比較に分ける。

1. original v0 と seam compiled/disabled: contract exact、live envelope 非悪化
2. seam disabled と enabled: deterministic fixture の decision/output 一致
3. seam enabled の serialization/I/O overhead: 記録のみ。production performance gate には使わない

- static/runtime manifest、fixture、比較器、tolerance を作る
- config/document/contract drift を分類する
- 確認された既存契約への不適合、または外部契約を変えない安全問題だけを `Phase 0b` の独立修正にする
- observation seam/Phase 0b を含む最終 clean commit から全 evidence と identity を再取得する
- external-contract RED と hard-safety RED が 0 の reference を Baseline v1 として承認する

Gate: `baseline.md` の Freeze checklist が完了していること。

### Phase 1: `QpSolver` extraction

既存 `solve_osqp()` を matrix-in/result-out の component に移す。

変更しないもの:

- QP の作り方と行列値
- OSQP settings と accepted status
- constraint validation
- fallback と failure count
- behavior/recovery の現行反応

Gate:

- QP structure は exact、numeric input は baseline tolerance 内
- feasible/infeasible/non-finite/constraint violation test
- status、solution、最大違反が tolerance 内
- build/package test/characterization replay が成功

### Phase 2A: `BasePathStore` read-only boundary

- public mutable vector への直接書込みを adapter の内側へ閉じる
- immutable base path と 1 周期の dynamic view を分ける
- CSV/topic/circular/open の現行 semantics を fixture 化する

Gate: 点列、曲率、base speed、幅、補間結果が baseline と一致し、consumer が base vector を直接変更しない。

### Phase 2B: `BehaviorDecision` extraction

- `init_problem()` から V2X behavior、gap、Follow/Overtake/SafetyBrake、overtake line の判断を抽出
- 最初は現在の結果をそのまま表す値型にし、戦術を改善しない
- ego projection、read-only path/corridor、explicit time/V2X/previous solver feedback を `BehaviorInput` にする

Gate: Cruise/Follow/Overtake/LowSpeedAvoidance/SafetyBrake の状態列と境界条件が baseline と一致する。

### Phase 2C: `LocalReferenceBuilder` extraction

- Domain/Start window/section の速度解決を `OperationalLimitResolver` に抽出
- target velocity/lateral offset/corridor/dynamic constraint を base path から分離
- base path の周期 mutation を廃止

Gate: horizon の全 field が baseline と一致し、base path snapshot が周期後も不変である。

### Phase 2D: `MpcProblemBuilder` extraction

- 完成済み reference と model/config だけから QP を生成
- V2X/FSM/lap/Domain 依存を除く
- previous solver result を入力にせず、solver feedback は `MpcSolveResult -> PreviousCycleFeedback -> RaceBehaviorPlanner` の別経路に保つ

Gate: `P/A/q/l/u`、dimensions、solver/behavior の周期 sequence が baseline と一致する。

### Phase 3: One-cycle API

- `update_current_speed`、`update_v_max`、path mutation、`get_control` の順序依存を `CycleInput -> CycleOutput` にまとめる
- dynamic config を周期単位の immutable snapshot にする
- ROS adapter と core state の境界を固定する
- `awsim_vehicle_state`、legacy `local_controller_enabled`、`local_stop_requested` を別 field にし、二つの control-mode topic を統合しない

Gate: cycle replay の command、prediction、behavior、solver state が tolerance 内。40 Hz、SingleThreadedExecutor、外部 I/O は不変。

### Phase 4: `CommandArbiter / SafetySupervisor`

- nominal/fallback/Recovery/SafeStop、後処理、gear、Boost の最終経路を一つにする
- Phase 0 で確定した現行優先順位を再現する
- 最終 finite/angle/rate/acceleration invariant を検証する

Gate:

- `/control/command/control_cmd` owner は常に一つ
- 競合 test がすべて成功
- gate scenarios と `make dev4` で final invariant、許可 event sequence、publisher ownership が baseline envelope/contract と一致
- disabled Recovery/gear は dormant pure-rule test が成功し、canonical runtime は N/A のまま

gain 適用後の limit など Phase 0 で安全問題と判断されたものは、ここで紛れて直さず Phase 0b の意図的修正として先に baseline 化する。

### Phase 5: Config/tools/package boundary

runtime 等価性確立後にのみ行う。

- flat YAML compatibility loader と typed internal config の責務を整理
- pure library の CMake target を整理
- offline trajectory tool と runtime dependency を分ける
- package 分割が必要なら `aichallenge_submit/` 内で行い、互換 entry を残す
- `mpc-integration.md` と必要な architecture 文書を更新

Gate: build/test、提出 tar、eval image、`make eval`、既存 CLI/config/launch 互換が成功。

legacy 削除、ROS node 分割、制御性能改善は Phase 5 完了後の別計画とする。

## 9. Test strategy

### 9.1 Pure unit tests

- value object validation と単位/サイズ invariant
- solver status/validation
- V2X freshness/ID validation
- behavior state sequence
- local reference boundaries
- arbitration conflicts、Boost/gear/Recovery rules

### 9.2 Characterization replay

- legacy/current implementation が吐いた normalized cycle fixture を入力
- `LegacyReplayHarness` は clean reset から input/event prefix を再生し、private state snapshot に依存しない
- 同じ fixture を現行実装で 2 回以上 replay し、正規化 output の決定性を先に確認する
- exact field と tolerant numeric field を分離
- test-only legacy oracle との shadow comparison は移行中だけ許可
- production binary に長期 dual path や runtime switch を残さない
- 走行 trace は wall-clock ではなく path progress/waypoint/event を基準に整列する
- 連続量は同一 image/host の反復から中央値と MAD を取り、hard safety limit と baseline envelope の両方で判定する

### 9.3 ROS integration

- node/param/topic/QoS/sole publisher
- Domain 1 と Domain 1〜4
- 40 Hz と timing distribution
- launch route と canonical config 解決

### 9.4 Evaluation ladder

```text
unit + characterization
  -> make autoware-build
  -> package test
  -> make dev
  -> affected gate1/2/3
  -> make dev4 (V2X/Domain-sensitive Phase)
  -> submission/eval checks (final Phase)
```

## 10. Change and rollback policy

- 一つの Phase を独立 commit/PR にする。
- Phase 内でも mechanical move、API 導入、caller 切替、旧コード削除を review 可能な commit に分ける。
- baseline mismatch が出た場合、tolerance を先に広げず、最初に入力、state transition、unit/frame、呼出順を調べる。
- Phase gate を通るまで旧 production path を削除しない。切替後は旧 production path を同じ Phase 内で削除する。
- rollback はその Phase の commit revert で前の green baseline に戻せる粒度を保つ。
