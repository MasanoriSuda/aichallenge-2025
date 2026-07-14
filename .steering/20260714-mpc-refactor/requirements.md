# MPC 疎結合化 要求仕様

- 作成日: 2026-07-14
- 状態: Draft
- 対象: `aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`
- 入力資料: `point-out-by-chatgpt-pro.md`、現行コード、現行 launch/config、`docs/spec/`、`docs/interface/`

## 1. 目的

現行 MPC の外部契約と観測可能な挙動を先に固定し、その基準と比較しながら責務を段階的に分離する。

この作業で優先する順序は次のとおりとする。

1. 安全性と評価インターフェース互換性
2. 現行挙動の再現性
3. 変更差分の説明可能性
4. 内部構造の疎結合化
5. 将来の controller、経路生成、V2X 戦術の差し替えやすさ

「凍結」は、現行コードを無条件に正解として永久保存する意味ではない。まず実挙動を `Baseline candidate v0` として記録し、既知の不整合や安全上の疑義を分類する。必要な修正を独立差分として行った後に、リファクタの比較対象となる `Baseline v1` を確定する。

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

## 5. 機能要求

### R-01: Baseline manifest

commit、dirty state、Autoware/AWSIM launch route、launch 引数、設定/resource hash、Domain 別解決値、Docker image、実行 binary、ROS graph、topic type/QoS/publisher 数を記録できること。Domain 別 effective config は別 parser で再実装せず、production と同じ C++ resolver から出力する。

現行 interface compatibility の対象は Domain 1〜4 とする。4 台時の競技挙動が 2026 公式評価対象かどうかは未確定でも、Domain 分離、fallback 設定、topic/type/owner は確認する。

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

固定 fixture は test-only `LegacyReplayHarness` で再生する。private state の完全 snapshot は保存せず、constructor/reset の clean state から explicit ROS/steady timestamp を持つ input/event prefix を順番に適用して FSM、filter、V2X history を再構成する。同じ fixture を現行実装で 2 回以上 replay し、正規化 output が同じになることを Baseline v1 の条件とする。

### R-03: 比較可能な出力

少なくとも次を比較対象とする。

- behavior state、reason、対象車両、pass side、phase
- reference horizon の座標、姿勢、曲率、速度、左右境界、横位置目標
- QP の次元、疎行列構造、`P/A/q/l/u`
- solver status、解、最大制約違反、failure count
- raw command、後処理後 command、最終 command
- fail-safe、Recovery、Boost、gear の決定と理由
- prediction と主要 diagnostics

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

### R-06: 互換 façade

既存の `mpc_controller_cpp` executable と node、launch、flat YAML key を当面の互換 façade として維持し、内部の型付き component に変換する。

### R-07: 一つの最終出力経路

MPC、Recovery、SafeStop が別々に `/control/command/control_cmd` を publish してはならない。最終調停、有限値確認、limit 確認を通過した command だけを既存 ROS adapter が publish する。

### R-08: 1 周期 API

`update_*()` の呼出順で結果が変わる API を、明示的な `CycleInput -> CycleOutput` に置き換える。同じ input と同じ初期 state に対して同じ output を返すこと。

### R-09: 段階ゲート

各 Phase は独立して build/test/characterization comparison を通過し、前 Phase の DoD が満たされるまで次へ進まない。各 Phase は原則として独立 commit または PR とする。

### R-10: 文書同期

実装・運用が変わった Phase で `docs/spec/mpc-integration.md` を更新する。node/package tree が変わる場合は `docs/spec/architecture.md` も更新する。外部契約は原則変更しない。

## 6. 非機能要求

### 6.1 決定性と許容差

- enum、flag、配列サイズ、疎行列構造、publisher 数などは原則 exact match とする。
- contract oracle、deterministic cycle fixture、live-run envelope を別 schema/判定にする。
- 浮動小数値は baseline の同一条件を最低 3 回測定し、工学的下限、solver epsilon、quantization floor、観測 jitter を確認してから絶対/相対 tolerance を決める。
- timestamp、ログ順序、実行時間をそのまま golden にしない。比較前に正規化する。
- 理由のない緩い tolerance や、差分を隠すための丸めを禁止する。

### 6.2 制御周期

40 Hz の設定を維持する。実行時間 budget は未確認の公式値を作らず、Baseline v1 の測定分布から定める。平均値だけでなく p95/p99、deadline miss、solver iteration/status を記録する。

### 6.3 安全性

- stale/incomplete/non-finite input から通常 command を生成しない。
- forced stop、Recovery、fault 中の Boost 抑止を確認する。
- 最終 command で steering angle/rate、acceleration、gear 条件が成立することを確認する。
- 実車検証は本計画の対象外とし、シミュレータと評価検証の完了前に進めない。
- external-contract RED と hard-safety RED は baseline waiver の対象にせず、解消するまで Baseline v1 と次 Phase を承認しない。
- hard-safety oracle は signal、単位、閾値、根拠、観測方法、`PASS / FAIL / UNRESOLVED` を持つ。`UNRESOLVED` も Baseline v1 を block する。

### 6.4 成果物

rosbag/MCAP、build/install/log、`output/` はコミットしない。コミット対象は小さな manifest、比較器、fixture、test、文書に限定する。

## 7. 全体 Definition of Done

- Baseline v1 の静的 manifest、runtime evidence、fixture、tolerance がレビュー済みである。
- observation seam を含む最終 clean commit と binary/image/tar identity から baseline が再取得されている。
- 各 component の責務と input/output が `design.md` と一致する。
- 外部契約、launch entry、config key、単一 publisher ownership が維持されている。
- 全 Phase の unit/characterization/integration gate が成功している。
- `make autoware-build`、対象 package test、必要な gate、多車両確認が成功している。
- 最終段階で提出 tar の構造、eval image build、`make eval` を確認している。
- 実行できなかった検証と残存リスクが明記されている。
