# AGENTS.md

This file provides guidance to Codex when working in this repository.

このリポジトリは Automotive AI Challenge 2026 向けベースリポジトリを作るための作業場です。現時点のコードは Automotive AI Challenge 2025 / Racing Kart 向けの Autoware + AWSIM 開発環境をベースにしているため、既存の ROS 2 パッケージ、launch、トピック契約、評価ログ、提出 tar.gz の流れを壊さずに 2026 向けへ整理していきます。

Kaggle 型の `expXXX`、CV/LB、`uv`、`pipeline.sh` 前提では扱わないでください。2026 公式仕様が未確認の事項は、2025 由来の仮置き仕様として明記し、確定仕様のように扱わないでください。

## 基本姿勢

- 目的は「2026 向けに再利用できるベースを作ること」。単発の 2025 参加者実装に閉じた最適化より、移植性・保守性・検証しやすさを優先する。
- 安全性、評価再現性、インターフェース互換性を最優先する。
- まず既存の launch、param yaml、ROS 2 topic、Docker/Makefile の流れを読む。
- 参加者実装は原則 `aichallenge/workspace/src/aichallenge_submit/` に閉じる。
- 評価基盤である `aichallenge/workspace/src/aichallenge_system/` は、目的と互換性影響を明確にした場合だけ変更する。
- 実車系の `vehicle/`、`remote/` はシミュレータで確認した後に扱う。速度、操舵、制動、通信断のリスクを常に意識する。
- ホストに ROS 2 環境がある前提で作業しない。原則として Docker Compose / Makefile 経由を正とする。

## 2026 ベース化方針

- 2025 由来の実装と 2026 向けに新しく決めた方針をドキュメント上で分ける。
- 2026 公式仕様が確認できたら、差分を `docs/spec/` または `docs/interface/` に反映してから実装を変える。
- まだ未確定の仕様は `TBD` または「2025 由来の暫定」と明記する。
- チーム固有・環境固有の設定をコードや Dockerfile に直書きしない。`.env`、param yaml、launch 引数、README に逃がす。
- 評価基盤、提出物、学習用 workspace、実車接続を混ぜない。境界を保つ。
- まず `make autoware-build`、`make dev`、`make eval` が通る最小ベースを維持する。
- 2026 向けに名称変更する場合も、一度に広範囲を置換せず、互換性と起動確認を刻んで進める。

## 正本ドキュメント

| ファイル | 内容 |
|---|---|
| `README.md` | セットアップ、全体概要、主要コマンド |
| `docs/README.md` | ドキュメント分類と運用方針 |
| `docs/spec/architecture.md` | リポジトリ構成、Compose、ROS 2 Domain、launch 階層 |
| `docs/spec/competition-rules.md` | 2026 SW 部門ルールの公式ページ要約とリポジトリへの影響 |
| `docs/spec/how-to-setup.md` | Ubuntu 22.04 想定の環境構築 |
| `docs/spec/open-questions.md` | 2026 公式仕様と現行実装の差分、運営確認事項 |
| `docs/spec/safety-gates.md` | 障害物停止、NPC 追い越し、車線維持の検証方針 |
| `docs/spec/submission-workflow.md` | 提出物作成、ローカル評価、公式提出前チェック |
| `docs/spec/v2x-multivehicle.md` | V2X と 3〜4 台同時走行の方針 |
| `docs/spec/compose-overlays.md` | `.env` の `COMPOSE_FILE`、GPU/CPU/headless 構成 |
| `docs/spec/log-design.md` | `output/` と `output/latest/` のログ・成果物設計 |
| `docs/spec/mpc-integration.md` | MPC 制御器の統合仕様 |
| `docs/interface/participant-interface.md` | 参加者が守る提出物・topic・service 契約 |
| `docs/interface/evaluation-interface.md` | 評価基盤が守る FSM、Domain、result JSON、成果物契約 |
| `aichallenge/README.md` | 評価・ビルド・起動スクリプトの設計メモ |

大会ルールの正本は Automotive AI Challenge 2026 公式ドキュメントの SW 部門ルールページ（`https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html`）。公式ページは WIP で今後更新される可能性があるため、変更が疑われる場合は必ず再確認する。確認した仕様は `docs/spec/` または `docs/interface/` に反映する。2026 仕様として確証がない内容は、2025 由来の現行仕様または暫定仕様として記録する。

## 変更してよい領域

- `aichallenge/workspace/src/aichallenge_submit/`
  - 参加者提出物。制御器、自己位置推定、経路生成、launch、param yaml、モデル重みなどの主戦場。
- `aichallenge/ml_workspace/`
  - 学習・データ抽出用。実行時の ROS 2 package と混同しない。
- `docs/spec/`
  - 現仕様の永続ドキュメント。古くなった仕様は更新する。
- `docs/interface/`
  - 破ると依存側が壊れる契約。変更時は互換性影響を必ず書く。
- `docs/guide/`
  - 初学者向けガイドや発表資料。

## 注意して扱う領域

- `aichallenge/workspace/src/aichallenge_system/`
  - 評価 FSM、result JSON、録画、V2X、AWSIM 管理を含む。変更は評価基盤への影響が大きい。
- `Makefile`
  - Compose、Domain、ログ出力、評価手順の入口。ターゲット名や Domain 既定値の変更は慎重に扱う。
- `Dockerfile`、`docker-compose*.yml`
  - dev/eval イメージ、GPU/サウンド、host network、UID/GID を管理する。
- `vehicle/`、`remote/`
  - 実車・遠隔接続。シミュレータと違い物理安全リスクがある。
- `.env`
  - `./setup.bash env` が生成するローカル設定。通常はコミットしない。

## 触らない生成物

以下は原則として編集・コミットしない。

- `output/`
- `outputs/`
- `aichallenge/build/`
- `aichallenge/install/`
- `aichallenge/log/`
- `submit/*.tar.gz`
- `rosbag2*/`
- `*.mcap`
- `*.db3`
- `.env`

## 憲法: 現行互換として破ってはいけない契約

以下は 2025 由来の現行実装が依存している契約です。2026 公式仕様で変更が必要になった場合は、先に `docs/interface/` の契約を更新し、影響範囲と移行方針を書いてから実装を変えてください。

1. Domain 0 は AWSIM と `awsim_state_manager_node` 専用。車両 Autoware を Domain 0 に同居させない。
2. 車両は Domain 1..N。多車両は `make dev2` / `make dev3` / `make dev4` の `docker compose -p N` 分離に従う。
3. クロスドメイン通信は `/v2x/vehicle_positions` と `v2x_msgs` を使う。`domain_bridge` は復活させない。
4. `/admin/awsim/start`、`/admin/awsim/reset`、`/admin/awsim/state` の名前・型・責務を変えない。
5. `/awsim/state` と `/awsim/control_mode_request_topic` の名前・型・状態文字列を変えない。
6. `autostart_orchestrator_node` に `/admin/awsim/start` の責務を持たせない。
7. `aichallenge_submit.launch.xml` を `aichallenge_submit_launch` パッケージから削除しない。
8. `control_method` は `mpc`、`pure_pursuit`、`tiny_lidar_net`、`pilot_net`、`joycon` の範囲で扱う。
9. `/control/command/control_cmd` は最終制御出力として維持する。
10. `/localization/kinematic_state` と `/planning/scenario_planning/trajectory` の連結を壊さない。
11. `/set_initial_pose` service を評価起動ハンドシェイクとして維持する。
12. 提出 tar.gz の最上位ディレクトリは必ず `aichallenge_submit/` にする。
13. `result-summary.json`、`dN-result-details.json` のファイル名、schema version、主要キーを変えない。
14. `output/latest/` は最新成果物への導線。リンク名・配置を勝手に変えない。
15. `HOST_UID` / `HOST_GID` による成果物所有者の設計を壊さない。

## 作業フロー

### 作業開始時

1. `git status --short` で既存変更を確認する。
2. 関連する正本ドキュメントと対象コードを読む。
3. 既存のユーザー変更を巻き戻さない。衝突する場合だけ確認する。
4. 大きめの作業では `.steering/YYYYMMDD-[作業タイトル]/` を作り、以下を置く。
   - `requirements.md`: 目的、変更範囲、制約
   - `design.md`: 方針、変更コンポーネント、影響範囲
   - `tasklist.md`: TODO、進捗、Definition of Done

`doc/experiment/expXXX.md` は使わない。Kaggle 由来の実験番号運用はこのリポジトリの正本ではない。短命の実装計画は `docs/plan/` に置けるが、ここは gitignore 対象。永続化すべき仕様は `docs/spec/` または `docs/interface/` に反映する。

### 実装時

- 既存の launch 構造、param yaml、package 境界に合わせる。
- ROS 2 topic 名、message 型、service 名の変更は互換性影響を先に確認する。
- C++ package は `CMakeLists.txt` と `package.xml` の依存を揃える。
- Python node は entry point、launch、param yaml、依存 package を揃える。
- 学習済み重みを差し替える場合は、読み込みパス、型、shape、実行時依存を確認する。
- 大きなログ、rosbag、build artifact をコミット対象に含めない。

### 完了時

- 変更に応じた最小限の検証を実行する。
- 実行したコマンドと結果を報告する。
- 実行できなかった検証は理由を明記する。
- 仕様や運用が変わった場合は `docs/spec/` または `docs/interface/` を更新する。

## 開発コマンド

### 初期診断・セットアップ

```bash
./setup.bash doctor
./setup.bash env
./setup.bash network tune
./setup.bash download awsim
./setup.bash pull image
./docker_build.sh dev
make autoware-build
```

一括セットアップが必要な場合:

```bash
./setup.bash bootstrap
```

### 開発起動

```bash
make dev
make dev2
make dev3
make dev4
make gate1
make gate2
make gate3
make ps
make down
```

### 走行開始・操作

```bash
make autoware-request-initialpose
make autoware-request-control
make awsim-request-start
make awsim-request-reset
make rviz2
```

### コンテナ内確認

```bash
make autoware-bash
make autoware-attach
```

コンテナ内でよく使う確認:

```bash
ros2 topic list
ros2 topic hz /control/command/control_cmd
ros2 topic echo --once /localization/kinematic_state
ros2 topic echo --once /planning/scenario_planning/trajectory
ros2 service list
```

### 評価・提出

```bash
./create_submit_file.bash
./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz
make eval
```

### 実車・遠隔

```bash
vehicle/setup_check.sh
make driver
make zenoh
make autoware-driver-zenoh
make autoware-driver-zenoh-rosbag
```

### 品質確認

```bash
pre-commit run -a
```

ROS 2 package 単位のテストが必要な場合は、コンテナ内で対象 package を絞って実行する。

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

## 検証基準

| 変更内容 | 最低限の確認 |
|---|---|
| ドキュメントのみ | リンクと正本ドキュメントとの整合 |
| launch / yaml / package 設定 | `make autoware-build` |
| C++ / Python ROS 2 node | `make autoware-build`、必要に応じて `colcon test` |
| 制御器・trajectory・localization | `make dev` または `make gate*`、topic hz、ログ確認 |
| 多車両・V2X | `make dev2` 以上、Domain と `/v2x/vehicle_positions` の確認 |
| 提出前 | `./create_submit_file.bash`、`./docker_build.sh eval --submit ...`、`make eval` |
| 実車関連 | シミュレータ確認後、`vehicle/setup_check.sh`、通信・停止手順確認 |

## ログ・成果物の見方

- 最新成果物の入口は `output/latest/`。
- Docker build log は `output/latest/docker_build.log`。
- 評価・走行ログは `output/latest/d<N>/autoware.log`。
- 結果は `output/latest/d<N>/result-summary.json` と `output/latest/d<N>/result-details.json`。
- rosbag は `output/latest/d<N>/rosbag2_autoware.mcap`。
- `result-summary.json` はレース全体、`dN-result-details.json` は車両ごとの詳細。

## 必要な専門観点

| 観点 | 役割 |
|---|---|
| Autoware / ROS 2 reviewer | package、launch、remap、topic、service、param のレビュー |
| ROS 2 error analyzer | colcon、launch、DDS、topic 未接続、runtime exception の解析 |
| Evaluation analyzer | lap time、finish、penalty、result JSON、評価ログの分析 |
| Rosbag analyzer | mcap から topic hz、欠損、制御出力、軌跡、自己位置を確認 |
| Controller tuner | MPC、Pure Pursuit、PilotNet、TinyLidarNet の調整 |
| Interface guardian | `docs/interface/` の契約違反を検出 |
| Vehicle integration | 実車、Zenoh、remote、RViz、driver の接続確認 |
| Documentation maintainer | `docs/spec/` と `docs/interface/` を最新に保つ |

## ローカルスキル

| スキル | 用途 |
|---|---|
| `code-reviewer` | Autoware / ROS 2 package、launch、制御器、評価互換性のレビュー |
| `error-analyzer` | colcon、docker compose、AWSIM、Autoware launch、DDS、評価失敗のログ解析 |
| `data-analyzer` | rosbag/mcap、走行ログ、topic hz、制御出力、trajectory、localization の分析 |
| `evaluation-analyzer` | `result-summary.json`、`dN-result-details.json`、lap、penalty、finish/timeout の解析 |
| `interface-guardian` | topic/service/message、Domain、提出 tar.gz、result JSON、`output/latest` の契約確認 |
| `web-summarizer` | 2026 公式情報や外部仕様を調べる必要がある場合の調査レポート作成 |

## レビュー方針

レビュー依頼では、所感よりも不具合・リスク・再現性・評価契約違反を先に出す。重大度順に、ファイルと行を添えて指摘する。問題が見つからない場合も、未検証のリスクと追加で見るべきログやシナリオを明記する。

## 禁止事項

- ユーザーの既存変更を勝手に revert しない。
- `git reset --hard` や広範囲削除を勝手に実行しない。
- 評価を通すために topic 契約や result schema を場当たり的に変えない。
- `output/` や rosbag を解析目的以外で編集しない。
- 実車向けの速度・操舵・制動変更をシミュレータ確認なしに進めない。
- Kaggle の CV/LB、`doc/experiment/expXXX.md`、`pipeline.sh expXXX` をこのリポジトリの標準ワークフローとして復活させない。
