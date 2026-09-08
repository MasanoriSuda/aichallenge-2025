# Repository instructions

Automotive AI Challenge 2026向けの再利用可能なベースを整備するリポジトリ。
2025 Racing Kart由来のAutoware / AWSIM環境を段階的に移行する。
安全性・評価再現性・インターフェース互換性を優先する。
2026公式仕様として未確認の事項は「2025由来の暫定」または`TBD`と記録する。

## 作業の進め方

- 開始時に`git status --short`を確認し、対象コードと関連する正本ドキュメントを読む。
  既存変更を巻き戻さず、衝突がある場合だけ確認する。
- 修正・実装依頼は、調査、必要な計測、実装、検証、文書更新までの依頼として扱う。
  既に承認された範囲での可逆的な作業は、毎段階の追加承認を求めず進める。
  監査・レビュー・計画のみの依頼では、その範囲を守る。
- 通常の実装判断は根拠と合理的な仮定で進める。欠けた情報が成果や安全性を変える場合は、
  質問しつつ独立して進められる作業を続ける。権限外の操作は既存の承認手順に従う。
- Skillsや過去steeringは今回の依頼・会話での承認を置き換えない。
  過去の「次は承認待ち」を現在の停止条件に引き継がない。
  ファイルの指示で停止する場合は、そのパス・該当文・適用理由を示す。
- 調査は対象symbol・差分・該当runから絞る。独立した検索・読み取りはまとめて実行する。
  subagentはユーザーまたは実行環境の指示で許可された場合だけ使う。
- 複数段階の作業は`.steering/YYYYMMDD-title/`へ目的・制約、設計判断、tasklistを残す。
  小変更に計画一式は不要。中断・再開時は同じ目的と残作業を引き継ぐ。
- 原因が分からない回帰は、失敗するtest/replayや観測で原因を絞ってから修正する。
  合格条件を緩めたり、警告を消しただけで修正済みにしない。
- 日本語で結論を先に簡潔に伝える。進捗は分かったことと次の確認、完了報告は
  変更・実行コマンドと結果・未検証事項を示す。実施していない検証を合格にしない。

## 対象領域と参照先

| 対象 | 方針・正本 |
|---|---|
| 参加者実装 | 原則`aichallenge/workspace/src/aichallenge_submit/`に閉じる |
| MPC/MPCC | 対象packageの`AGENTS.md`と[統合仕様](docs/spec/mpc-integration.md) |
| 評価基盤 | `aichallenge_system/`。変更目的・互換性への影響を明確にする |
| 学習・データ抽出 | `aichallenge/ml_workspace/`。実行用ROS packageと分離 |
| 起動・環境 | [README](README.md)、[architecture](docs/spec/architecture.md)、[Compose](docs/spec/compose-overlays.md)、`Makefile` |
| 参加者・評価契約 | [participant-interface](docs/interface/participant-interface.md)、[evaluation-interface](docs/interface/evaluation-interface.md) |
| 評価・提出 | [log-design](docs/spec/log-design.md)、[safety-gates](docs/spec/safety-gates.md)、[submission-workflow](docs/spec/submission-workflow.md) |
| 2026への移行 | [competition-rules](docs/spec/competition-rules.md)、[open-questions](docs/spec/open-questions.md)、[V2X](docs/spec/v2x-multivehicle.md) |
| agent設定 | [Codex harness](docs/spec/codex-harness.md) |

永続仕様は`docs/spec/`、破ると依存側が壊れる契約は`docs/interface/`を更新する。
短期計画の`docs/plan/`はgitignore対象。詳細分類は[docs/README.md](docs/README.md)。
大会ルールは[2026 SW公式ページ](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html)が正本。
仕様変更に依存する作業では最新公式情報を確認し、差分を文書へ反映してから実装する。

## 維持する契約

以下は現行互換の要点。変更が必要なら先に`docs/interface/`へ影響と移行方針を書く。

1. Domain 0はAWSIMと`awsim_state_manager_node`の管理面専用。車両AutowareはDomain 1..N。
   多車両は`make dev2/dev3/dev4`の`docker compose -p N`分離を使う。
2. クロスドメイン通信は`/v2x/vehicle_positions`と`v2x_msgs`。`domain_bridge`を復活させない。
3. `/admin/awsim/start`、`/admin/awsim/reset`、`/admin/awsim/state`と、
   `/awsim/state`、`/awsim/control_mode_request_topic`の名前・型・責務・状態文字列を維持。
   `autostart_orchestrator_node`へ`/admin/awsim/start`の責務を移さない。
4. `aichallenge_submit_launch`の`aichallenge_submit.launch.xml`を維持。
   `control_method`は`mpc/pure_pursuit/tiny_lidar_net/pilot_net/joycon`の範囲。
5. 最終指令は`/control/command/control_cmd`。
   `/localization/kinematic_state`と`/planning/scenario_planning/trajectory`の連結、
   評価起動ハンドシェイクの`/set_initial_pose`を維持。
6. 提出tar.gzの最上位は`aichallenge_submit/`。
   `result-summary.json`、`dN-result-details.json`の名前・schema version・主要キーを維持。
7. `output/latest/`の配置・リンク名と`HOST_UID/HOST_GID`による成果物所有者設計を維持。

## 実装・検証

ホストにROS 2があると仮定せず、Docker Compose / Makefileを実行入口にする。
package依存、CMake、Python entry point、launch、param yamlの整合を保つ。
重み変更はpath・dtype・shape・実行依存を確認。環境固有値は設定や引数へ分離する。

| 変更 | 最低限の検証 |
|---|---|
| 文書・Skills | リンク、記述整合。Skillsはfrontmatterも検証 |
| launch/yaml/package、ROS node | `make autoware-build`。ロジック変更には対象packageの有効な回帰test |
| 制御・trajectory・localization | 上記に加え`make dev`または`make gate*`、topic hz・ログ確認 |
| 多車両・V2X | `make dev2`以上、DomainとV2X・全車の挙動を確認 |
| 提出前 | `./create_submit_file.bash` → `./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz` → `make eval` |
| 実車・遠隔 | シミュレータ確認後、`vehicle/setup_check.sh`、通信・停止手順確認 |

テストは変更が壊し得る挙動を検証する。軽微な可逆変更の文言をなぞるtestは追加しない。
必要な検証が通った後は、新しい変更・失敗・未解決の懸念がなければ再実行や範囲拡大をしない。
ROS package testはコンテナ内で`colcon test --packages-select <package>`、
`colcon test-result --verbose`。品質確認は変更に応じた`pre-commit` hookを使う。

ログは`output/latest/`を入口にして、各リンクの実体、run ID、Domain、commit/configを固定する。
JSON・log・MCAPが別runの場合は混ぜない。過去のbuild/test/runは現HEADの証拠と区別する。

## Skillsの選択

必要な観点のSkillだけ読む。名前が似ていても一括起動しない。

- 差分レビュー: `code-reviewer`。契約変更の影響調査: `interface-guardian`。
- build/launch/DDS障害: `error-analyzer`。完走・penalty: `evaluation-analyzer`。
- rosbagの時系列・欠損・車両挙動: `data-analyzer`。
- MPCCの未解明回帰・authority/solver/proof監査: `mpcc-root-cause-auditor`。
- 大会・ROS等の外部仕様: `web-summarizer`。

## 保護するもの

- ユーザー変更のrevert、`git reset --hard`、広範囲削除を無断で行わない。
- `output/outputs/`、build/install/log、rosbag、`*.mcap/*.db3`、提出tar.gz、`.env`は
  手編集・コミットしない。必要な解析と正規手順での生成は可。
- `vehicle/remote/`の速度・操舵・制動・通信変更をシミュレータ確認なしに実車へ進めない。
- Kaggle由来の`expXXX`、CV/LB、`uv`、`pipeline.sh`を本リポジトリの前提にしない。
