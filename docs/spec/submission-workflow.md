# Submission Workflow

> 2026 SW 部門向けの提出ワークフロー。
> 正本は公式ドキュメントと公式提出環境。ここでは現行リポジトリで再現できる提出物作成・ローカル検証手順をまとめる。
>
> 確認日: 2026-07-01

## Scope

この文書は以下を扱う。

- `aichallenge_submit/` の提出 tar.gz 作成
- ローカルでの build / dev / eval 確認
- 公式提出前のチェック

公式プラットフォームへのアップロード方法、提出回数制限、提出先 URL、認証情報、クラウド評価環境の詳細は運営管理のため、この文書では固定しない。

## Source Of Truth

- 公式ルール: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html>
- ローカル参加者契約: [`../interface/participant-interface.md`](../interface/participant-interface.md)
- ローカル評価契約: [`../interface/evaluation-interface.md`](../interface/evaluation-interface.md)

## Submission Artifact

現行リポジトリの提出物は `aichallenge/workspace/src/aichallenge_submit/` を最上位ディレクトリ名 `aichallenge_submit/` として固めた tar.gz。

```bash
./create_submit_file.bash
```

生成物:

```text
submit/aichallenge_submit.tar.gz
```

tar の最上位が `aichallenge_submit/` であることを確認する。

```bash
tar tf submit/aichallenge_submit.tar.gz | head
```

## Local Verification

### 1. Build

```bash
make autoware-build
```

確認すること:

- colcon build が成功する。
- `aichallenge_submit/` 配下の package が解決される。
- `package.xml` / `CMakeLists.txt` / Python entry point の不足がない。

### 2. Dev Run

```bash
make dev
make autoware-request-initialpose
make autoware-request-control
```

確認すること:

- `/localization/kinematic_state` が publish される。
- `/planning/scenario_planning/trajectory` が publish される。
- `/control/command/control_cmd` が publish される。
- `output/latest/d1/autoware.log` に fatal error がない。

停止:

```bash
make down
```

### 3. Safety Gates

現行ターゲット:

| コマンド | 対象 |
|---|---|
| `make gate1` | 障害物停止 |
| `make gate2` | 追い越し |
| `make gate3` | 車線維持 |

各 gate の確認観点は [`safety-gates.md`](safety-gates.md) を参照する。

### 4. Eval Image

```bash
./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz
make eval
```

確認すること:

- eval image build が成功する。
- `output/latest/d<N>/result-summary.json` が生成される。
- `output/latest/d<N>/result-details.json` が生成される。
- 未完走の場合でも、原因を `autoware.log` / result JSON / rosbag から追える。

## Pre-Submission Checklist

- [ ] `git status --short` で提出に関係ない変更が混ざっていない。
- [ ] `make autoware-build` が通る。
- [ ] `make dev` で制御出力が出る。
- [ ] 必要な safety gate を確認した。
- [ ] `./create_submit_file.bash` で tar.gz を作成した。
- [ ] tar の最上位が `aichallenge_submit/`。
- [ ] `./docker_build.sh eval --submit submit/aichallenge_submit.tar.gz` が通る。
- [ ] `make eval` の結果を確認した。
- [ ] `output/latest/` の result JSON と log を保存・確認した。

## Do Not Commit

- `submit/*.tar.gz`
- `output/`
- `aichallenge/build/`
- `aichallenge/install/`
- `aichallenge/log/`
- `rosbag2*/`
- `*.mcap`

## Organizer Confirmation Needed

以下は公式提出前に運営確認または公式ページ更新確認が必要。

- 公式提出先と認証方式。
- 1日あたりの提出回数制限。
- クラウド評価で使われる repository / Docker image / branch。
- 2026 公式インターフェースと現行 `aichallenge_submit/` layout の差分。
- 提出物に含めてよいモデル重み、データ、外部依存。
