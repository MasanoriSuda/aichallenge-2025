# aichallenge-racingkart

本リポジトリでは、自動運転AIチャレンジ 2026 SW 部門に向けた開発環境のベースを提供します。参加者は Autoware Universe をベースとした自動運転ソフトウェアを開発し、AWSIM 上のレーシングカートおよび決勝の実車両へインテグレートします。2026 ルールでは、従来のタイムアタックではなく、複数車両の同時走行、追い抜き、安全ゲート、ペナルティを含むレース形式を前提にします。

This repository provides a base development environment for the Automotive AI Challenge 2026 software class. Participants develop autonomous driving software based on Autoware Universe and integrate it into a racing kart in AWSIM and, for finalists, a real racing kart. The 2026 rules are race-oriented: multiple vehicles run together, with overtaking, safety gates, and penalties.

## ドキュメント / Documentation

ルールの詳細や環境構築方法など、大会に関する情報を以下のページで提供します。

Toward the competition, we will update the following pages to provide information such as rules and how to set up your dev environment. Please follow them. We are looking forward your participation!

- [日本語ページ](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/)
- [English Page](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/en/)
- [SW 部門ルール](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html)
- [ローカルルール要約](docs/spec/competition-rules.md)
- [スクリプト設計メモ（評価/ビルド/起動）](aichallenge/README.md)

## リポジトリ構成（トップレベル）

- `aichallenge/`: シミュレータ/Autoware/評価の起動・操作スクリプト群
- `vehicle/`: 実車環境向け（セットアップ確認、Zenoh、rosbag など）
- `remote/`: 実車/遠隔接続の補助（SSH/Zenoh/RViz/joy）
- `docs/`: ガイド・スライドは [`docs/guide/`](docs/guide/)、仕様は [`docs/spec/`](docs/spec/)、インターフェイス契約は [`docs/interface/`](docs/interface/)
- `submit/`: 提出物（tar.gz）置き場
- `output/`: 実行結果・ログ出力先（生成物）

## Docker Compose（推奨）

### 全体像（開発: Makefile / 個別起動）

```text
Host (you)
  ├─ make autoware-build / make dev / make eval / make simulator ...
  └─ docker compose（.env の COMPOSE_FILE でオーバーレイを選択）
        ├─ docker-compose.yml       (ベース: 全サービス定義)
        │     ├─ autoware           (Autoware)
        │     ├─ autoware-build     (colcon build 専用)
        │     ├─ simulator          (AWSIM)
        │     ├─ autoware-command   (ros2 service/topic の単発操作)
        │     ├─ zenoh              (Zenoh ブリッジ)
        │     ├─ driver             (実車ドライバ)
        │     └─ rviz2              (可視化)
        ├─ docker-compose.gpu.yml   (NVIDIA GPU オーバーレイ、オプション)
        └─ docker-compose.sound.yml (PulseAudio / simulator のみ、オプション)
        → output/ にログ・結果を出力（最新結果は output/latest/d<domain>/）
```

オーバーレイの選択は `.env` の `COMPOSE_FILE` 行で行う（`docker compose` が自動読み込み）。
`./setup.bash env` を実行すると GPU の有無を自動検出して `.env` を生成する。

### はじめてのセットアップ

```bash
# 1. 環境セットアップ（Docker インストール〜.env 生成〜イメージ取得〜AWSIM DL）
./setup.bash bootstrap

# または個別に実施する場合:
./setup.bash env            # .env 生成（GPU/CPU 自動判定）
./setup.bash network tune   # DDS ホストチューニング（rmem_max + loマルチキャスト）
./setup.bash download awsim # AWSIM バイナリのダウンロード
./setup.bash pull image     # Autoware ベースイメージの取得

# 2. 開発イメージのビルド
./docker_build.sh dev

# 3. ROS 2 ワークスペースのビルド（コンテナ内 colcon）
make autoware-build

# 4. 開発ループ（AWSIM + Autoware 起動）
make dev
make autoware-request-initialpose
make autoware-request-control
make down
```

コンテナはホストの UID/GID（`HOST_UID`/`HOST_GID`）で動作するため、`output/` 配下の生成物はホストユーザー所有になり root 権限は不要です。

## まずは読んでほしいもの

- [初学者向けセットアップ資料](./docs/spec/how-to-setup.md)
- [初学者向け説明資料](./docs/spec/introduction.md)
- [2026 SW 部門ルール要約](./docs/spec/competition-rules.md)
- [提出ワークフロー](./docs/spec/submission-workflow.md)
- [未確定事項・運営確認リスト](./docs/spec/open-questions.md)
- [初学者向けリポジトリ入門スライド (Marp)](./docs/guide/beginner-deck.marp.md)

## OSS 貢献にあたって

PR を出す前に `pre-commit run -a` を通してください。

```.sh
check for merge conflicts................................................Passed
check xml................................................................Passed
check yaml...............................................................Passed
detect private key.......................................................Passed
fix end of files.........................................................Passed
mixed line ending........................................................Passed
shellcheck...............................................................Passed
shfmt....................................................................Passed
```
