# Log設計メモ（/output 配下に集約）

> 仕様ドキュメント（現仕様の正）。最終確認: 2026-08-19。文書運用方針は [docs/README.md](../README.md) を参照。

作成日: 2026-01-27  
更新日: 2026-08-19

対象: `docker-compose.yml`（make 経由 / 主要パス）・`aichallenge/run_evaluation.bash`（評価オーケストレータ）

## 0. 目的

- **評価1回の結果と実行時ログを、1つの「実行単位ディレクトリ」に集約して追跡可能にする**
- **/output（ホストの `./output`）配下を見るだけで、何が起きたか・なぜ失敗したかを再現できる状態にする**

## 1. 現在の実行経路

### 主要パス（docker compose + make）

- **ビルド**: `./docker_build.sh [dev|eval] [--submit <tar>]`
- **開発 / 評価起動**: `make dev` / `make dev2..4` / `make eval`
- `docker-compose.yml` で `./output:/output` と `./aichallenge:/aichallenge` をマウント

`make` 経由であれば `HOST_UID`/`HOST_GID` が自動設定され、`output/` の所有者がホストユーザになる。

### レガシーパス（docker_run.sh / rocker）

`./docker_run.sh eval` による単発実行は旧来の一回限り用途として残っている。eval イメージは `CMD ["bash", "/aichallenge/run_evaluation.bash"]` で評価を実行するが、`./aichallenge` がコンテナにマウントされないため **スクリプト変更を反映するにはイメージの再ビルドが必要**。

## 2. 現在の出力レイアウト（実装済み）

### 2.1 run ディレクトリ

```
output/
  <YYYYMMDD-HHMMSS>/
    awsim.log                  # run_simulator.bash が LOG_DIR 直下に書く
    d<N>/                      # N = 1..4（Autoware domain per vehicle）
      autoware.log
      capture/
      ros/log/
      rosbag2_autoware/
      <dN>-result-details.json
      result-summary.json      # AWSIM が cwd (= d<N> or run_dir) に書く
  latest/                      # 実ディレクトリ（autostart_orchestrator が更新）
    d<N>/                      # 車両ごとのサブディレクトリ（N = domain id）
      result-details.json      -> ../<run_id>/d<N>/<dN>-result-details.json
      result-summary.json      -> ../<run_id>/d<N>/result-summary.json（またはその親）
      capture.mp4              -> ../<run_id>/d<N>/capture/cap-*.mp4
      rosbag2_autoware.mcap    -> ../<run_id>/d<N>/rosbag2_autoware/...
      motion_analytics.html    -> ../<run_id>/d<N>/motion_analytics-*.html
      autoware.log             -> ../<run_id>/d<N>/autoware.log
    docker_build.log           -> docker/<ts>-docker_build-<pid>.log
    docker_run.log             -> docker/<ts>-docker_run-<pid>.log
  docker/
    <ts>-docker_build-<pid>.log
    <ts>-docker_run-<pid>.log
```

### 2.2 ログの基本方針

- 重要ログは標準出力に出すだけでなく、**必ずファイルに tee する**
- build / run / eval の各段階で、コマンド・引数・環境（GPU/DOMAIN_ID 等）をログ先頭に記録する

## 3. 設計上の注意点

### 3.1 rosbag の安全停止

rosbag compose サービスには `stop_grace_period: 10s` を設定してあり、`docker compose down` 時に SIGINT が届いてメタデータ/クローズ処理が完了するまで待機する。`docker compose down --timeout` を短くすると rosbag が破損する可能性があるため注意。

### 3.2 COMPOSE_FILE による GPU/音声切り替え

`.env` の `COMPOSE_FILE` 変数でオーバーレイを選択する。5 つの compose ファイルの組み合わせは `makefile-target-naming.md` を参照。

### 3.3 `output/latest/` について

`latest/` は `autostart_orchestrator_node.py`（`_refresh_latest_artifact_links`）が評価完了時に更新する実ディレクトリ。`latest/d<N>/` 配下に最新 run の成果物を指す symlink が置かれる。`docker_build.log` / `docker_run.log` については `docker_build.sh` / `docker_run.sh` が `latest/` 直下に symlink を直接作成する。`topic_check.sh` が `output/latest/topic_check.txt` を出力する用途も引き続き有効。

### 3.4 追い越しDecision Trace

追い越しの候補生成、制御実行、実行中の代替側切替は、`autoware.log` の
`Overtake decision trace:` で相関できるようにする。記録段階は次の3種類とする。

- `stage=planning`: 左右候補、GapPlanner棄却gate、solver/bridge判定、採用結果
- `stage=tracking`: 採用経路の追従状態、速度制限、制御fallback
- `stage=runtime-failover`: 実行中Mission失効時の現側・代替側評価と採用動作

相関キーは次のように役割を分ける。

- `attempt`: Follow中の動的障害物回避を含む、追い越し要求1回の識別子
- `mission_episode`: `ShiftOut` 以降の正式なOvertakeLine Mission識別子。Mission開始前は0を許容する
- `generation`: 同一Mission内で経路を差し替えた世代
- `target`: 対象車両ID

棄却理由は自由文だけにせず、`planner_gate`、`action`、`reason` などの固定カテゴリを併記する。
同一状態を制御周期ごとに出さず、カテゴリ変化時と低頻度heartbeatだけを記録する。数値の微小変動は
change判定へ含めない。これによりログ量を抑えつつ、次を区別可能にする。

1. 左右どちらにも物理的な回廊がない
2. 回廊はあるがsolverまたはbridgeで不成立
3. 候補は成立したがauthority/admissionで採用されない
4. 実行中に現側が失効し、代替側へ切り替えた／切り替えられなかった

## 4. 今後の改善候補

- `meta.json`（run_id, started_at, exit_code, image, host, container など）の充実
- `ROS_HOME` / `ROS_LOG_DIR` を `output/<run_id>/dN/ros/` へ確実に誘導
- `logs/`, `results/`, `artifacts/` への整理（互換 symlink を残しつつ移行）
- 古い run のローテーションポリシー（要件確定後）
