# Log設計メモ（/output 配下に集約）

> 仕様ドキュメント（現仕様の正）。最終確認: 2026-09-01。文書運用方針は [docs/README.md](../README.md) を参照。

作成日: 2026-01-27  
更新日: 2026-09-01

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

### 3.4 AWSIM result provenance

AWSIMは`result-summary.json`と`dN-result-details.json`をprocess cwdへ書き出す。
`run_simulator.bash`はstdoutの出力先を変えるだけでなく、AWSIM起動前にcwdを正規化済みの
`LOG_DIR`（通常`/output/<run_id>/`）へ変更する。これによりAWSIM結果、`awsim.log`、各
`d<N>/`を同じrun identityへ閉じ込める。

リポジトリ直下の`aichallenge/result-summary.json`や`dN-result-details.json`をdev runの
根拠として参照しない。これらはcwd契約修正前のrunにより上書きされ、別時刻の車両結果が
混在し得る。結果JSONを持たない古いrunは、bagが残っていてもcompetition acceptance上は
`incomplete`とする。

### 3.5 追い越しDecision Trace

追い越しの候補生成、制御実行、実行中の代替側切替は、`autoware.log` の
`Overtake decision trace:` で相関できるようにする。記録段階は次の3種類とする。

- `stage=planning`: 左右候補、GapPlanner棄却gate、solver/bridge判定、採用結果
- `stage=tracking`: 採用経路の追従状態、速度制限、制御fallback
- `stage=runtime-failover`: 実行中Mission失効時の現側・代替側評価と採用動作

`stage=planning` の `pass_through=1` / `authority=1/accepted-pass-through` は、
GapPlanner回廊とreachable bridgeが成立し、走行ラインが既に回廊内または必要横移動が
最小shift未満だったことを表す。これは物理判定の省略ではない。また
`qualified=0` の間は `follow_cap_suppressed=0` とし、同じtarget/sideのtracking solveが
成立してからのみ前車速度capを解除する。

`stage=runtime-failover` は自由文 `trigger` に加えて固定カテゴリ `trigger_gate` を持つ。
代表カテゴリは `target-wall-conflict`、`pass-entry-no-prefix`、
`pass-entry-wall-unresolved`、`optimized-horizon-physical`、
`live-corridor-unavailable` である。`source` は判定経路を表し、
`dynamic-wait-resolver`、`mpcc-lite-same-side`、`mpcc-lite-cross-side`、
`opponent-side-replan` を区別する。実差し替え後は
`replace-current-applied` / `replace-alternate-applied` / `replacement-rejected` を記録する。

相関キーは次のように役割を分ける。

- `attempt`: Follow中の動的障害物回避を含む、追い越し要求1回の識別子
- `mission_episode`: `ShiftOut` 以降の正式なOvertakeLine Mission識別子。Mission開始前は0を許容する
- `generation`: 同一Mission内で経路を差し替えた世代
- `target`: 対象車両ID

棄却理由は自由文だけにせず、`planner_gate`、`trigger_gate`、`action`、`source` などの
固定カテゴリを併記する。
同一状態を制御周期ごとに出さず、カテゴリ変化時と低頻度heartbeatだけを記録する。数値の微小変動は
change判定へ含めない。runtime-failoverの自由文 `reason`、および分類済み `trigger` の
詳細変化だけでも再出力しない。未分類triggerは欠陥を隠さないため、分類が追加されるまで
保守的に変化を記録する。これによりログ量を抑えつつ、次を区別可能にする。

1. 左右どちらにも物理的な回廊がない
2. 回廊はあるがsolverまたはbridgeで不成立
3. 候補は成立したがauthority/admissionで採用されない
4. 実行中に現側が失効し、代替側へ切り替えた／切り替えられなかった

### 3.6 追い越しExecution AuthorityとEpisode Summary

個別の候補判定だけでなく、MPC問題へ適用する直前の最終所有者を
`Overtake execution authority:`へ記録する。主要項目は次のとおり。

- `action`: 当該周期のCruise、Follow、ShiftOut、Pass、DynamicWait、
  ContactEscape、Recovery、SafetyBrake
- `lateral_owner`: RacingLine、GapPlanner、OvertakeLine、DynamicWaitPrefix等
- `longitudinal_owner`: FollowCap、OvertakeLine、PassFloor、SolverFallback、
  SafetyBrake等
- `corridor_min` / `wall_min`: target制約込み／壁だけの将来最小回廊幅
- `valid_until` / `rear_clear`: Missionの静的・動的有効距離と予測rear-clear距離
- `conflict`: 同時成立してはいけない権限の固定カテゴリ

同一authorityは制御周期ごとに出さず、カテゴリ変化時と低頻度heartbeatだけを
記録する。`conflict!=none`はWARNとし、安全判定と速度floor、複数横所有者、
front-cap解除とFollow capなどの組合せをsilentに通過させない。
左右branchの仮評価は含めず、実際の制御問題へ採用されたauthorityだけを記録する。

`Overtake episode summary:`は`episode`終了時に一度だけ出す。所要時間、通過phase、
最低速度、最小回廊幅、最大要求横加速度、Mission世代数、authority変更回数、
DynamicMissionWait／ContactEscape回数、終了理由を含める。これを追い越し完遂率と
外れ走行の一次集計単位とする。

### 3.7 AWSIM衝突指標と静的壁証明の相関

開発用rosbagのallowlistには`/aichallenge/pitstop/condition`を含める。topicが存在する環境では
制御器は初回値を
`Pitstop condition baseline:`、値の変化を`Pitstop condition transition:`として
change-awareに記録する。transitionには同時点の生姿勢、速度、直前指令、および制御器が
利用する静的occupancy grid上の実寸footprint接触判定を含める。

current AWSIMでこのtopicがpublishされない場合、bag metadataに現れないこと自体を
`condition=unavailable`として扱う。代わりにodometryの連続sample間で、速度低下量が
`max(1.0 m/s, 2 * abs(configured a_min) * dt)`以上かつ`dt <= 0.25 s`なら、
`Abrupt measured speed loss:`を一度記録する。このeventには前後速度、観測加速度、生pose、
静的map sample、condition可用性、decision ID、直前commandを含める。

これにより、AWSIM側の衝突・ペナルティ指標が変化した時点で、次を区別する。

1. 静的地図上でも車体接触していた
2. 静的地図はclearだがAWSIM指標だけが変化した
3. 指標変化より前に制御器が減速・停止を命令した
4. 前進指令中に外部要因で実速度が急減した

これらの観測は制御authority、command、衝突判定を変更しない。静的wall certificateは
設定済みoccupancy gridに対する証明であり、AWSIM collider／penalty sourceとの同値性は
保証しない。地図とAWSIM colliderの不整合が
確認されるまでは、wall marginやsolver重みの調整根拠として使用しない。

## 4. 今後の改善候補

- `meta.json`（run_id, started_at, exit_code, image, host, container など）の充実
- `ROS_HOME` / `ROS_LOG_DIR` を `output/<run_id>/dN/ros/` へ確実に誘導
- `logs/`, `results/`, `artifacts/` への整理（互換 symlink を残しつつ移行）
- 古い run のローテーションポリシー（要件確定後）
