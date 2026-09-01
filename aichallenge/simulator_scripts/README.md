# simulator_scripts

モード別の AWSIM 起動スクリプト。**起動引数の正本は各 `<mode>.sh`**。

## 呼び出しの仕組み

```
make simulator-<mode> / make dev / make dev2..dev4 / make gate1..gate3 / make e2e-single / make e2e-teacher / make e2e-npc-single / make e2e-npc-gap-teacher / make e2e-peer-audit-mpc / make e2e-peer-audit-student / make e2e-final-contact-teacher / make e2e-final-precontact-teacher / make e2e-final-precontact-teacher-all
  → docker compose up simulator (SIM_MODE=<mode>)
    → run_simulator.bash <mode> [args...]
      → simulator_scripts/<mode>.sh [args...]

make eval → run_evaluation.bash → evaluation.launch.xml
  → run_simulator.bash <sim_mode>（既定 eval、SIM_MODE で上書き可）
```

- `run_simulator.bash` はモード名（第1引数 > `SIM_MODE` > 既定 `simulator`）で `<mode>.sh` に委譲する。
- モード名 `dev<N>` / `gate<N>` は `dev.sh N` / `gate.sh N` に解決される
  （例: `SIM_MODE=dev2` → `dev.sh 2`）。
- 不明なモードはフォールバックせず、対応モード一覧を出して exit 1。
- Makefile は `*.sh` を wildcard で拾って `make simulator-<mode>` を自動生成する。
  `dev2..dev4` / `gate1..gate3` のエイリアスも `SIM_MODES` に追加してあり、
  `make simulator-dev2` / `make simulator-gate1` のように使える（AWSIM のみ起動）。
- `make dev` / `make gate1..gate3` / `make e2e-single` / `make e2e-teacher` / `make e2e-npc-single` / `make e2e-npc-gap-teacher` / `make e2e-peer-audit-mpc` / `make e2e-peer-audit-student` / `make e2e` / `make e2e-final` / `make e2e-final-contact-teacher` / `make e2e-final-precontact-teacher` / `make e2e-final-precontact-teacher-all` は AWSIM に加えて Autoware も起動する複合ターゲット。
  `make dev2..dev4`、`make e2e-final`、各`e2e-final-*-teacher*`は N 台分の autoware を別 compose
  プロジェクト（ROS_DOMAIN_ID=1..N）で起動する。`e2e-final` はsync開始のため、全車Ready後に
  `make awsim-request-start` を実行する。

## モード一覧

| スクリプト | 用途 | 引数 | 主な設定 |
|---|---|---|---|
| `eval.sh` | 評価 | - | 1台 / 6 laps / 600s / count開始 / handicap・wall-recovery・ranking off |
| `dev.sh` | 開発 | 車両数 N（既定 1） | unlimited laps・timeout / count開始 / handicap・wall-recovery・ranking off |
| `parallel.sh` | 複数台レース | - | 3台 / 6 laps / 600s / sync開始 / handicap・ranking・start-random off / wall-recovery off |
| `gate.sh` | Safety Gate テスト | テスト番号 1/2/3/all（既定 all） | 1台。all は test1〜3 を順次実行 |
| `e2e-single.sh` | E2E 単車ベースライン | - | 1台 / NPC 0 / 3 laps / 420s / LiDAR on / 固定スタート |
| `e2e-teacher.sh` | E2E MPC教師収集 | - | `e2e-single`と同条件 / LiDAR・GNSS・IMU on / controllerはMake側でMPCを明示 |
| `e2e-npc-single.sh` | E2E NPC学生gate | - | 1台 / NPC 2 / 3 laps / seed 2026 / LiDAR on / V2X off |
| `e2e-peer.sh` | E2E peer監査pair | - | 3台 / 3 laps / domain 2低速peer / domain 3をMPCまたはTinyへ切替。現MPC runは教師へ自動採用しない |
| `e2e.sh` | E2E 練習参考 | - | 1台 / NPC 2 / 6 laps / Camera・LiDAR on |
| `e2e-final.sh` | E2E 決勝参考 | - | 4台 / 6 laps / sync開始 / handicap・ranking on |
| `sample-scenario.sh` | シナリオ指定起動 | - | `StreamingAssets/Race/official.yaml` を `--scenario` で読み込む |

`make e2e-npc-gap-teacher`は`e2e-npc-single.sh`と同じworldを使用し、TinyLidarNetの
明示的な`gap_teacher`モードを有効にする。これは教師候補収集専用であり、本番既定値の
`control_mode=fixed`を変更しない。
再現可能なrun-level train/validation分割では、例えば
`make e2e-npc-gap-teacher E2E_START_RANDOM_SEED=2027`のようにseedを明示する。
未指定時は従来どおり2026であり、整数以外はAWSIM起動前に拒否する。

`make e2e-final-contact-teacher`は`e2e-final.sh`の決定論的worldを使う診断専用A/Bである。
domain 1〜3はproductionの`fixed_lidar_brake`、domain 4だけをteacher-only
`gap_teacher`にする。全Domainが`Grounded`になった後に`make awsim-request-start`で開始する。
domain 4のbagはFinish・接触・stall gateを通るまで学習データへ採用しない。

`make e2e-final-precontact-teacher`は、上記A/Bで不合格だったhistorical
`gap_teacher`と区別した後継診断である。d4だけを`precontact_teacher`にし、3点以上の
coherentな側方returnと、障害物方向へ操舵させないprojectionを評価する。これも
run-level gate合格までは教師データへ採用しない。

`make e2e-final-precontact-teacher-all`は次のrun-level admissionであり、4 domainすべてを
`precontact_teacher`にする。個別d4回避ではなく、対称なteacher同士がFinishまで競合を
解消できるかを確認する。1台でもstall/contact/terminal gateを満たさなければ抽出しない。
| `multiplay-server.sh` | Multiplay 専用サーバー | - | `-batchmode -nographics`、port 7777 |
| `multiplay-host.sh` | Multiplay ホスト | - | 127.0.0.1:7777、vehicle-index 1 |
| `multiplay-client.sh` | Multiplay クライアント | - | 127.0.0.1:7777、vehicle-index 1 |
| `simulator.sh`（既定） | 引数なし素起動 | - | 起動時UIで設定を選択 |

- start-mode: `dev.sh` は count（全車接地後にカウントダウン開始、`/admin/awsim/start` 不要）。
  `eval.sh` / `parallel.sh` は sync（`/admin/awsim/start` 待ち。評価では awsim_state_manager が
  自動送信、手動で送るなら `make awsim-request-start`）。
- 通常の dev/gate はセンサー（camera/LiDAR）off が既定。E2E モードは各 script に明記したセンサーだけを有効にする。
- 現在同梱する AWSIM の LiDAR CLI は `on/off` を受け付ける。旧 upstream script の `cpu` は無効値になるため使用しない。
- 現在同梱するAWSIMではGNSS publisherを無効にすると車両がReady/Groundedへ到達しない。
  E2E modeのGNSSは起動ハンドシェイク専用に有効化するが、TinyLidarNetはGNSSを
  subscribeしない。モデル入力契約と評価基盤の初期化を分けて監査する。
- 引数の完全な仕様は AWSIM リポジトリの `docs/AIChallenge/specs/CLI.md` を参照。

## 設計方針

**あえてモード別 1 ファイルにしている**（config 集約しない）。
1 ファイルで完結し、コピーしてモードを増やせ、`gate` のような差分も素直に書ける。
そのため `dev.sh` と `eval.sh` のようなほぼ同一ファイルもあるが、意図した重複であり DRY 化しない。

新モードは近いものを `cp` して引数を直すだけ（`simulator-<新mode>` が自動で使える）。
末尾の GPU 切り替えコメントは編集対象行の隣に置くガイドなので、共通化せず各ファイルに残す。
