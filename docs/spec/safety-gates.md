# Safety Gates

> Automotive AI Challenge 2026 SW 部門の安全ゲートに対するローカル検証方針。
>
> 確認日: 2026-07-01

## Source Of Truth

公式ルールでは、決勝以降に以下の安全ゲートをすべてクリアする必要がある。

- 障害物停止
- NPC 追い越し
- 車線維持

公式ルール: <https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/competition/sw-class.html>

## Local Targets

現行 Makefile には以下の gate ターゲットがある。

| コマンド | シナリオ | 起動内容 |
|---|---|---|
| `make gate1` | 障害物停止 | AWSIM を `--safety-gate 1` で起動し、Autoware も起動する |
| `make gate2` | 追い越し | AWSIM を `--safety-gate 2` で起動し、Autoware も起動する |
| `make gate3` | 車線維持 | AWSIM を `--safety-gate 3` で起動し、Autoware も起動する |

`Makefile` の `gate1/2/3` は `SIM_MODE=gate1/2/3` を指定し、`aichallenge/run_simulator.bash` が `aichallenge/simulator_scripts/gate.sh` に番号を渡す。`gate.sh` は AWSIM を `--safety-gate <番号>` で起動する。

つまり、シナリオ起動口は既にある。新規実装が必要なのは、各シナリオを通過するための参加者側の認識・判断・制御ロジックである。

## Gate Checklist

### 障害物停止

運営回答により、障害物情報の入力は `/v2x/vehicle_positions` のみを使用する。Gate1 は、前方の停止対象を V2X 上の位置情報として受け取り、追い越し不可または危険距離では停止するシナリオとして扱う。

現行実装では、MPC controller の `use_v2x_stop=true` が Gate1 用の縦方向速度制限を有効にする。これは `use_obstacle_avoidance` の横回避制約とは別で、前方対象を検出したら MPC の `v_max` / `v_ref` に速度 cap を重ねる。

確認すること:

- 障害物または停止対象に接近したとき、制御出力が安全側に落ちる。
- 停止後に不要な再加速をしない。
- 制御出力に NaN / Inf / 急激なスパイクがない。
- 停止判断の根拠が log または rosbag から追える。

見る topic:

- `/control/command/control_cmd`
- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- `/v2x/vehicle_positions`

見る log:

- `V2X stop: braking ... gap=<m> ... v_cap=<m/s>`
- `V2X stop: holding_stop ... v_cap=0.00m/s`
- `V2X stop: clear`

### NPC 追い越し

確認すること:

- 他車両または NPC の位置を認識している。
- 追い越し中に壁接触、コース逸脱、他車衝突を避ける。
- 追い越し後に走行ラインへ復帰できる。
- V2X 入力がある場合、遅延や欠損時も危険な制御に倒れない。

見る topic:

- `/v2x/vehicle_positions`
- `/control/command/control_cmd`
- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`

### 車線維持

確認すること:

- lane / reference path から継続的に逸脱しない。
- 急操舵や蛇行が過大でない。
- localization がジャンプしても制御が破綻しない。
- ペナルティまたは wall contact が増えない。

見る topic:

- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- `/control/command/control_cmd`
- map / reference path 関連 topic

## Evidence To Keep

各 gate で以下を残す。

- 実行コマンド
- `output/latest/d<N>/autoware.log`
- `result-summary.json`
- `result-details.json`
- rosbag / mcap
- 問題発生時刻と該当 topic

## Open Questions

- 公式判定の合否基準。
- 障害物・NPC・車線維持のシナリオ詳細。
- スタック時の手動復帰可否。
