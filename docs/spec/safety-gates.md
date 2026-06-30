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

```bash
make gate1
make gate2
make gate3
```

これらが公式 2026 安全ゲートと完全に一致するかは未確認。現時点では「ローカル安全ゲート検証シナリオ」として扱う。

## Gate Checklist

### 障害物停止

確認すること:

- 障害物または停止対象に接近したとき、制御出力が安全側に落ちる。
- 停止後に不要な再加速をしない。
- 制御出力に NaN / Inf / 急激なスパイクがない。
- 停止判断の根拠が log または rosbag から追える。

見る topic:

- `/control/command/control_cmd`
- `/localization/kinematic_state`
- `/planning/scenario_planning/trajectory`
- センサ入力 topic

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

- `make gate1/2/3` と公式安全ゲートの対応関係。
- 公式判定の合否基準。
- 障害物・NPC・車線維持のシナリオ詳細。
- スタック時の手動復帰可否。
