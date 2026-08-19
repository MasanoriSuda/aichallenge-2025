# Requirements

## 目的

2026-08-19の3周試走で観測した、動的障害物回避のprimary候補失効後に
alternate候補が評価されず、Follow/失速へ戻る経路を明確化・縮小する。

同時に、Follow中の動的回避試行とOvertakeLine Missionを同じ`episode`として
記録している曖昧さを解消し、次回ログだけで「候補生成・到達性・authority・
tracking・runtime failover」のどこで失効したか判定可能にする。

## 変更範囲

- `multi_purpose_mpc_ros`内の動的障害物 lateral escape 候補評価
- Overtake decision traceの相関IDと棄却ゲート
- active Mission失効時のruntime failover trace
- 上記の単体テスト

## 制約

- 速度、加速度、壁余裕、車間、Recoveryパラメータは変更しない。
- 壁接触、EmergencyBrake、V2X不連続、no-returnのhard gateは緩和しない。
- `/control/command/control_cmd`など既存ROSインターフェースを変更しない。
- `output/`、result JSON、評価基盤は変更しない。
- 既存のユーザー変更 `aichallenge/result-summary.json` は触らない。

## Definition of Done

- planning traceが`attempt`と`mission_episode`を区別する。
- planner棄却ログに構造化されたgateと棄却sampleが出る。
- primaryのbridge/backoff失効時、sideが確定していれば反対側を1回評価する。
- dynamic Mission waitの候補鮮度・no-return・hard fault・最終actionが1行で追える。
- 単体テスト、対象package build、diff checkが通る。
