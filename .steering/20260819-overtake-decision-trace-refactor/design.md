# Design

## Approach

制御ロジックから独立した`overtake_decision_trace`モジュールを追加する。ノード固有の
`GapPlannerOutput`やROS loggerをモジュールへ持ち込まず、採否結果だけを正規化した
DTOへ変換する。

```text
Gap Planner primary ─┐
bridge / backoff     ├─ CandidateTrace(primary)
                     │
forced opposite ─────┤─ CandidateTrace(alternate)
                     │
authority resolver ──┴─ OvertakeDecisionTrace
                              │
                       change-aware formatter
                              │
                Overtake decision trace: ...
```

## Components

### `overtake_decision_trace.hpp/.cpp`

- candidate dispositionを次に正規化する。
  - `not-evaluated`
  - `planner-inactive`
  - `planner-rejected`
  - `bridge-rejected`
  - `backed-off`
  - `ready`
- primary / alternateのrequested side、resolved side、理由、backoff回数を保持する。
- 最終outcomeを`active-primary`、`active-alternate`、`alternate-rejected`、
  `authority-rejected`などへ分類する。
- categorical signatureが変化した場合、または一定時間経過した場合だけ出力する。
  残りbackoff秒、corridor幅、waypointなどの連続値はsignatureへ含めない。

### authority rejection reason

`resolve_dynamic_obstacle_lateral_escape_authority()`の戻り値へ拒否理由enumを追加する。
既存の早期return順序を保ち、採否結果は変えない。

### controller integration

- primaryとalternateを上書き前に個別記録する。
- 既存の1秒周期authorityログをDecision Traceへ置換する。
- tracking失敗時は`stage=tracking outcome=failed`、失敗履歴を解消した最初の成功時は
  `stage=tracking outcome=recovered`を出す。

## Compatibility

- 制御出力とROS interfaceは不変。
- 変更は参加者package内に閉じる。
- 従来の`OvertakeLine`遷移ログは維持する。
