# Design

## 現象

DP planner の経路は rolling refresh が間に合わないと 0.5 秒で実行権限を失う。一方、40 Hz 制御側には新しい progress-contouring QP 解が存在しても、現状は一時的な `solved_execution_bridge` としてしか使われず、次の execution prefix の正本へ引き継がれない。

## 方針

`SolvedMpccExecutionTrajectory` を Frenet-DP execution prefix へ昇格できる handoff を追加する。

1. active ShiftOut / Pass かつ同一 target・Mission generation・side の新しい solved source を検出する。
2. 既存 rolling refresh interval を満たした場合だけ再検証する。DP authority が既に失効している場合は即時評価を許可する。
3. 現在 horizon へ resample し、hard wall footprint を再検証する。
4. 現在の execution reference に対する lateral trust envelope を適用する。
5. trust 調整後の path をもう一度 hard wall footprint で検証する。
6. 条件を全て満たした場合だけ、path・source timestamp・runtime validation timestamp を atomic に更新する。

## 時刻管理

- planner candidate の `planner_generated_at_sec` と solved QP の `solved_sec` は別フィールドで管理する。
- execution の絶対寿命は実際の `solved_sec` を起点にする。
- last-feasible reuse は source timestamp が新しくない限り promotion せず、古い解の寿命を延ばさない。

## 安全境界

Handoff は既存の target continuity、body separation/contact continuation、prediction、wall、EmergencyBrake、solver recovery、forbidden waypoint guard を通過した場合だけ有効にする。Handoff 後も既存 live receding-horizon target-bound validation を通すため、target-wall conflict を無視しない。
