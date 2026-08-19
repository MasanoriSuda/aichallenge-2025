# Design

## 原因

従来実装は `dynamic_obstacle_lateral_escape_quarantine_until_sec_` を1個だけ持っていた。
そのため次の二つが同時に起きた。

1. 0.5秒後に同じsolver-infeasible corridorを再試行する。
2. quarantine中は左右両方のGapPlanner要求を停止する。

reachability bridgeは横加速度・到達包絡を確認するが、QP全体の数値的可解性まで
保証しない。このためbridge validでも最初のtracking solveが失敗する候補は残る。

## 状態管理

`DynamicObstacleLateralEscapeSolverBackoff` をcoreへ追加し、次を候補単位で保持する。

- target ID
- pass side
- consecutive failures
- last failure time
- blocked-until time

失敗holdはbase 0.5秒を倍増し、4.0秒で飽和する。tracking solve成功時は同じ
target/sideだけを消去する。失敗間隔が8.0秒を超えた場合も履歴を初期化する。

## 候補選択

Follow状態の動的障害物GapPlannerはbackoff中も実行する。最初に選ばれたsideが
backoff中なら、GapPlannerのtarget lockを解除し、反対sideを強制して1回だけ再評価する。

- 反対sideがbridge validかつbackoff外: 反対sideを採用
- 反対sideが不成立またはbackoff中: lateral authorityを不許可

不許可時は既存のpost-authority scrubによりscoped planner出力を破棄し、通常Followの
縦速度所有権を維持する。

## ログ

authorityログへ次を出す。

- `backoff=<blocked side>/<failure count>/<remaining sec>`
- `alternate=<attempted>/<selected>`

solver failureログは候補のfailure countと今回のhold秒数を出す。

## 設定

- `v2x_dynamic_obstacle_lateral_escape_solver_quarantine_sec`: 初回hold（互換名を維持）
- `v2x_dynamic_obstacle_lateral_escape_solver_backoff_max_sec`: 最大hold
- `v2x_dynamic_obstacle_lateral_escape_solver_backoff_reset_sec`: 履歴reset時間
