# Design

## 方針

`mpc_controller_cpp.cpp`に散在する判断自体は維持し、その出力をMPC問題へ反映
する直前に`OvertakeExecutionAuthorityRequest`へ集める。純粋関数は次を返す。

- `action`: Cruise / Follow / ShiftOut / Pass / Return / DynamicWait / ContactEscape / Recovery / SafetyBrake
- `lateral_owner`: RacingLine / GapPlanner / OvertakeLine / DynamicWaitPrefix / ContactEscape / RecoveryLine / SafetyHold
- `longitudinal_owner`: RacingLine / FollowCap / OvertakeLine / PassFloor / DynamicEscape / SolverFallback / SafetyBrake
- 既存速度reference、limit、floorを適用するフラグ
- 権限競合カテゴリ

安全系、Recovery、接触継続、DynamicMissionWait、通常Missionの順に優先度を
明示する。今回の統合では既存の出力適用条件と同値になるようにし、競合時は
挙動を新しく緩和せずWARNを記録する。

左右branchを採点する仮想`init_problem()`でも同じresolverを使うが、authorityログと
episode観測は通常の制御周期だけに限定する。未採用候補を実行権限として数えない。

## Episode集計

`OvertakeEpisodeAccumulator`を同じ独立モジュールへ置く。

- Idleから追い越し状態へ入った時点で開始
- 各制御周期で速度、回廊幅、横加速度、Mission generation、authorityを観測
- Idleへ戻る直前に終了サマリーを生成

これにより個別ログを手作業で結合しなくても、追い越しがどの権限で終了し、
どこまで性能が劣化したかを比較できる。

## 回廊診断

実行中horizonについて以下をauthorityログとepisode summaryへ渡す。

- target制約込みの最小将来回廊幅とその距離
- 壁だけの最小将来回廊幅
- static/dynamic validity距離
- predicted rear-clear距離

現在だけ広く、その先で閉じる行き止まり候補を、次の性能修正で機械的に識別
できる状態を先に作る。
