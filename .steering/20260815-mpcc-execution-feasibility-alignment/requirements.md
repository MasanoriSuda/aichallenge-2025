# Requirements

## 目的

2026-08-15 の `make dev2` 走行で、曲線対応Frenet DPによる
`ShiftOut -> Pass` は増えた一方、Pass実行時の物理wall/target gateでMissionが
中断される不整合を解消する。

## 観測事実

- `Idle -> ShiftOut`: 20回
- `ShiftOut -> Pass`: 18回
- `Pass -> Return`: 1回
- `Pass -> Recovery`: 12回
- 主な失敗理由:
  - `physical target separation conflicts with wall bounds`
  - `Pass entry physical wall gate unresolved`
  - SafeSeparation再検証失敗
- rolling DPで `sweep_dive -> inside_dive -> sweep_dive` の短時間切替がある。
- Recovery完了後に保持した同一Missionがruntime hard faultで再びRecoveryへ戻る例がある。

## 要求

1. DPが使うhard corridorを、実行時の物理target separationとhard wall clearanceに一致させる。
2. robust target/wall clearanceはhard rejectionにせず、DPのsoft preferenceとして扱う。
3. 前回戦術がまだfeasibleなら、小さな評価差だけで戦術を切り替えない。
4. RecoveryRetention直後に同じhard faultを検出した場合、同じMissionでRecoveryを反復しない。
5. ROS 2 topic、message、service、launch、提出インターフェースは変更しない。
6. `aichallenge/result-summary.json` の既存変更には触れない。

## Definition of Done

- core unit testでhard/soft corridorと戦術ヒステリシスを確認できる。
- RecoveryRetention hard faultの終端方針がテストまたは純粋関数で確認できる。
- 対象packageのbuild/testが通る。
- 変更を1コミットにまとめる。
