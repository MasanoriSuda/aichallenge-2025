# Tasklist

- [x] `20260815-203758` の長時間wait経路を特定
- [x] retention contractへauthority/progress条件を追加
- [x] controllerへprogress checkpointを接続
- [x] request生成を局所リファクタ
- [x] unit testを追加
- [x] build/package testを実行
- [x] ユーザー変更を除外してコミット

## Definition of Done

- prefixがactiveでも進捗またはclosing authorityがなければ短いleaseでfresh searchへ戻る。
- recent progressとfull closing authorityがあればrear-clearまで継続できる。
- runtime検証済みcontinuous DPも同じ進捗条件で継続できる。
- terminal waitと既存hard fault contractを維持する。

## Validation

- `make autoware-build`: 成功（25 packages）
- 対象GoogleTest 3件: 成功
  - `RetainsHealthyExpiredPausedMissionUntilRearClear`
  - `RetainsDynamicWaitOnlyForCommittedForwardExecution`
  - `TerminalBudgetRequiresRearmOrFreshSearch`
- `ctest --test-dir /aichallenge/workspace/build/multi_purpose_mpc_ros`: 25/25成功
- 動的効果確認: ユーザー試走待ち
