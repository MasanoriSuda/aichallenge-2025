# Design

## Pure admission contract

`MpccLitePrefixExecutionRequest` に
`safe_separation_tactical_rearmed` を追加する。

SafeSeparation中のprefix bypassは次をすべて満たす場合だけ有効とする。

- active execution
- before-no-return
- Pass phase
- callerがtactical re-arm済み

それ以外は既存どおり `SafeSeparation` で拒否する。wall、target clearance、
body-clear、minimum speed、残りtime/distance budgetの検査はbypass後も全て通す。

## Planner authority

MPCC shadowの現在側prefixだけに、既存の
`can_rearm_runtime_completion_tactical_replan()` 結果を渡す。反対側prefixには渡さず、
反対側切替はcomplete Missionの既存contractを維持する。

期待ログは次の変化となる。

```text
prefix=1/0/safe-separation, authority=none
  -> prefix=1/1/admitted, authority=replace
```

## Transactional commit

実行層でも同じtactical re-arm条件を再確認し、same-side progressive replacementへ
明示的に権限を渡す。既存のfreeze/rollback処理を使用するため、commit成功時にだけ
Mission generationが更新され、SafeSeparationとruntime pendingがリセットされる。

## Local refactor

`MpccLitePrefixExecutionRequest` の位置依存aggregate初期化3箇所を、明示的な
field assignmentへ変更する。安全flag追加時の引数ずれを防ぐ。
