# Design

## Observed failure

最新実走ではMission総時間budget到達後、次を反復した。

1. `FollowPrepare -> Recovery`: Mission total budget expired
2. `Recovery -> FollowPrepare`: committed pass mission retained
3. 期限切れ判定が再度成立

Recovery中はMission総時間budgetの対象phaseではないため、Recovery完了時の保持判定が
期限切れを認識できないことが原因である。

## Change

`OvertakeLineState` にMission保持禁止フラグを追加する。Mission総時間budgetのAbortで
Recoveryへ入る直前に設定し、RecoveryのMission保持判定へ渡す。

- フラグ未設定: 従来どおり、通常Recovery完了後に同じMissionを保持可能
- フラグ設定: Recovery完了後は必ずMissionを破棄してIdleへ戻る

フラグは`reset_overtake_line_state()`により新しいMission開始前に初期化される。

## Verification

- core単体テストで通常Recoveryは保持できることを維持する。
- 同じ条件でもMission保持禁止時は保持されないことを追加する。
- 対象packageのbuild/testを実行する。
- 次回実走でbudget expiry 1回につきRecovery entryが1回以下であることを確認する。

