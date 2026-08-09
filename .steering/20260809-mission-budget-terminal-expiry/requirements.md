# Requirements

## Purpose

`v2x_overtake_mission_total_time_limit_sec` 到達後に、期限切れの追い越しMissionが
`Recovery -> FollowPrepare -> Recovery` と再開され続ける不具合を止める。

## Scope

- Mission総時間budgetによるAbortを、Mission保持不可の終端Abortとして記録する。
- 横位置を戻すためのRecoveryは維持する。
- Recovery完了後は期限切れMissionを破棄してIdleへ戻す。
- 通常の壁余裕・短期不成立Recoveryでは既存のMission保持方針を変えない。

## Constraints

- ROS 2 topic/service/message契約を変更しない。
- 追い越しパラメータおよび15秒の上限値は変更しない。
- maneuver再ランキングなど別の性能変更を混ぜない。

