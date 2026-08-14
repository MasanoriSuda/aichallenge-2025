# Requirements

## Goal

直近走行 `output/20260814-084203` で確認した、Pass中断後に
`FollowPrepare` へ移行したまま新しい実行候補を生成できず、追従速度へ
失速する事象を解消する。

## Scope

- `DynamicMissionWait` に限定されているMPCC-lite rolling replanを、
  `TacticalRevalidation` と `RecoveryRetention` にも適用する。
- no-return後も、現在のpass sideについては現在位置から再評価する。
- staleな凍結Missionを新しいcurrent-state prefixより優先しない。
- 明確な壁接触、EmergencyBrake、solver recovery、禁止区間では従来どおり
  rolling replanを許可しない。

## Non-goals

- no-return後の反対側への全幅切り返しを許可しない。
- SafetyBrake pauseの復帰契約を変更しない。
- 壁、車体、予測footprintのhard guardを緩和しない。
- ROS 2 topic/service/interfaceを変更しない。
