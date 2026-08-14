# Design

## Observed failure

直近走行では5回のPassに対してReturnは0回で、主な離脱先は
`FollowPrepare` だった。`DynamicMissionWait` では速度保持が働く一方、
`TacticalRevalidation` と `RecoveryRetention` は通常のFollowPrepareとして
扱われ、MPCC-lite候補が`PlanningUnavailable`のままタイムアウトした。

また、no-return latch後のshadow評価は左右比較を止める実装だが、同じ条件で
現在sideの再評価まで止まっていた。このため、安全上禁止すべき反対側切替だけでなく、
許可すべきsame-side continuationも生成されなかった。

## Change

1. `DynamicMissionWait`、`TacticalRevalidation`、`RecoveryRetention`を
   tactical rolling replan causeとして一元判定する。
2. tactical rolling中は現在sideをno-return後もshadow評価する。
3. 通常のentry gap判定が不成立でも、target continuity、現在車体非重複、
   prediction有効、非Emergencyなどを満たすsame-sideだけをcurrent-state
   prefix preflightへ渡す。
4. この例外は候補採用ではなく候補生成の許可に留める。既存の壁、target sweep、
   横加速度、body-clear、速度、時間・距離budgetのpreflightは維持する。
5. rolling中は旧Missionのhold candidateをshadow rankingから外し、
   新しいprefixがhard-feasibleな場合にsame-side replacementを選ばせる。
6. 新候補がない間は現在横位置と速度を短時間保持し、既存の短い
   dynamic wait timeout/distanceで打ち切る。

## Safety boundary

- SafetyBrake causeは対象外。
- pass-side intrusion、target discontinuity、現在車体重複、壁異常、
  EmergencyBrake、solver recovery、禁止waypointではcurrent-state prefixを作らない。
- cross-sideは既存no-return gateを維持する。
