# Design

## Current issue

現行のfront-cap解除・再適用判定は、対象車との共通コース横離隔を主に使用する。
最新ログでは対象車が縦方向に3.4〜8.6 m離れていてbody footprintが非重複でも、
横離隔が1.43〜1.45 mへ低下した時点でcapが再適用された。

minimum-motionはtarget車両をinflateした予測corridorから最小横目標を選ぶため、
実行側が固定1.50 mだけを再度要求すると計画と速度制御が一致しない。

## Footprint sweep

対象車との相対位置を共通コース座標の`(s, lateral)`で扱い、現在点から
V2X予測時刻の点まで線形に補間する。この線分が次のbody矩形内部を通る場合を
予測重複とする。

- longitudinal half extent:
  `0.5 * (ego length + target length)`
- lateral half extent:
  `v2x_vehicle_radius`

境界への接触だけは非重複とし、矩形内部との交差を重複とする。入力欠損時は
解除せず、従来capを維持する。

## Front-cap policy

minimum-motion corridorを保持したPassでは、次をすべて満たすと初回解除する。

- lateral goal complete
- target seen / no position jump
- wall execution path physically feasible / no actual wall contact
- current body footprints separated
- predicted footprint sweep separated

解除後はlateral goalやwall clampの一時的な制約ではなく、target continuity、
actual wall contact、current/predicted body overlapで保持可否を決める。

minimum-motion以外は既存の1.50 m release / 1.45 m reapply policyを維持する。

## Diagnostics

front-cap遷移ログへ以下を追加する。

- minimum-motion footprint policy active
- current footprint clear
- predicted sweep clear / prediction valid
- current and predicted relative longitudinal/lateral
- policy transition reason

## Verification

- 縦方向に十分離れた1.43 m横離隔で解除・保持できる。
- closing trajectoryがbody矩形へ入る場合は解除しない／再適用する。
- prediction欠損、position jump、wall contactでは解除しない。
- 非minimum-motion policyの既存テストを維持する。
