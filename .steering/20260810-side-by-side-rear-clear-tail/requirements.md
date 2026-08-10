# Requirements

## 背景

`output/20260810-093645/d1/autoware.log` の4回目の追い越しでは、
`SideBySideCommitted` 後に対象車が自車より約0.58 m後方となり、現在車体、予測sweep、
実行corridorはいずれもclearだった。それにもかかわらず、残りPass予算内の
rear-clear rolloutが不成立となったことを `short horizon unsafe` と扱い、
SafeSeparationからRecoveryへ移行した。

## 要求

- `SideBySideCommitted` かつ対象車が横並び以下まで後退し、現在車体、予測sweep、
  target continuity、実行corridorが物理的に成立する場合は、完遂rollout／通常local budget
  の不成立だけでPassを中断しない。
- 上記継続中は同じsideと横goalを保持し、rear-clearまで前進速度を維持する。
- physical wall contact、wall sample loss、confirmed body overlap、target discontinuity、
  pass-side intrusion、EmergencyBrake、solver recoveryは従来どおりhard faultとする。
- absolute Pass time／distance limitは上記継続の上限として維持する。
- `Selectable` / `ShiftCommitted` と、対象車がまだ前方にいる状態の挙動は変更しない。
- ROS 2 topic / service / message、launch、yaml parameterの契約は変更しない。

## 非対象

- LowSpeedDirectのcommit stageとhandoff指令連続性
- planner横断episode budget
- opponent multi-hypothesis prediction
- wall clearanceや速度parameterの変更
