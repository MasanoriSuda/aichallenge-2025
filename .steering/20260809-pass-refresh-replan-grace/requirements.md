# Requirements

## 背景

`output/20260809-220122/d1`では4回すべて`ShiftOut -> Pass`へ入ったが、将来のPass経路更新失敗を契機にSafeSeparationへ移り、`Pass -> Return`は0回だった。Pass中は速度指令11.11 m/s、加速度指令+1.0 m/s^2であり、失速はSafeSeparation移行後の最大-3.0 m/s^2指令によって発生した。

## 目的

現在のコミット済み短時間経路が安全な間は、将来経路の更新失敗だけでPassを中断せず、同じMissionとsideを維持して再計画を試行する。

## 必須要件

- 将来経路の更新失敗が再計画可能な種類の場合だけgraceを許可する。
- grace中も通常Passの前進速度所有権を維持し、Follow速度へ落とさない。
- 車体重複、予測sweep重複、壁接触・壁余裕違反、EmergencyBrake、solver recovery、target不連続ではgraceを許可しない。
- コミット済みstatic horizonとMission絶対時間・距離を越えない。
- graceは既存の`pass_horizon_hold_max_sec`と`pass_horizon_hold_max_distance`で制限する。
- 再計画成功時はgraceを解除し、期限切れ時は従来のSafeSeparationへ移る。

## 制約

- ROS 2 topic、message、launch、評価インターフェースを変更しない。
- `aichallenge_system`を変更しない。
- `aichallenge/result-summary.json`のユーザー変更に触れない。

