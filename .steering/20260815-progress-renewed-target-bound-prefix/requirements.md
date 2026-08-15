# Requirements

## Objective

`optimized horizon escaped target separation bounds` が短時間続いた際、物理的に
成立している同側 Pass prefix と前進速度を固定 1.5 s / 8 m で捨てず、実測の
追い越し進捗が続く間だけ再計画待ちを延長する。

## Constraints

- 壁接触、壁余裕不足、EmergencyBrake、target jump、回復不能な車体重複は
  従来どおり hard fault とする。
- 延長は Pass phase のみとし、未完了 ShiftOut を延命しない。
- 進捗が止まった場合は延長しない。
- Pass の immutable absolute time / distance budget は超えない。
- ROS 2 topic / service / message 契約を変更しない。
- `aichallenge/result-summary.json` のユーザー変更は変更・commitしない。
