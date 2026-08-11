# Requirements

## 目的

追い越し実行中の一時的なSafetyBrakeで`FollowPrepare`へ入った後、既に確保した
target・side・frozen pathを捨ててFollowし続ける負のループを解消する。

## 必須要件

- SafetyBrake由来pauseと、dynamic replan / Recovery由来pauseを区別する
- `ShiftOut`または`Pass`を中断したSafetyBrake pauseだけを早期再開対象にする
- 緊急条件が解消し、target continuityとfrozen pathが有効ならentry距離判定を
  再適用せず、同じsideの実行を再開する
- lateral clearance成立済みなら`Pass`、未成立なら`ShiftOut`へ戻す
- wall、target jump、course-progress discontinuity、side intrusion、forbidden waypoint、
  solver recovery、Mission invalidationは早期再開を禁止する

## 制約

- 速度、車間、wall clearanceなどのパラメータ値は変更しない
- 反対側への直接切替は行わない
- Recovery完了やdynamic Mission waitの既存再開規則は変更しない
- ROS 2インターフェースと評価基盤は変更しない

## 完了条件

- SafetyBrake解除後、同じMissionを`ShiftOut`または`Pass`として再開できる
- full lateral clearance未成立だけを理由に4秒間FollowPrepareへ滞在しない
- hard fault時は従来どおりHold/Recoveryとなる
- pure coreの境界テストと対象package全テストが成功する
