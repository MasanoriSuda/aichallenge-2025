# Requirements

## 目的

`output/20260809-194105` で確認した、停止した追い越し対象と横並びの
`Pass` 中に短い分類欠損が起き、直前まで成立していた同側Missionを即座に
`Recovery` へ落とす事象を抑止する。

## 対象事象

- `Pass`、target=d2、target速度は停止閾値以下
- 現在車体は非重複
- 直前の予測footprint sweepは非重複
- 壁接触・壁余裕違反・solver recoveryはない
- その後 `fresh target prediction unavailable` だけでMission generationを破棄

## 制約

- 壁、現在車体重複、EmergencyBrake、solver failureは緩和しない。
- 保持は既存のtarget hold時間とPass horizon hold距離以内に限定する。
- Reverse等の外部復帰後に古い凍結経路を再利用しない。復帰後は再計画する。
- ROS 2 topic、message、service契約は変更しない。

## 完了条件

- 停止・横並び・直前予測clearの条件だけで短時間のprediction leaseが成立する。
- 観測期限、距離期限、壁、重複、Emergency、solver recoveryで必ず不成立になる。
- 既存テストと追加単体テストが通り、`make autoware-build` が成功する。
