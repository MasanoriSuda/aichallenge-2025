# Requirements

## 目的

追い越し中の参照経路更新を、失敗後の救済処理ではなく正常な
`ShiftOut` / `Pass` 実行の主経路にする。

## 背景

20260815-221525 の走行では、DP rolling refresh は4回に留まり、その多くが
`FollowPrepare(DynamicMissionWait)` 移行後だった。固定Missionがtarget境界で破綻してから
再計画するため、抜き始めてから引く挙動が残っている。

## 要求

- 同一target・同一sideの `ShiftOut` / `Pass` 中も、MPCC-lite/DP候補を周期評価する。
- targetが選択sideへ移動したことだけでは候補生成を止めず、最新target制約を含む
  current-state prefixを評価する。
- 新prefixが制御horizon全体を覆わない場合、直前のfeasible参照をtailとして用い、
  結合後のhorizon全体をwall・横加速度制約で再検証する。
- 採用はatomicに行い、候補不成立時は直前のfeasible経路を保持する。
- wall接触、緊急制動、target断絶、solver recovery等のhard faultでは実行権を失効する。
- 既存ROS topic/service、提出物、評価schemaは変更しない。

## 非目標

- 非線形車両モデルを含む全面的なgMPCCへの置換。
- no-return後の無条件な左右切替。
- wall/車体hard constraintの緩和。
