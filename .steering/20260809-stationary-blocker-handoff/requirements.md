# Requirements

## Goal

`output/20260809-191928` で確認した、停止車の前に空きがあるにもかかわらず
Overtakeへ引き継がず停止・Reverseを繰り返す事象を解消する。

## Required behavior

- 最初のShiftOut/Pass/FollowPrepareでMission総時間の開始時刻を必ず確定し、
  `elapsed=NaN`による即時Recoveryを発生させない。
- 停止車が連続観測で確認済みで、完全なShiftOut/Pass/Return Missionが現周期に
  成立している場合は、相対速度の0.3秒確認待ちを省略してOvertakeへ引き継ぐ。
- 壁、車体、body-clear deadline、rear-clear rollout、前方3 m、EmergencyBrake、
  V2X連続性の判定は従来どおり維持する。

## Non-goals

- `v2x_overtake_guard_min_front_distance`を緩和しない。
- 壁・車体マージンやEmergencyBrakeを無効化しない。
- ROS 2 topic、message、launch、評価インターフェースを変更しない。

