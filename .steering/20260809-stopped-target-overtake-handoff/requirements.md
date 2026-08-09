# Requirements

## 背景

`output/20260809-232958/d1/autoware.log` では、通常 Overtake が先に停止車を
lock した後に `low_confirm=3/3` となっても `LowSpeedAvoidance` へ移行しない。
通過 corridor が存在する場面でも SafetyBrake、FollowPrepare、Recovery、stuck
recovery が連鎖し、停止車の後方で停滞する。

## 要求

- 通常 Overtake の ShiftOut / Pass 中に同一 target の停止が確定した場合、現在車体が
  非重複で、停止車用の同一 side 経路が成立するなら LowSpeedAvoidance へ引き継ぐ。
- 引継ぎは同一制御周期内で行い、Follow / SafetyBrake / Recovery を経由しない。
- 引継ぎ時に反対 side へ切り返さない。
- 停止車用経路が不成立、target 不連続、または現在車体が重複している場合は、既存の
  通常 Overtake と安全判定を維持する。
- 新規の通常 LowSpeedAvoidance の最小準備距離は変更しない。既に横経路を commit
  している安全な引継ぎに限り、最小準備距離を緩和する。
- ROS 2 topic / service / message 契約は変更しない。

## 制約

- `aichallenge/result-summary.json` の既存変更には触れない。
- `output/` と rosbag は変更しない。
- 加速度、壁余裕、車体寸法などのパラメータは本修正では変更しない。

