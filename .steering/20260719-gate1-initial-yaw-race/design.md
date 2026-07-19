# Design

## Root cause

`output/20260719-151346`では、AWSIMが`Grounded`になった直後の`/set_initial_pose`が
GNSS未到着で失敗した。その後の最初のGNSS callbackはGNSS/IMU orientationをそのまま
`initial_pose3d`へpublishしてEKFをtriggerしたため、reference pathに対して
`e_psi=-2.444 rad`となり、通常MPCが連続してinfeasibleになった。

## Approach

1. heading-referenceの最近傍探索、yaw算出、raceline-aligned initial pose生成をpure coreへ分離する。
2. 初回GNSS callbackは、継続measurement用messageとは別にinitial poseを生成する。
3. initial pose生成に失敗した周期はEKFをtriggerせず、次のGNSS callbackで再試行する。
4. `/set_initial_pose`も同じpure coreを使い、手動経路と自動経路のyawを一致させる。

## Compatibility

- ROS topic/service名とmessage型は変更しない。
- 継続measurement `/localization/imu_gnss_poser/pose_with_covariance` のorientationは変更しない。
- 変更は参加者package `imu_gnss_poser`に閉じる。

