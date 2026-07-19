# Requirements

## Purpose

`make gate1`でGNSS到着前に`/set_initial_pose`が呼ばれた場合でも、最初の
`/localization/initial_pose3d`をreference headingへ整合させ、誤yawによるMPCの連続
solver failureと発車不能を防ぐ。

## Scope

- `imu_gnss_poser`の初回自動初期姿勢生成
- heading-referenceからyawを求めるpure coreと単体テスト
- `/set_initial_pose`の名前・型・応答契約は維持する
- 継続GNSS measurementは従来どおりGNSS/IMU orientationを使う
- `aichallenge_system`とAWSIMの開始FSMは変更しない

## Acceptance criteria

- 初回自動`initial_pose3d`と`/set_initial_pose`が同じheading計算を使う
- headingを計算できない場合は誤yawでEKFをtriggerしない
- `make autoware-build`が成功する
- pure coreの単体テストが成功する
- `make gate1`で開始直後の`e_psi`が小さく、OSQP連続失敗なしに正方向へ発車する

