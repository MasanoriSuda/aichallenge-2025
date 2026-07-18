# Rejoin Feedback and Stepwise Clearance Reassessment Requirements

作成日: 2026-07-18
状態: Completed

## 背景

`output/20260718-165739`では、改善中のstepwise後退がstatic候補の
`contact_worsened`化で`WAIT_REVERSE_REPORT`へ移り、停止してDriveへ戻った後に
`escape_not_confirmed`でSafeStopした。候補を再選択するための
`reassess_after_drive_`がこの遷移では設定されていない。

`output/20260718-170218`のD2は10 step・2.174 mでescapeを確認したが、
LowSpeedRejoin中の横偏差が1.207 mから0.915 mまでしか縮まず、5秒から10秒へ
猶予を延長しても`rejoin_timed_out`となった。

## 要求

1. stepwise後退中の完全なstatic/V2X snapshotが持続的な後方blockを示した場合、まず停止してDriveを確認し、その後`STOP_AND_REASSESS`から候補を再選択する。
2. 情報欠落時は従来どおりReverseで停止保持し、timeoutを回復完了やblockの証拠に使わない。
3. 非stepwise後退の持続的blockは従来どおりDriveへ戻した後にSafeStopする。
4. LowSpeedRejoinに参照曲率feedforward、横偏差、heading誤差の低速専用操舵feedbackを追加する。
5. 実際に出すrate-limit後の操舵角で既存0.8 m swept footprintを評価し、clearでない周期は前進させない。
6. hard gate通過後のLowSpeedRejoinでは、通常MPCが接触後に0 m/sを返しても専用の速度目標1.0 m/sを使用する。
7. 速度1.0 m/s、timeout 5.0秒、許容横偏差0.5 m、許容heading誤差0.35 radを緩和しない。
8. static、V2X、solver、Boost、gear、最大後退距離・step回数のhard gateを維持する。
9. ROS topic、service、message、Domain、評価JSON契約を変更しない。

## 実験値

- 横偏差gain: 0.60 rad/m
- heading誤差gain: 1.20
- 復帰最大操舵角: 0.35 rad

いずれも2025 AWSIM final_ver3向けのローカル実験値であり、2026公式値・実車値ではない。

## Definition of Done

- pure C++単体テストで操舵符号、曲率feedforward、limit、非有限値拒否を確認する。
- stepwise持続blockがDrive確認後にSafeStopせず`CHECK_CLEARANCE`へ戻るテストを追加する。
- 非stepwise持続blockのSafeStopを回帰テストで維持する。
- `make autoware-build`と対象package testが成功する。
- `make dev3`でLowSpeedRejoinが5秒以内に完了するか、hard gateにより安全停止する。
- 実操舵とstatic rollout操舵が同じ値であることをログで確認する。
- LowSpeedRejoin中に正の専用速度目標と実速度をログで確認する。
