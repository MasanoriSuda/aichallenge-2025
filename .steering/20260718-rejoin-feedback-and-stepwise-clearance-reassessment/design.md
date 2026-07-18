# Rejoin Feedback and Stepwise Clearance Reassessment Design

作成日: 2026-07-18
状態: Completed

## 状態遷移

stepwise後退の`WAIT_REVERSE_REPORT`でcompleteなsnapshotがtimeoutまでblockを示した場合、
`reassess_after_drive_`を設定してから`STOP_BEFORE_DRIVE`へ進む。Drive report後は既存の
`STOP_AND_REASSESS -> CHECK_CLEARANCE`を再利用し、step数とepisode距離を保持したまま
新しい候補を評価する。非stepwise後退にはこのフラグを設定しない。

## LowSpeedRejoin操舵

pure helperで次を計算する。

```text
delta_ff = atan(wheelbase * path_curvature)
delta_target = clamp(
  delta_ff - lateral_gain * e_y - heading_gain * e_psi,
  -max_rejoin_steering,
  +max_rejoin_steering)
```

ROS adapterで前回commandから既存steer-rate上限を適用し、rate-limit後のtire angleを
RecoveryInputへ渡す。同じ値を次の2箇所で使用する。

1. current poseから0.8 mのforward swept-footprint評価
2. `LowSpeedRejoin`の実control command

これにより安全判定とactuationの曲率を一致させる。最初のMPC reset周期は従来どおり停止する。

## LowSpeedRejoin速度

既存実装の`min(normal_mpc_speed, rejoin_speed_limit)`では、物理接触後に通常MPCが
0 m/sを返すと復帰状態のまま静止する。LowSpeedRejoinはescape距離、Drive report、
static swept footprint、V2X、solverの各hard gateを通過済みであり、各周期でもgateを
再確認する。この状態に限り設定値1.0 m/sを専用の前進目標として使用する。
設定値0以下は起動時に拒否する。

## Fail-safe

- 非有限値または不正なwheelbase / limitはhelperで候補なしとする。
- 候補なし、static block、V2X不完全、solver不健康、Drive report喪失では前進しない。
- 最大操舵角は実験値とMPC上限の小さい方に制限する。
- LowSpeedRejoin完了条件、速度設定値、timeoutは変更しない。

## 判定

- Pass: LowSpeedRejoinが5秒以内に`rejoin_complete`し、横偏差とheading誤差が閾値内で0.3秒継続する。
- Safe negative: 新操舵のstatic rolloutがblockを検出し、前進せず再評価またはSafeStopする。
- Fail: static判定と異なる操舵を出す、情報不完全時に前進する、発散する、または既存距離上限を超える。

## 1回目runからの設計補正

`output/20260718-172154`のD2は2.005 mのescape後にLowSpeedRejoinへ入ったが、
`e_y=2.305 -> 2.263 m`のまま5秒でtimeoutした。操舵は-0.35 radまで出ていた一方、
速度調停が通常MPCの0 m/sを採用したため、専用速度目標を追加して再実験する。
