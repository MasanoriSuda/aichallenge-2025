# Design

## 現行

Side/Mixed/Noneの壁分類では、現在footprintがclearでもstepwise modeを選択する。
1 stepの判定距離は0.4 mで、停止距離reserveを含めて到達すると次を反復する。

```text
ReverseManeuver
-> StopBeforeDrive
-> ShiftToDrive
-> StopAndReassess
-> CheckClearance
-> ShiftToReverse
```

## 変更

### Fast continuous Reverse

`fast_continuous_reverse_enabled` が有効かつ現在footprintがclearの場合、
stepwise候補を使わず、残りの通常escape距離に対するrolloutを評価する。

rolloutが静的壁とV2X予測の両方を通過した場合は、primitiveとsteeringをロックして
ReverseManeuverを連続実行する。途中で安全性が失われた場合は既存の停止遷移を使う。

現在footprintがclearでない場合は、接触改善を確認する既存stepwiseを維持する。
最初の短いstepでclearになった後の再判定からcontinuousへ切り替えられる。

### Escape target

通常目標は既存の `reverse_escape_distance_m=2.0` とする。

以下をすべて満たす場合のみ、目標を `fast_rejoin_min_reverse_distance_m` まで短縮する。

- fast continuous modeが有効
- 現在footprintがclear
- rate-limit後のsteeringによる前進rejoin rolloutがclear

### Braking

`実移動距離 + stopping reserve >= 現在のescape目標` になったらReverse gearを保持したまま
制動指令を出す。実距離がescape確認閾値へ達したら既存のStopBeforeDriveへ移行する。
停止不足時はReverseManeuver内で再加速できるため、Drive/Reverse反復は発生させない。

## 設定

- `fast_continuous_reverse_enabled: true`
- `fast_rejoin_min_reverse_distance_m: 0.8`
- `max_reverse_speed_mps: 2.0`
- `reverse_acceleration_magnitude_mps2: 1.0`
- `reverse_stop_acceleration_mps2: -1.0`
- `verified_reverse_stop_deceleration_mps2: 0.6`

`0.6 m/s^2` は既存コメントに記録されたAWSIM実測平均 `0.628 m/s^2` を超えない値として使う。
実走ログでは停止距離とovershootを再確認する。

## 互換性

- ROS topic/service/message: 変更なし
- 評価JSON: 変更なし
- simulation-only guard: 維持
- 機能無効時: 従来のstepwise動作
