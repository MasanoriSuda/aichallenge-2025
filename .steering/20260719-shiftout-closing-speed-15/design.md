# Design

## A/B Variable

```yaml
v2x_overtake_shiftout_max_closing_speed: 1.5
```

前回値は1.0 m/s。ShiftOut中に限って前走車速度に加算するsoft速度上限を0.5 m/s緩和する。Passへ移行した後は従来どおり前走車由来の上限を解除する。

## Fixed Conditions

- `v2x_overtake_stage_speed_enabled: true`
- `v2x_overtake_active_hard_curve_completion_enabled: true`
- active hard-boundary rear-clear/buffer: 0.5 m / 0.5 m
- front risk、SafetyBrake、EmergencyBrake
- gap reachability、wall clearance、curve cooldown

## Measurements

- D2の追い越し開始WPとfront distance
- ShiftOutからPassまでの時間・走行距離・front distance
- hard境界でのavailable/required distance
- lap count、lap time、順位
- solver failure、衝突、全車停止、Reverseの有無

