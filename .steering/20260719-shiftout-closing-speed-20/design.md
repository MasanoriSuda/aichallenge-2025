# Design

## A/B Variable

```yaml
v2x_overtake_shiftout_max_closing_speed: 2.0
```

前回値は1.5 m/s。ShiftOut中だけ前走車速度へ加算するsoft上限を0.5 m/s緩和する。横移動距離は検証済みの8.0 mを維持し、横位置が整う前にPassへ入った6.0 m A/Bの失敗を避ける。

## Fixed Conditions

- `v2x_overtake_line_shift_distance: 8.0`
- `v2x_overtake_stage_speed_enabled: true`
- `v2x_overtake_active_hard_curve_completion_enabled: true`
- active hard-boundary rear-clear/buffer: 0.5 m / 0.5 m
- lateral acceleration、gap reachability、wall clearance
- front risk、SafetyBrake、EmergencyBrake、curve cooldown

## Measurements

- D2のShiftOut開始WP、Pass開始WP、front distance、横位置
- WP150のfront distanceとhard available/required distance
- hard中断位置または追い越し完了
- lap count、lap time、順位
- solver failure、衝突、全車停止、Reverseの有無

