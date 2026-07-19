# Design

## A/B Variable

```yaml
v2x_overtake_line_shift_distance: 6.0
```

前回値は8.0 m。OvertakeLineの横移動blend距離を2.0 m短縮し、ShiftOutからPassへの遷移を早める。Passでは前走車由来の速度上限が解除されるため、hard境界までに縦距離を詰める時間を増やす。

## Fixed Conditions

- `v2x_overtake_shiftout_max_closing_speed: 1.5`
- `v2x_overtake_active_hard_curve_completion_enabled: true`
- active hard-boundary rear-clear/buffer: 0.5 m / 0.5 m
- lateral acceleration、gap reachability、wall clearance
- front risk、SafetyBrake、EmergencyBrake、curve cooldown

## Measurements

- D2のShiftOut開始WP、Pass開始WP、遷移時間、走行距離
- Pass開始時とWP150のfront distance
- hard境界available/required distanceと中断WP
- lap count、lap time、順位
- solver failure、衝突、全車停止、Reverseの有無

