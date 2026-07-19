# Run Data Analysis

## Summary

- valid run: `output/20260719-202354`
- 全車1周を記録。全車停止、collision、Reverse、stuck判定はない。
- 距離適応は1.50、1.79、2.00 m/sを実際に選択した。
- 固定1.5 m/s比でD2は0.590秒短縮したが、D2の`Overtake -> SafetyBrake`が0回から2回へ増えたため不採用とした。

| Domain | 固定1.5 m/s (`20260719-194740`) | 距離適応 (`20260719-202354`) | 差 |
|---|---:|---:|---:|
| D1 | 134.245377 s | 132.403580 s | -1.841797 s |
| D2 | 128.519791 s | 127.929329 s | -0.590462 s |
| D3 | 122.962059 s | 122.937073 s | -0.024986 s |

## Evidence

- D2 WP131: front distance 7.00 mで1.50 m/sを選択。
- D2 WP135: front distance 6.34 m、残り時間0.75 sで1.79 m/sを選択。
- D2 WP130（次周）: front distance 12.95 mで2.00 m/sを選択。
- D2 WP137: `e_y=0.73 m`でPassへ入り、WP144のfront distance 3.36 mでSafetyBrake。
- D2 WP138（次周）: `e_y=-0.01 m`でPassへ入り、WP157のfront distance 3.73 mでSafetyBrake。
- D2の全SafetyBrake遷移は固定値7回、適応値8回。追い越し起因は固定値0回、適応値2回。
- OSQP failureは固定値31件、適応値34件で同程度。適応機能固有の停止は確認できない。

## Topic Chain

```text
/v2x/vehicle_positions
  -> front distance / front speed
  -> adaptive ShiftOut closing cap (1.5..2.0 m/s)
  -> OvertakeLine distance-based ShiftOut completion
  -> Pass releases front-speed cap before lateral completion
  -> MPC accelerates toward trajectory speed
  -> front risk / SafetyBrake
  -> /control/command/control_cmd
```

## Suspected Causes

1. ShiftOutからPassへの遷移が、横位置未到達でも走行距離8 mだけで成立する。
2. 速度stageがOvertakeLine phaseへ直結しているため、Pass遷移と同時に前走車速度上限を完全解除する。
3. 距離適応はShiftOut中には有効だが、Pass解放後の急な接近を制御できない。

## Next Checks

- OvertakeLineがPassでも、`abs(current_ey - pass_target_ey)`が許容値内へ入るまで速度stageをShiftOutとして維持する。
- speed-cap解放条件と横line phaseを分離した純粋関数を追加する。
- 同じ1.5〜2.0 m/s適応設定で再実験し、D2の`Overtake -> SafetyBrake`を0回へ戻せるか確認する。
