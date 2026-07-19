# Run Data Analysis

## Summary

- valid run: `output/20260719-200646`
- A/B変数: `v2x_overtake_shiftout_max_closing_speed` 1.5 -> 2.0 m/s
- D1/D2/D3は全車1周し、停止時点で2周目を走行中だった
- 全車停止、衝突、Reverse実行は再現しなかった
- D2の1周目WP130追い越しはfront distance 3.29 mまで接近しWP145でSafetyBrakeへ入った
- D2の2周目追い越しはWP152でhard境界中断となり、1.5 m/s runのWP154より早く終了した
- D2 lapとD2-D3差が悪化したため2.0 m/sは不採用とし、現行configは1.5 m/sへ戻す

## Evidence

### D2 target pass

| 指標 | 1.5 m/s run | 2.0 m/s run |
|---|---:|---:|
| 追い越し開始 | WP130 / 14.08 m | WP130 / 14.94 m |
| Pass開始 | WP137 / ey=-0.06 m | WP137 / ey=-0.10 m |
| hard継続観測 | WP150 / fd=8.27 m | WP148 / fd=9.80 m |
| hard中断 | WP154 | WP152 |
| 中断時余裕 | 12.39 - 12.73 = -0.34 m | 14.39 - 14.85 = -0.46 m |

2.0 m/sはWP132でfront speed 2.81 m/sに対して`desired_v=4.81 m/s`となり、設定値が
実際に使われた。横移動距離8.0 mによりPass開始時横位置は`ey=-0.10 m`まで進み、6.0 m A/Bの
横移動途中遷移は再発しなかった。しかしrun開始時のfront distance差を含めても、hard境界で
抜き切るだけの改善は得られなかった。

同じrunの1周目ではWP130をfront distance 6.91 mで開始し、WP137でPassへ入った後、
WP145でfront distance 3.29 mとなって`inside stopping distance`のSafetyBrakeへ移行した。
固定2.0 m/sは開始距離が短いケースで近距離リスクを増やす。

### Lap result

| 車両 | 1.5 m/s run | 2.0 m/s run | 差 |
|---|---:|---:|---:|
| D3 | 122.962 s | 123.267 s | +0.305 s |
| D2 | 128.520 s | 129.155 s | +0.636 s |
| D1 | 134.245 s | 131.162 s | -3.083 s |

D2-D3差は5.558 sから5.889 sへ0.331 s悪化した。D1の短縮は対象パラメータの直接効果ではなく、
車間相互作用またはrun間変動と判断する。

### Solver and recovery

- D2の連続maximum-iterationsはスタート待機中の速度0 m/sで発生し、発車後に復帰した
- D2は1周目WP44の追い越しで短いsolver failure Recoveryを記録した
- D2の2周目対象区間WP130〜152ではsolver failureは0だった
- Reverse状態、衝突ログ、stuck確定は記録されなかった

## Topic Chain

- V2X: D2はD3をlocked targetとして継続検出
- localization/reference progress: WP130 -> WP152 -> Recovery -> WP277まで連続進行
- behavior: Follow -> Overtake -> Follow、および別周でOvertake -> SafetyBrakeを記録
- OvertakeLine/MPC: ShiftOut -> Pass -> Recoveryを実行
- control: 3台とも1周後も制御を継続

このrunにはrosbagが保存されていないため、topic rateと実command波形はAutowareログから分かる
範囲だけを評価した。

## Suspected Causes

1. ShiftOutは約2.3 sのため、上限を0.5 m/s上げても得られる縦距離改善は小さい。
2. 追い越し開始時front distanceが周回ごとに6.91〜14.94 mと大きく異なり、固定2.0 m/sは近距離ケースで過剰になる。
3. 遠距離ケースでもhard境界までに必要な加速時間が足りず、固定上限の緩和だけでは抜き切れない。

## Next Checks

固定上限は1.5 m/sへ戻す。次はfront distanceとShiftOut残時間から、Pass移行時に確保すべき
最小前方距離を逆算する距離適応型closing capが候補である。遠い対象には強く接近し、近い対象では
SafetyBrake距離へ入らないよう自動で上限を下げる。gap、wall、hard-boundary、EmergencyBrakeは維持する。

