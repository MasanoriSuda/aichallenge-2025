# Run Data Analysis

## Summary

- valid run: `output/20260719-194740`
- A/B変数: `v2x_overtake_shiftout_max_closing_speed` 1.0 -> 1.5 m/s
- D1/D2/D3は全車1周し、停止時点で2周目を走行中だった
- 全車停止、衝突、Reverse実行は再現しなかった
- D2のhard curve中断位置はWP153からWP154へ約1 m延びた
- D2はD3を抜き切れず、lap time差の改善も0.065 sに留まった
- 1.5 m/sは部分改善として維持するが、この変更単独では追い越し完了に不足する

## Evidence

### D2 target pass

| 指標 | 1.0 m/s run | 1.5 m/s run | 差 |
|---|---:|---:|---:|
| 追い越し開始 | WP130 / 14.54 m | WP130 / 14.08 m | -0.46 m |
| Pass開始 | WP137 / 12.98 m | WP137 / 12.35 m | -0.63 m |
| WP150 front distance | 8.92 m | 8.27 m | -0.65 m |
| WP150 required distance | 15.34 m | 14.41 m | -0.93 m |
| hard中断 | WP153 | WP154 | +約1 m |

WP130のShiftOut中はfront speed 2.68 m/sに対して`desired_v=4.18 m/s`となり、
1.5 m/s上限が実際に使われた。ShiftOutからPassまでは約2.33 sで、前回の約2.35 sと
ほぼ同じである。WP154ではavailable 12.39 mに対してrequired 12.73 mとなり、0.34 m不足で
`overtake hard curve blocked`へ移行した。前回はWP153で0.82 m不足だった。

### Lap result

| 車両 | 1.0 m/s run | 1.5 m/s run | 差 |
|---|---:|---:|---:|
| D3 | 122.937 s | 122.962 s | +0.025 s |
| D2 | 128.560 s | 128.520 s | -0.040 s |
| D1 | 136.753 s | 134.245 s | -2.507 s |

D2-D3差は5.623 sから5.558 sへ0.065 sだけ縮小した。D2自身のlap time差は0.040 sであり、
run間変動を超える競争性能向上とは判定しない。D1の短縮は対象パラメータの直接効果ではなく、
車間相互作用またはrun間変動の可能性が高い。

### Solver and recovery

- D2の連続maximum-iterationsはスタート待機中の速度0 m/sで発生し、発車後に復帰した
- D1は1周目に短いmaximum-iterationsを記録したが、走行を継続した
- D2の対象区間WP130〜154ではsolver failureは0だった
- Reverse状態、衝突ログ、stuck確定は記録されなかった

## Topic Chain

- V2X: D2はD3をlocked targetとして継続検出
- localization/reference progress: WP130 -> WP154 -> Recovery -> WP260まで連続進行
- behavior: Follow -> Overtake -> Followが診断値付きで遷移
- OvertakeLine/MPC: ShiftOut -> Pass -> Recoveryを完了
- control: 3台とも1周後も制御を継続

このrunにはrosbagが保存されていないため、topic rateと実command波形はAutowareログから分かる
範囲だけを評価した。

## Suspected Causes

1. 1.5 m/s上限が作用するShiftOutは約2.33 sしかなく、追加で詰められる距離が小さい。
2. Pass開始時点でもfront distanceが12.35 mあり、hard境界まで29.89 mでは抜き切れない。
3. Pass移行後は前走車速度上限が解除されるが、車両加速度制約によりWP150でも4.46 m/sに留まる。

## Next Checks

次は1.5 m/sを維持し、ShiftOut距離8.0 mを短くしてPassへの移行を早めるA/Bが候補である。
2.0 m/sまで接近速度を上げるだけでも改善は見込めるが、作用時間が短いため効果は限定的と予想する。
SafetyBrake、EmergencyBrake、gap、wall、hard-boundary判定は維持する。

