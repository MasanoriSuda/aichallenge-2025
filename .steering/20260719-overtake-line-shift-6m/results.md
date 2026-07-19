# Run Data Analysis

## Summary

- valid run: `output/20260719-195616`
- A/B変数: `v2x_overtake_line_shift_distance` 8.0 -> 6.0 m
- D1/D2/D3は全車1周し、停止時点で2周目を走行中だった
- 全車停止、衝突、Reverse実行は再現しなかった
- D2のPass移行はWP137からWP135へ早まった
- D2はPass側へ十分横移動する前にPassへ遷移し、WP141でgap幅不足により中断した
- hard-boundary継続判定まで到達せず、lap timeも悪化したため6.0 mは不採用とする
- 現行configは8.0 mへ戻す

## Evidence

### D2 target pass

| 指標 | 8.0 m run | 6.0 m run | 差 |
|---|---:|---:|---:|
| 追い越し開始 | WP130 / 14.08 m | WP130 / 15.05 m | run差あり |
| ShiftOut時間 | 約2.33 s | 約1.88 s | -0.45 s |
| Pass開始 | WP137 / ey=-0.06 m | WP135 / ey=-0.29 m | 2 WP早い |
| 中断 | WP154 / hard boundary | WP141 / gap width | 13 WP早い |

6.0 mでは積算前進距離が閾値へ達した時点でPassへ入ったが、車体はpass-side lineへ十分移って
おらず、Pass開始時の`ey=-0.29 m`だった。その後WP140でside clearance 1.37 mまで狭まり、
WP141では連続gap点が確保できず`overtake guard gap width, max=0.693, req=0.8`でRecoveryへ入った。
前回8.0 m runで成立したWP150の`hard_continue=1`には到達していない。

### Lap result

| 車両 | 8.0 m run | 6.0 m run | 差 |
|---|---:|---:|---:|
| D3 | 122.962 s | 122.987 s | +0.025 s |
| D2 | 128.520 s | 128.990 s | +0.470 s |
| D1 | 134.245 s | 134.801 s | +0.556 s |

D2-D3差は5.558 sから6.003 sへ0.445 s悪化した。run間変動を含むが、対象追い越しが
hard境界より13 WP手前で失敗した時系列と整合するため、6.0 mを採用する根拠はない。

### Solver and recovery

- D2の連続maximum-iterationsはスタート待機中の速度0 m/sで発生し、発車後に復帰した
- D1は1周目に短いmaximum-iterationsを記録したが走行を継続した
- D2の対象区間WP130〜141ではsolver failureは0だった
- Reverse状態、衝突ログ、stuck確定は記録されなかった

## Topic Chain

- V2X: D2はD3をlocked targetとして継続検出
- localization/reference progress: WP130 -> WP141 -> Recovery -> WP272まで連続進行
- behavior: Follow -> Overtake -> Followがgap診断付きで遷移
- OvertakeLine/MPC: ShiftOut -> Pass -> Recoveryを実行
- control: 3台とも1周後も制御を継続

このrunにはrosbagが保存されていないため、topic rateと実command波形はAutowareログから分かる
範囲だけを評価した。

## Suspected Causes

1. ShiftOut完了が実横位置ではなく積算前進距離でも成立するため、6.0 mでは横移動途中でPassへ入った。
2. 早いPass移行により前走車速度上限は解除されたが、pass-side line未到達のためgap再評価に失敗した。
3. 横加速度制限とwall制限が有効で、blend距離だけを短縮しても横移動自体は同じ速度では完了しない。

## Next Checks

8.0 mを維持する。次に追い越しを進める場合はShiftOut最大接近速度を1.5から2.0 m/sへ段階的に
上げる方が、Passフェーズ整合性を壊さずに前方距離を詰められる。SafetyBrake、EmergencyBrake、
gap、wall、hard-boundary判定は維持する。

