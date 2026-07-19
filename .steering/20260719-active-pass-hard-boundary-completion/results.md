# Run Data Analysis

## Summary

- valid run: `output/20260719-193720`
- D1/D2/D3は全車1周し、停止時点で2周目を走行中だった
- 全車停止、衝突、Reverse発火は再現しなかった
- D2のhard curve中断位置は従来runのWP148からWP153へ約5 m延びた
- D2はD3を抜き切れず、順位はD3、D2、D1のままだった
- active hard-boundary機能は部分成功として次A/B用に有効のまま残す

## Evidence

### D2 hard-boundary pass

- WP130: `Follow -> Overtake`, front distance 14.54 m
- WP150: `hard_continue=1`
  - hard boundary distance: 16.89 m
  - available distance: 16.39 m
  - required distance: 15.34 m
  - ego/front speed: 4.42 / 2.82 m/s
- WP153: `Overtake -> Follow`
  - hard boundary distance: 13.89 m
  - available distance: 13.39 m
  - required distance: 14.21 m
  - block: `overtake hard curve blocked`

従来run `output/20260719-191752`ではWP148でhard curveを検出した時点で即中断していた。
今回は完了予測が成立する間だけPassを維持し、予測が不成立になったWP153で中断した。

### Lap result

| 車両 | 今回 | 従来run | 差 |
|---|---:|---:|---:|
| D3 | 122.937 s | 126.230 s | -3.293 s |
| D2 | 128.560 s | 131.803 s | -3.243 s |
| D1 | 136.753 s | 138.289 s | -1.536 s |

絶対lap timeは全車で短縮したが、D2-D3差は5.623 sで従来の5.573 sから改善していない。
同一シミュレーション内の順位も変わらないため、追い越し性能向上とは判定しない。

### Solver

- D2のmaximum-iterationsログの大半はスタート待機中の速度0 m/sで発生し、その後復帰した
- 1周目WP86のD2追い越しはOSQP連続失敗でRecoveryへ移行した
- 対象となった2周目WP130〜153ではsolver failureは記録されていない
- 停止操作直後の`status=interrupted`は実験終了に伴うもので走行中断原因から除外する

## Topic Chain

- V2X vehicle positions: 3台を継続検出
- localization/reference progress: WP130からWP153へ連続進行
- behavior: Follow -> Overtake -> Followが診断値付きで遷移
- OvertakeLine/MPC: ShiftOut -> Passを完了し、hard completion不成立後にRecoveryへ移行
- control: 全車が1周後も制御出力を継続

rosbagはこのrunに保存されていないため、topic rateと実command波形はAutowareログから分かる
範囲だけを評価した。

## Suspected Causes

追い越し未完了の主因はhard guardの即時中断ではなく、Passへ入るまでの相対速度不足である。
現行`v2x_overtake_shiftout_max_closing_speed=1.0 m/s`によりShiftOut中は前走車への接近が
抑えられ、WP130の14.54 mからPass開始までに十分な縦距離を詰められていない。WP150で
Pass継続は可能になったが、3 m進んだWP153でrequired distanceがavailable distanceを上回った。

## Next Checks

次はhard-boundary機能を維持したまま、ShiftOut中の最大接近速度だけを1.0 m/sから段階的に
引き上げるA/Bが妥当である。rear-clear/bufferを0.5 m未満へ削るより、直線区間で早く縦距離を
詰めてPass開始時のhard境界余裕を増やす。SafetyBrake、EmergencyBrake、gap/wall判定は維持する。
