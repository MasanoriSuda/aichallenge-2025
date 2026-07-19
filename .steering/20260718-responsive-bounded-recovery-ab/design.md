# Design

## A/B policy

失敗した積極復帰runの無制限再試行と予測sweep bypassを無効化し、成功実績のある
0.40 m単位のstepwise escape、2.0 m escape target、LowSpeedRejoinを使用する。

## Responsive tuning

| Parameter | Before | Candidate |
|---|---:|---:|
| Reverse acceleration sign | -1.0 | +1.0 |
| Reverse stop acceleration | +0.8 m/s2 | -0.8 m/s2 |
| Reverse acceleration magnitude | 0.5 m/s2 | 0.5 m/s2 |
| Reverse speed ceiling | 0.8 m/s | 0.8 m/s |
| Rejoin speed ceiling | 1.0 m/s | 1.1 m/s |
| Rejoin heading gain | 1.20 | 1.50 |
| Aggressive retry | enabled | disabled |
| Forced rejoin after retries | 3 | 0 (disabled) |
| Aligned early rejoin | unavailable | 0.60 m, abs(e_y)<=2.0 m, abs(e_psi)<=0.35 rad |

姿勢ゲインを横偏差ゲインより強め、位置だけを横切る挙動を抑える。Recovery中の制御は
primitive/FSMとpath feedbackであり、`rejoin_complete`後に通常MPCへ戻る。

Run 1で加速度0.7 m/s2は停止余裕を増やし、1 stepの実移動を約0.15〜0.20 mへ縮めたため
不採用とした。Run 2では元の駆動値へ戻し、current footprint、0.8 m static sweep、feedback
steering、最小退避距離、姿勢・横偏差をすべて満たす場合だけLowSpeedRejoinへ早期遷移する。

## Invariants

- `max_reverse_distance_m: 3.0`
- `max_reverse_duration_sec: 4.0`
- `reverse_escape_distance_m: 2.0`
- `escape_step_distance_m: 0.40`
- `max_escape_steps: 10`
- static swept-footprint and V2X clearance gates
