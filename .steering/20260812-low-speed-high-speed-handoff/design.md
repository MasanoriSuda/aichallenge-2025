# Design

## 1. Direct-control entry feasibility

direct-controlの初期PhaseはShiftであり、設定速度は
`v2x_low_speed_avoidance_shift_velocity` である。入口では次を計算する。

```text
available_distance = front_distance - front_reserve
required_distance = ego_speed * latency
                  + max(0, (ego_speed^2 - shift_speed^2) / (2 * max_deceleration))
```

`required_distance <= available_distance` の場合だけdirect-controlへ所有権を渡す。
不成立時もLowSpeedAvoidanceのローカル経路とMPC制約は有効なため、MPCが経路を
追従する。これにより停止車両を無視せず、高速状態で低速用の単点操舵へ切り替わる
ことだけを防ぐ。

front reserveには、低速回避のprepare距離と通常追従hard distanceの大きい方を使う。
最大減速度はMPCの`a_min`、遅延は`state_prediction_delay_sec`を使い、設定の重複を
増やさない。

## 2. Curve-centred steering bounds

direct-control時の回避補正は、基準曲率から得た操舵を中心に横加速度由来の補正幅を
設定する。その範囲と、直前操舵から1周期で到達可能な範囲を交差させる。

範囲が交差しない場合は操舵レートを優先して基準曲率方向へ1周期分だけ戻す。
これにより異常な直前操舵を無期限に中心として保持せず、同時に操舵指令の不連続も
作らない。

## 3. 非対象

- 通常Overtake Mission selectorの候補単位fault isolation
- target identity continuityの有限lease化
- body-clear handoff consume条件

上記はProレビュー上の残課題だが、今回の逸走では通常OvertakeLineが起動しておらず、
直接原因ではないため別ステアリングで扱う。
