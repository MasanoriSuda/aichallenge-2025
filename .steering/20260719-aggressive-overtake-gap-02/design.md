# Design

## 判定モデル

直線・平行、V2X covariance 0のとき、壁と前車車体端の必要幅は次で近似する。

```text
1.45 m  物理車幅
+0.075 m 壁側追加余裕
+0.10 m  V2X予測余裕
+0.20 m  center-line gap
=1.825 m
```

従来0.8 m設定の約2.425 mから0.6 m縮小する。

## 方針

- plannerとguardは両者の最大値を使うため、両方を0.2 mへ変更する。
- OvertakeLineは選択後のgap両端からmarginを取るため、0.2 m corridorの中央を一意に狙える0.1 mとする。
- front risk、SafetyBrake、curve completion、solver fallbackは変更せず、横gap緩和だけを切り分ける。

## 観測指標

- `V2X behavior: Follow -> Overtake`
- `OvertakeLine: Idle -> ShiftOut` / `ShiftOut -> Pass`
- `wall_limited=1`
- gap拒否理由の`max` / `req`
- `SafetyBrake`、OSQP failure、接触後停止
- 追い越し完了と順位変化
