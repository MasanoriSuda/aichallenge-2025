# Design

## 方針

車両の世界座標速度をreference courseの現在接線へ射影するだけでは、ヘアピンで
接線方向が変化した成分を横移動と誤認しやすい。そのため、速度推定に使われた
前回位置と現在位置を個別にbounded course projectionし、次式で横速度を求める。

```text
lateral_velocity = (current_course_lateral - previous_course_lateral) / sample_dt
predicted_lateral = current_course_lateral
                  + filtered_lateral_velocity * (sample_age + horizon_time)
```

`abs(lateral_velocity)` がdeadband以下なら0とし、その後に最大横速度へclampする。
投影失敗、無効な速度観測、時刻異常では既存のCartesian lateralを使う。

## 設定

```yaml
v2x_prediction_use_course_lateral_velocity: true
v2x_prediction_course_lateral_velocity_deadband: 0.15 # [m/s]
v2x_prediction_course_lateral_velocity_max: 1.0       # [m/s]
```

既存の次設定はA/B結果に従い無効のままとする。

```yaml
v2x_prediction_use_course_progress: false
```

## 影響範囲

- `v2x_overtake_core`: 横予測のpure helperと単体テスト
- `mpc_controller_cpp`: V2Xサンプル間隔保持、2点投影、gap plannerへの適用
- `config.yaml`: 実験設定
- `docs/spec/mpc-integration.md`: 暫定仕様とA/B方法

## A/B

- A: `v2x_prediction_use_course_lateral_velocity: false`
- B: `v2x_prediction_use_course_lateral_velocity: true`

比較対象は、ShiftOut開始回数、`locked target no longer executable`、Recovery理由、
ShiftOut→Pass→Return完了、接触、lap timeとする。

