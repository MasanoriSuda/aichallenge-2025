# Design

## 判定

前方車両ごとに次を計算する。

```text
closing_speed = ego_speed - target_along_track_speed
distance_to_entry = max(0, forward_distance - overtake_entry_front_reserve)
time_to_entry = distance_to_entry / closing_speed
```

次をすべて満たす場合、通常GapPlannerの動的front候補にする。

- course-progress探索範囲内
- 現在または短時間予測したtarget lateral sweepがレーシングラインのrobust envelopeと重なる
- `closing_speed`が最小値以上、または既にentry reserve内
- `time_to_entry`がactivation horizon以内
- V2X速度が有効でposition jumpがない
- start-grid grace外

候補が複数ある場合は`time_to_entry`が最短の車両を選び、同値なら前方距離が短い車両を選ぶ。

## 低速確認

`corridor_promotion_max_speed`以下の車両は従来どおり連続観測確認後にauthorityを渡す。それより速い移動車両は、有効な速度観測と予測衝突条件を満たせば低速確認を要求しない。

## パラメータ

- `v2x_dynamic_obstacle_cruise_activation_horizon_sec`: entry reserveへ到達する予測時間の上限
- `v2x_dynamic_obstacle_cruise_min_closing_speed`: 遠方から計画を開始する最小相対速度
- `v2x_dynamic_obstacle_cruise_corridor_promotion_max_speed`: 連続観測確認を要求する低速域の上限
- 探索距離上限は既存の`v2x_front_progress_detection_distance`を共有する

## ロールバック

`v2x_dynamic_obstacle_cruise_authority_enabled: false`で従来authorityへ戻す。
