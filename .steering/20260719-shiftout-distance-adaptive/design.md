# Design

## 適応式

ShiftOutの残り時間を次で見積もる。

```text
remaining_time = max(min_time,
                     remaining_shiftout_distance / max(ego_speed, minimum_speed))
distance_budget = max(0, front_distance - protected_front_distance)
raw_closing_speed = distance_budget / remaining_time
closing_speed = clamp(raw_closing_speed, minimum_closing_speed, maximum_closing_speed)
```

現行dev3では以下を使う。

- minimum closing speed: 1.5 m/s
- maximum closing speed: 2.0 m/s
- protected front distance: 既存の追い越し開始最小前方距離5.0 m
- remaining shift distance: 8.0 mからShiftOut走行済み距離を引いた値
- minimum speed: 既存の到達性判定最小速度1.0 m/s
- minimum time: 0.5 s

## 実装範囲

- `v2x_overtake_core`: 適応接近速度を計算する純粋関数と単体テスト
- `mpc_controller_cpp`: ShiftOut時の前方速度上限へ適用し、診断値をログへ追加
- `config.yaml`: dev3 A/B設定を有効化
- `docs/spec/mpc-integration.md`: 暫定仕様とA/B結果を記録

## 互換性

ROS topic、service、message、Domain、評価JSONの契約は変更しない。適応機能を無効にした場合は既存の固定`v2x_overtake_shiftout_max_closing_speed`を使う。

