# Requirements

## 目的

dev3シミュレーション予選で、壁と前走車の間に物理的には通過可能な空間があるのに、
追加gap 0.8 mによって追い越しを拒否する挙動をA/B評価する。

## 変更条件

- `v2x_overtake_min_gap_width`: 0.8 mから0.2 m
- `v2x_overtake_guard_min_gap_width`: 0.8 mから0.2 m
- `v2x_overtake_line_min_wall_clearance`: 0.8 mから0.1 m
- 車両合成半幅1.45 m、予測余裕0.1 m、壁側margin 0.8 mは変更しない。
- topic、service、message、Domain、評価インターフェースは変更しない。
- 実車設定とは扱わず、2025 AWSIM由来dev3シミュレーションだけを対象とする。

## 完了条件

- YAML設定とビルドが正常である。
- dev3でOvertake開始、`wall_limited`、SafetyBrake、OSQP failureを比較する。
- 追い越し機会が増えるか、接触・停止・順位悪化が増えるかを結果へ記録する。
