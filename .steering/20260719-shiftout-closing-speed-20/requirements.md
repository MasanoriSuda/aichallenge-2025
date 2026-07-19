# Requirements

## Goal

dev3で、ShiftOut中の最大接近速度を1.5 m/sから2.0 m/sへ引き上げ、OvertakeLineの8.0 m横移動を維持したままD2がhard curve境界までにD3を抜き切れるかをA/B確認する。

## Scope

- `v2x_overtake_shiftout_max_closing_speed`だけを2.0 m/sへ変更する
- `v2x_overtake_line_shift_distance: 8.0`を維持する
- active hard-boundary completion機能を維持する
- SafetyBrake、EmergencyBrake、gap、wall、追い越し開始条件を変更しない
- `make dev3`で3台走行を確認する

## Acceptance Criteria

- 3台が発車し、衝突または全車停止を起こさない
- D2がpass-side lineへ移動してからPassへ入る
- Pass開始時front distanceとhard境界判定を1.5 m/s runと比較できる
- solver failureとReverse発火の有無を確認する

