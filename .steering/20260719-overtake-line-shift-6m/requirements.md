# Requirements

## Goal

dev3で、OvertakeLineのShiftOut距離を8.0 mから6.0 mへ短縮し、前走車速度上限が解除されるPassへ早く移行することで、D2がhard curve境界までにD3を抜き切れるかをA/B確認する。

## Scope

- `v2x_overtake_line_shift_distance`だけを6.0 mへ変更する
- ShiftOut最大接近速度1.5 m/sを維持する
- active hard-boundary completion機能を維持する
- SafetyBrake、EmergencyBrake、gap、wall、追い越し開始条件を変更しない
- `make dev3`で3台走行を確認する

## Acceptance Criteria

- 3台が発車し、衝突または全車停止を起こさない
- D2のShiftOut開始からPass開始までの時間・距離が短縮する
- hard境界中断位置または追い越し完了状況を前回runと比較できる
- solver failureとReverse発火の有無を確認する

