# Requirements

## Goal

dev3で、ShiftOut中の最大接近速度を1.0 m/sから1.5 m/sへ引き上げ、D2がhard curve境界までに前走車を抜き切れるかをA/B確認する。

## Scope

- `v2x_overtake_shiftout_max_closing_speed`だけを1.5 m/sへ変更する
- active hard-boundary completion機能は有効のまま維持する
- SafetyBrake、EmergencyBrake、gap、wall、追い越し開始条件は変更しない
- `make dev3`で3台走行を確認する

## Acceptance Criteria

- 3台が発車し、全車停止または衝突デッドロックを起こさない
- D2の対象追い越しについてShiftOut開始、Pass開始、hard境界判定をログで比較できる
- 前回run `output/20260719-193720`との差を記録する

