# Requirements

## Goal

通常Overtakeの新規entryを、成立する最小横移動（小まくり）を基本とし、
同程度の候補ならカーブ内側を選ぶ方針へ変更する。

## Requirements

- 現在のレーシングラインが相手footprintと非重複なら横移動しない。
- 重複する場合は、左右それぞれのvalidated corridor内で最小横移動目標を使う。
- 両側が成立する場合は必要横移動が小さい側を優先する。
- 横移動差が設定値以内ならカーブ内側を優先する。
- 一度ShiftOutを開始した後のside固定・early replan制限は維持する。
- 通常Overtake以外のStart Grid、Stuck Recovery、実車設定は変更しない。
- topic、service、messageのインターフェースを変更しない。

## Out of scope

- Overtake中の速度上限変更
- 壁・footprint・横加速度guardの緩和
- Start Grid breakoutの左右選択
