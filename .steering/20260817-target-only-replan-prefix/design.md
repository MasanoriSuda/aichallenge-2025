# Design

## Overview

既存の target-bound execution hold を拡張する。新しい状態機械は追加せず、失敗分類と prefix 選択を局所的に整理する。

## Failure ownership

`OvertakeRecedingHorizonFailureKind` を導入する。

- `Target`: 相手予測との separation interval だけが不成立
- `Wall`: 壁の hard bounds が不成立、または軌道が壁 bounds を逸脱
- `Physical`: static map / lateral execution の物理再検証が不成立
- `Invalid`: サンプルや次元が不正
- `None`: hard failure なし

診断用の文字列は維持するが、hold / abort の分岐は enum を使う。

## Prefix selection

### Pass / completed ShiftOut

既存の整列済み warm-start prefix を壁のみで再検証して利用する。

### Incomplete ShiftOut

相手予測が変化した後に古い横移動を続けると接触方向へ進む可能性がある。このため、全 horizon を現在 `e_y` に固定した wall-only prefix を利用する。Mission と side は保持し、次周期から左右候補を即時再評価する。

専用 budget:

- 0.35 s
- 2.0 m
- forward-progress extension なし

Pass の既存 budget（1.50 s / 8.0 m）とは分離する。

## Safety ownership

target-only hold が上書きできるのは将来 target bounds の不成立だけ。以下は上書きしない。

- actual wall contact / margin blocked / sample unavailable
- non-recoverable current body overlap
- emergency front risk
- solver recovery request
- forbidden waypoint
- target identity discontinuity / position jump / course-progress rejection

## Logging

保持開始ログに `mode=freeze-current|last-feasible` と failure kind/index を含める。周期ログは増やさず、開始・終了・解決時のみ記録する。
