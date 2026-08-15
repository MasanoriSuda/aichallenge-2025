# Design

## 局所リファクタ

`UnseparatedClosingReserve`は「現在の車体矩形が分離しているか」を入力として
いたため、縦方向に離れている接近段階では無効だった。責務を横離隔成立前の
closing管理へ限定し、`LateralClearanceClosingReserve`へ改名する。

入力も`current_body_footprints_separated`から
`lateral_body_separation_established`へ変更する。これにより、前車の縦位置が
2.30 mより遠くても、横離隔未成立なら残り横移動距離に応じてclosingを漸減する。

## Mission entry距離

通常走行の新規Missionについて、候補ごとに次を計算する。

```text
closing = max(current_closing, planned_closing)
closing_budget = closing * (predicted_body_clear_time + prediction_margin_time)
required_front_distance = max(
  configured_entry_floor,
  body_longitudinal_clearance + reserve_distance + closing_budget)
```

現在すでに横車体離隔が成立している候補とstart-grid専用breakoutには適用しない。
通常の新規ShiftOutだけに適用し、active Missionの再計画をこのentry-only条件で
破棄しない。

初期値は次とする。

- 新規ShiftOut中心間距離: 4.5 m以上
- Follow cap開始: 中心間4.5 m
- Follow目標: 中心間4.0 m（2 m車体同士で面間約2.0 m）
- hard center distance: 2.05 m（変更なし）
- unseparated reserve: 0.25 m（body clearanceと合算して2.30 m、変更なし）

## 診断

Mission候補が見つからないログへ次を追加する。

- entry longitudinal reserveによる棄却数
- 棄却候補の最小必要中心間距離

採用Missionログには実測entry距離と候補が要求した距離を表示する。
