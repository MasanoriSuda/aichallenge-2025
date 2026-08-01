# Requirements

## Goal

minimum-motionで検証済みの追い越し経路へ到達した後、固定横離隔の揺れだけで
Pass速度capが再適用される不整合を解消する。

## Requirements

- 通常Overtakeのminimum-motion Passだけを対象とする。
- 横離隔だけでなく、共通コース座標上の縦・横body footprintで非重複を判定する。
- 現在位置から設定済みV2X予測時刻までにbody footprintが交差しない場合だけ
  front capを解除する。
- 解除後も予測sweepが非重複なら、1.45〜1.50 m付近の横離隔変動だけでは
  capを再適用しない。
- 予測sweep重複、V2X位置ジャンプ、実壁接触ではcapを再適用する。
- Start Grid breakout、inter-vehicle corridor、通常の非minimum-motion Passは
  既存方針を維持する。
- front-risk、SafetyBrake、壁・横加速度Recoveryは無効化しない。
- topic、service、message、設定ファイルの公開契約を変更しない。

## Out of scope

- `v2x_overtake_pass_unlatched_max_closing_speed`などの数値変更
- wall clamp Recoveryの変更
- イン優先量、gap幅、壁余裕の変更
- Stuck Recoveryの変更
