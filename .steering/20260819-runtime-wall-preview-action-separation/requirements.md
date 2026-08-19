# Requirements

## 目的

`20260819-125346` で確認した、1.2 秒先の runtime wall 予告が候補不成立直後に
`ExitCurrentMission`へ直結し、ShiftOut/Passを早期中断する回帰を解消する。

## 要件

- 将来の壁 warning を「再計画を開始する preview」と「現行 Mission を中断できる action」に分離する。
- preview 中は fresh same-side Mission または centerward prefix を探索するが、候補不成立だけで現行の可行実行を破棄しない。
- 現在位置の warning、hard wall fault、または設定した短い TTC 以内だけを action 対象にする。
- 予告で得た走行距離を、横加速度を下げるための centerward shift 距離へ使用する。
- 既存の footprint、壁、横加速度、DP preflight は緩和しない。
- ROS 2 topic/service/message 契約を変更しない。

## 非対象

- ShiftOut失敗後の一般的なcenterward handoff全体の再設計
- SafetyBrake距離とclosing-speed envelopeの統合
- MPCC solver/modelの変更
