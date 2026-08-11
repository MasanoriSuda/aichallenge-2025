# Design

## 方針

### 1. Overtake entry stage policy

controller内の次の入力を`resolve_overtake_entry_stage()`へ渡す。

- base racing line DirectPass
- paused same-side DirectPass resume
- SafetyBrake pauseからのPass resume
- SafetyBrake pauseからのShiftOut resume
- 通常のpaused Mission resume

結果は`ShiftOut`または`Pass`と、既存ログに対応するreason enumを返す。
controller固有の`OvertakeLinePhase`とログ文字列への変換だけをcontrollerに残す。

### 2. Rearward completion context policy

`resolve_rearward_pass_completion_context()`が次を一度に分類する。

- targetがSideBySide committed Missionの後方にいるか
- ContactContinuation tailの共通資格があるか
- body-separated rear-clear tail候補か
- predictionで物理的にclearか
- 実測forward progressで継続可能か

ContactContinuation固有の横位置、相対横速度、時間上限と、SafeSeparation固有の
local/absolute budgetは従来どおり各policyに残す。

## 非対象

- 微小横移動をDirectPassへ変更する性能修正
- rearward tailの条件緩和
- entry pre-arm条件の変更
- config追加・変更

## 互換性

外部interfaceは変更しない。pure coreの内部API追加とcontroller内の呼び出し置換だけを行う。
