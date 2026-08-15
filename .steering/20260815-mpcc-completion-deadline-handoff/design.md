# Design

## 原因

期限延長の入力が`OvertakeMissionCandidate::rear_clear_prediction_*`へ直結している。
progressive candidateは完全rear-clear Missionではないため、この値を持たない。
一方、MPCC-lite shadowはprefixのbody-clearとterminal progressを組み合わせた
有限時間の完遂予測を既に生成しているが、ログ・rankingにしか利用していない。

また期限延長はMission置換関数内で一度だけ評価される。置換時に残時間が十分でも、
Pass継続中に予測残時間が期限を上回った場合は再評価されず、15秒でRecoveryになる。

## 局所リファクタ

### 1. 型付きMPCC-lite完遂予測

coreへ以下を追加する。

- `MpccLiteCompletionPredictionSource`
  - `None`
  - `CompleteRearClear`
  - `RecedingPrefix`
- `MpccLiteCompletionPredictionRequest/Resolution`
- `resolve_mpcc_lite_completion_prediction()`

fresh、hard-feasible、side有効、有限の時間・距離を必須とする。active Missionでは
固定Holdではなく現在状態から再評価した同側branchを使い、完全Missionと
実行許可済みprefixを区別したままcontrollerへ渡す。

### 2. 期限延長APIの一般化

DynamicMissionWait専用名だったdeadline extensionを、同側Pass完遂用へ一般化する。

- `DynamicMissionWaitDeadlineExtension*`
  -> `MissionCompletionDeadlineExtension*`
- `rear_clear_prediction_valid`
  -> `completion_prediction_valid`
- `predicted_rear_clear_time_sec`
  -> `predicted_completion_time_sec`

計算式と累積上限は変更しない。

### 3. active Passでの継続更新

Mission total budget判定直前に、最新のMPCC-lite完遂予測を評価する。
延長を許可する条件は次のANDとする。

- phaseが`Pass`
- frozen Missionが有効
- 予測sideが現在Missionと同じ
- target観測とcourse progressが連続
- current bodyが非重複、またはrecoverable side contact
- footprint predictionが有効
- wall contact/margin/sample faultなし
- EmergencyBrake、solver recovery、forbidden waypointなし

必要残時間は`予測完遂時間 + clear/prediction reserve`。既存と同じく累積最大
`min(4.0, max(1.0, 0.25 * mission_total_time_limit))`、現設定では3.75秒とする。

## Pass phaseについて

現行の`resolve_paused_replacement_execution_mode()`は、Pass由来・同側の置換を
既に`ContinuePass`へ解決している。今回ログでも唯一のPass由来置換は
`mode=pass-continuation, phase=Pass`だった。ShiftOutへ戻った他の置換は
ShiftOut由来であり、body-clearを保証せずPassへ昇格させる変更は行わない。

## 動的確認項目

- `Mission completion deadline extended`が必要局面で出る。
- `extension`が0より大きく3.75秒以下。
- `Pass -> Return`が発生する。
- wall/solver hard fault時に延長されない。
- 同一Missionが3.75秒を超えて延命されない。
