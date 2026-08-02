# Requirements

## 背景

`output/20260802-193929` ではPass horizonは5回すべてfreshな状態で開始できたが、
同側extensionは3回要求して0回成功した。追い越し結果は正常完遂1回、SafetyBrake中断2回、
horizon枯渇Recovery 1回、壁制約Recovery 1回だった。

SafetyBrake中断した通常Passでは、front cap解除後に予測footprintが重複へ変化しても
固定横goalを維持し、接触寸前まで `attack_hold` が継続した。現行extensionはPass距離の
延長が中心で、相手の横移動へ追従する同側横goal再計画になっていない。

## 目的

- Pass中の連続した予測重複を、horizon期限より前の再計画トリガーにする。
- 反対側へ横断せず、現在選択中の側で最小限の横goal調整を1回だけ行う。
- 横goal変更を含むShift/Pass/Return全体を再検証してからatomicに採用する。
- 再計画不能時は既存bounded Holdへ移し、未検証ラインで突進し続けない。
- extension失敗段階を状態変化ログだけで判別できるようにする。

## 制約

- `/control/command/control_cmd` を含むROS 2 topic・message・service契約は変更しない。
- `aichallenge_submit/` 内へ変更を閉じる。
- start-grid breakoutとinter-vehicle corridorは今回の同側goal再計画対象外とする。
- target ID、pass side、mission generation、absolute distance/time limitを維持する。
- 同一missionのextension上限は1回のままとする。
- 現在footprint重複またはEmergency時は再計画より既存hard guardを優先する。
- 生成済み `aichallenge/result-summary.json` のユーザー変更は触らない。

## Acceptance criteria

- 連続予測重複が確認されたPassで、期限余裕が残っていても同側extensionを要求する。
- replacement goalは同じpass sideに留まり、現在のtargetと必要車体離隔を満たす。
- 横移動距離に応じた再計画距離を使い、固定0.5 mへの依存をなくす。
- 通常の1周期goal平滑化値と、atomic replacementの総goal変更上限を分離する。
- extension成功時はgoal、Pass保持距離、Return開始、静的・動的期限を一括更新する。
- extension失敗時に `freshness / rollout / pass distance / static preflight / goal jump / commit`
  のいずれかをログで判別できる。
- 既存単体テストと追加テストが成功し、`make autoware-build` が成功する。

