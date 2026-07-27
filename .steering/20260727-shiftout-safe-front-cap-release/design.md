# Design

## 1. front-cap解除の所有者

OvertakeLineが当周期の横目標horizonを評価してからfront-cap解除を決める。
Behavior側はOvertakeLineが前周期に確定した解除状態を参照し、同じ周期の
OvertakeLine速度参照で安全側へ上書きできる構造を維持する。

## 2. 解除条件

commit済みShiftOut / Passについて、次をすべて要求する。

1. locked targetを有効に観測している
2. pass側の横目標へ到達している
3. 実行horizonが横加速度、wall bound、static wallの制限を受けていない
4. 現在横離隔が解除閾値以上、またはtargetが自車より後方

横離隔解除後の1.30 m再適用ヒステリシスは維持するが、横目標未到達または
horizon制限発生時はヒステリシスより優先してcapを再適用する。

## 3. 評価順序

従来はfront-cap解除を決めた後にOvertakeLine horizonを評価していた。
横目標・horizon評価を先に行い、その結果を速度解除helperへ渡す。

## 4. 非変更範囲

候補gap、pass side選択、Recovery遷移、closing speed設定、start-grid breakout、
Emergencyおよび別車両のfront capは変更しない。
