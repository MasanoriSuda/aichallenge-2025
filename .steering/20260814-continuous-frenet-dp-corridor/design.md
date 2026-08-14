# Design

## 背景

現行HEADは左右候補を個別に作り、各候補を単一の `pass_lateral` へ縮約する。
そのため、予測ホライズン中にfree-corridorが横方向へ移動すると、全sampleに共通する
固定goalがなくなりMission生成が失敗する。

一方、実行側には既に以下がある。

- stage-wise wall/target bounds
- box-constrained receding-horizon lateral optimizer
- previous solutionのshift warm-start
- last-feasible execution lease

不足しているのは、左右および横方向セルの連続性を選ぶhomotopy層である。

## DP corridor

各path-distance sampleについて、左右候補のfree intervalを横方向binへ離散化する。
各branch内で次をコスト化し、始点から終端までの最小コスト列を求める。

- 現在横位置からの初期移動
- 隣接sample間の横移動
- 前回DP経路からの偏差
- 狭い回廊
- active sideからのbranch変更

横移動勾配が設定上限を超えるedgeは接続しない。DPは左右を独立homotopyとして解き、
相手車体を横断する途中side変更は生成しない。cross-side変更は既存no-return admissionが
監督する。

## 固定goal不成立時のbridge

既存のdynamic corridor intersectionが不成立でもDP経路が成立する場合、ShiftOut完了距離
におけるDP横位置を局所goalとして使う。ただしこの候補には `prefix_only` を付ける。

`prefix_only`候補は、既存の次の検証を全て通過した場合だけ実行できる。

- local ShiftOut preflight
- body-clear rollout
- target surface clearance
- wall footprint
- lateral acceleration
- prefix time/distance budget

rear-clear/Returnを含む完全Mission候補には昇格させない。実行中は既存rolling replanが
次のDP prefixを更新する。

## MPCC-lite scoreとの接続

DP branchの平均costをMPCC-lite候補へ付与し、hard-feasible候補間のscoreへpenaltyとして
加える。DPが未観測の場合はpenaltyを適用せず、既存挙動を維持する。DPが観測済みで
不成立のbranchはhard-feasible候補として採用しない。

## 状態保持

最後に採用したDP経路をtarget ID、side、timestampと共に保持する。同一target・同一side・
freshな場合だけ次回DPのprevious pathとして使う。reset、external maneuver、Mission終了で
破棄する。

## 安全境界

- DPはfree-corridorの選択器であり、wall/target hard boundsを緩和しない。
- DP bridgeはprogressive prefix専用であり、rear-clear feasibleを偽装しない。
- no-return後の反対side選択権は増やさない。
- DP不成立時に最後の経路を無期限保持しない。
