# Design

## 観測事象

`20260815-150227` では `Pass -> Return` が11回へ増えた一方、全件が
`SafeSeparation confirmed target clear` で、target はまだ約2.3～5.3 m前方だった。
rear-clear による正常完遂は0回であり、見た目の「抜こうとして引く」に一致する。

## 方針

Pass中のtarget-ahead状態を純粋関数で次の三つへ分類する。

1. `ForwardEscape`: 現車体・予測sweep・corridorがclear。Passを維持して前進加速する。
2. `HoldSameSide`: 現車体とcorridorはclearだが予測sweepが未確定。速度を落とさず同側再計画を待つ。
3. `Inactive`: hard fault、target不連続、距離上限外。既存の再選択/中断処理へ渡す。

前方継続は6 m以内に限定する。SafeSeparationの既存local/absolute budgetも
そのままhard boundとして使い、無期限追走にはしない。

## DP後処理の確認結果

target execution boundから外れた結果は、現行コードですでに境界へ射影してから
壁・横加速度・実車体を再評価している。ログに残るtarget-bound failureは、射影後に
static wall補正を適用すると物理target境界と両立しないケースだった。

ここを強制clampすると壁条件を破るため、本変更では緩和しない。既存の
last-feasible prefix保持を維持し、前方targetを成功扱いしてReturnする縦方向の
誤判定だけを修正する。
