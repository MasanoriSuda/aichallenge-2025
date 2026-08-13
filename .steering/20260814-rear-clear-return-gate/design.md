# Design

## 1. Runtime wall warning

`RuntimeWallPreplanRequest`へrear-clear情報を渡す。
`ReturnToBaseLine`は、壁だけでなくlocked targetのrear-clearも成立した場合だけ許可する。

rear-clear未成立かつ、fresh same-side候補・center contractionが使えない場合は
`HoldCurrentSide`を返す。これは安全判定の緩和ではなく、soft warningだけで
前車の進路へ横断しないための明示的な継続判断である。hard wall faultは従来どおり
後段で停止・Recovery判断を行う。

## 2. Same-side MPCC-lite replacement

同側候補のscoreがcurrent-side holdを上回る量を計算し、既存の
`side_quality_min_score_advantage`以上の場合だけ置換を許可する。current holdが
hard-infeasibleの場合は候補を許可する。

さらに既存の`opponent_side_replan_stable_sec`を最小置換間隔として利用し、
Mission generationの短周期リセットを防ぐ。新しい調整パラメータは増やさない。

## 3. ログ

- rear-clear未成立でReturnを抑止したこと
- 同側候補のscore advantageとcooldown状態

を既存MPCC-lite debugへ追加し、次回試走で判定できるようにする。

