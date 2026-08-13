# Design

## 1. 候補変換の局所リファクタ

`OvertakeMissionCandidate`から`MpccLiteShadowCandidate`への変換と
admission理由決定を`v2x_overtake_core`の純粋関数へ分離する。

判定順は以下とする。

1. planner未評価／Missionなし
2. Mission自体の不成立
3. progressive entryのためrear-clear horizon未完成
4. rear-clear未確認／不成立
5. target clearance未確認
6. outer transition未検証
7. runtime emergency / solver Recovery
8. Mission総時間の残予算不足
9. SafeSeparationの残時間・残距離不足
10. 共通score評価

## 2. 左右同時評価

既存の`assess_side`へshadow-onlyモードを追加する。shadow-onlyでは、

- side retry blockを設定しない
- frozen/committed Mission continuityの早期returnを使わない
- locked targetを使ったside-replan相当のpreflightを行う

として、left/rightを8 Hzのshadow周期ごとに個別生成する。生成結果は既存の
`left_assessment`/`right_assessment`や実行sideへ書き戻さない。

## 3. Return判定

Pass中は従来どおりrear-clear確認後だけReturnを候補化する。Return実行中は、
Returnへ遷移済みという事実をrear-clear admission済みとして扱い、live Return
corridor、runtime emergency、solver Recoveryをhard gateとして再確認する。

## 4. 残予算

- Mission総時間: `mission_total_time_limit_sec - elapsed`
- SafeSeparation時間: `pass_horizon_safe_separation_max_sec - elapsed`
- SafeSeparation距離: `pass_horizon_safe_separation_max_distance - traveled`

候補の予測rear-clear時間・距離が残予算を超える場合は、理由を分けて棄却する。

## 5. ログ

各branchの`why`を細分化し、top-levelへ次を追加する。

- `both_sides`: 左右を同一周期にplanner評価したか
- `mission_rem`: Mission総時間の残り
- `safe_sep`: active / 残時間 / 残距離

authorityは引き続き`none`とする。
