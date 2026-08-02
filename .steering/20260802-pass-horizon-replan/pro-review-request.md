# Pro Review Request

## Status

レビュー完了。結果は `pro-review.txt` に保存済み。

判定は `Approve with changes`。P0指摘である静的／動的horizon分離、actionとcommitの分離、generation契約、絶対Pass原点、固定lineを出し続けるbounded Hold、side-by-side safe separation、wall contactとmargin-only violationの分離を `requirements.md`、`design.md`、`tasklist.md` へ反映した。

## 前提

kinematic rolloutと左右global candidate選択により、最新runではShiftOut 9回すべてがPassへ到達した。一方、Pass完遂は2回で、固定8 mの検証範囲を超えて最大約34.5 m同じ横目標を保持した。

既存の32 m progress watchdogは、対象へ0.5 m接近するたびに再装填されるため、この事象を検出できなかった。

## 提案

1. 入口rolloutでrear-clear時刻・距離を予測する。
2. predicted rear-clearまでのPass保持とReturnを含む経路を検証する。
3. 実行中はvalidated Pass距離・時間の手前で同じ側だけを再評価する。
4. horizon判断をReturn、RequestSameSideExtension、EnterHold、Abortへ分類し、extension採用処理とは分離する。
5. opposite-sideへのmid-Pass横断と、side-by-side状態から通常trajectoryへの無条件復帰は禁止する。

## 確認したい論点

1. Pass距離を`predicted rear-clear + reserve`で動的に決める設計は妥当か。
2. rear-clear予測とReturn preflightを入口で一体評価する場合、過度に候補を棄却しないための評価境界はどこか。
3. same-side extension失敗かつReturn corridor blocked時に、短時間のPass内部Holdを設けるべきか。それとも専用の上位状態が必要か。
4. runtime監視は距離slackと時間slackの両方を持つべきか。優先順位はどうするか。
5. absolute Pass上限24 m・10秒という初期値は、12 km/h対象とほぼ同速対象の両方に対して妥当か。
6. 入口で成立したShiftOut 9/9を維持しながらPass完遂率を上げるため、最初に最小実装すべき範囲はどこか。
7. current footprint非重複だがfuture corridor不成立の場合、競技用の攻撃的方針としてどこまでforward escapeを許すべきか。

## 採用判定

- Passがvalidated horizonを超えない。
- 6周でPass -> Return成功率が改善する。
- SafetyBrake、wall、solver failure、Reverseが減る。
- クリアラップ45～46秒台を悪化させない。
