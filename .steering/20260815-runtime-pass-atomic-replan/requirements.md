# Requirements

## 背景

`20260815-191724` の試走では runtime Pass budget refresh により、
SafeSeparation の固定距離上限による中断は解消した。一方で、rear-clear が
絶対 Pass 予算内に収まらないと判定した後、tactical replan を 22 回要求したが、
新しい Mission の採用は 0 回だった。

ログ上は SafeSeparation 中の古い `CurrentSideHold` が shadow の最良候補として
残り、SafeSeparation 開始時に立った no-return latch によって、相手がまだ十分
前方でも反対側の枝評価・採用が止まっている。

## 要求

- runtime completion infeasible を一回の再計画要求としてラッチする。
- 待機中は古い frozen Mission の hold を shadow 選考から外す。
- 相手がまだ前方で、車体・予測・壁の hard guard が正常なら左右枝を再評価する。
- same-side または cross-side の新しい feasible Mission を atomic に置換する。
- side-by-side、実接触、壁 hard fault、target discontinuity では cross-side re-armしない。
- 新しい Mission 採用、Pass終了、または infeasible 解消時にラッチを解除する。
- ROS topic/service、設定値、評価インターフェースは変更しない。

## 制約

- 実車用ではなく現行 Racing Kart シミュレーション制御の局所変更とする。
- `aichallenge/result-summary.json` のユーザー変更は変更・コミットしない。
- 大規模なFSM追加やRecovery変更は行わない。
