# Requirements

## 背景

`output/20260822-064544/d1/autoware.log`では、約3分の走行中にDynamic Escapeの
planning attemptが299個生成された。planner requestが一周期でもfalseになるとattemptを
即時破棄するため、同じ前方車`d2`を追跡中でもexit contract、solver backoff、左右候補の
連続性が失われている。

## 要件

1. 同一targetが前方scope内にいる間は、planner requestの一時欠落でattempt IDを変更しない。
2. target観測が短時間欠落しても、有限grace内は同一attemptを保持する。
3. target変更、観測喪失timeout、race/recovery resetではattemptを終了する。
4. solver解の期限を延長せず、attemptの同一性と実行経路の鮮度を分離する。
5. attempt開始・保持・終了理由を、過剰な周期ログを避けて追跡可能にする。
6. ROS topic、message、launch、評価結果schemaは変更しない。

## 制約

- 追い越し速度、壁clearance、左右評価weightは変更しない。
- staleなretained solutionの実行期限は延長しない。
- 参加者実装`aichallenge_submit/`内に変更を閉じる。
