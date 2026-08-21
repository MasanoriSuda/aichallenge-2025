# Requirements

## 目的

DynamicEscape が同一の前方車両を回避中であるにもかかわらず、新規進入用の判定や
一周期の候補欠損で横経路を破棄し、Follow・保持解・再計画を反復する構造を解消する。

## 必須要件

- 新規遭遇を開始する条件と、開始済み遭遇を継続する条件を分離する。
- 同一targetのattemptが有効な間は、新規進入条件が閉じてもGapPlanner評価を継続する。
- hard fault、Recovery、明示的な別Mission所有以外ではattemptを解除しない。
- target-blockingをsolver／wall失敗としてbackoffへ投入しない。
- attempt、target、side、実行解の採用元を一つのlifecycleログで追跡できる。
- 正常なincoming解の周期ログを抑え、状態変化と異常を中心に記録する。

## 制約

- ROS 2 topic、message、launch、提出インターフェースを変更しない。
- 車体寸法、壁余裕、速度、加速度、操舵制限は変更しない。
- 保持解の0.35秒上限を延長しない。
- 壁接触、out-of-map、EmergencyBrake、Recoveryのhard guardは維持する。
- `output/` と既存の `aichallenge/result-summary.json` を変更しない。
