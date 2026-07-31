# Requirements

## 目的

MPC solver failure 時の fail-operational crawl が、既に基準経路から大きく外れた車両を
さらにコース外へ進め、Stuck Recovery 開始時点を復帰不能な位置まで遅らせる事象を防ぐ。

## 要件

- solver failure crawl は、基準経路に対する横偏差・方位偏差が既存の Recovery rejoin
  許容範囲内である場合だけ許可する。
- 現在の車体 footprint が静的地図上で完全に clear でない場合は crawl を許可しない。
- 非有限な追従誤差、無効な閾値、地図外・不明 footprint は安全側に停止する。
- crawl を棄却した solver fallback は既存の強制停止と Stuck Recovery 判定へ渡す。
- 安全な直線 Cruise での simulation-only crawl は維持する。
- ROS 2 topic/service/message、gear、評価結果、launch の契約は変更しない。

## 対象外

- Stuck Recovery のリバース速度・加速度・stepwise escape の変更。
- Overtake 経路生成または V2X 判定の変更。
- 実車向け fail-operational 動作の有効化。
