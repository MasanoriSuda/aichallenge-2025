# Requirements

## 目的

壁接触後の Stuck Recovery が、接触セル数だけを減らしながら基準経路から遠ざかり、最終的に候補方向不明の SAFE_STOP を反復する事象を防ぐ。

## 要件

- 基準経路から `rejoin.max_lateral_error_m` を超えて外れた車両は、横偏差をさらに悪化させる Recovery 候補を選択しない。
- 接触・前方障害物による「後退優先」は、安全確認済みで基準経路へ近づく前進候補がある場合に限り解除できる。
- ソルバ異常、協調停止、既に確定した Reverse-only episode の後退必須条件は変更しない。
- 静的地図、車体 footprint、V2X 完全性・衝突予測、速度・距離・時間上限を緩和しない。
- ROS 2 topic/service/message、評価結果、launch の契約を変更しない。

## 対象外

- Overtake Pass 中に壁余裕を超えて接触へ至る経路生成そのもの。
- 実車向け Recovery の有効化または速度・加速度上限の変更。

