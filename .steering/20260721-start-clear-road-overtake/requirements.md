# Requirements

## 目的

dev3のスタートで前方が空いている車両が、後方横の他車を理由に`Follow`へ入り、
他車後方へ収束する挙動を解消する。また、開始済み追越しがヘアピンの短いgap判定欠落で
解除されないようにする。

## 要件

- 前方車なし・後方side車のみの場合は`Cruise`を維持する。
- 既に選択した追越し側は、実測された短時間のgap欠落を同じ側で継続する。
- EmergencyBrake、明示禁止WP、target position jump、wall boundsは緩和しない。
- ROS 2インターフェース契約は変更しない。

## Definition of Done

- 後方side候補の境界判定に単体テストがある。
- gap hold設定が最新ログの欠落時間を覆う。
- 対象packageのbuildとtestが成功する。
