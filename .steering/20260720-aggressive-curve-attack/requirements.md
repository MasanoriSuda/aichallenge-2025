# Requirements

## 目的

dev3の3台レースで、直線だけでなくヘアピン進入後もイン／アウトの空き回廊へ
機敏に進入し、短いV2X gap判定欠落で成立中の追い越しを中断しない攻め側設定を試す。

## 要件

- hard curve認識後でも、内側または外側に到達可能なgapが成立すれば新規ShiftOutを許可する。
- 新規進入は明示WP禁止、cooldown、EmergencyBrake、wall/body境界を越えて許可しない。
- ShiftOut/Pass中にgap width/time/reachabilityが一時的に欠落した場合だけ、最大0.5秒間locked sideを維持する。
- gap holdは同一locked targetが健全に観測され、位置jumpがなく、直前に有効gapを確認済みの場合だけ使う。
- 新規進入は連続2点、継続中は連続1点のgapで判定できるよう分離する。
- 追い越し入口距離、横加速度、closing speed、Return確認、cooldownをdev3向けに攻め側へ調整する。
- NaN/Inf、odometry timeout、solver異常、target ID/position jump、実車系設定は緩和しない。
- 設定省略時はhard curve新規進入とgap holdを無効にし、従来互換を維持する。
- ROS 2 topic、service、message、Domain、評価インターフェースは変更しない。

## 完了条件

- pure coreでhard curve進入とgap holdの成立／拒否条件を単体テストする。
- `make autoware-build`が成功する。
- 対象gtestが成功する。
- dev3実走で確認するログ項目を記録する。
