# Requirements

## 目的

`DynamicMissionWait` で安全な同側前進prefixが成立しているにもかかわらず、Behavior側の
`SafetyBrake` と再開後の未latch Pass速度capにより失速する不整合を解消する。

## 対象

- `FollowPrepare / DynamicMissionWait` 中の前進権限とSafetyBrake仲裁
- fresh same-side `Pass` 再開時のfront-cap状態引継ぎ
- 上記条件のpure coreと単体テスト

## 制約

- 実車体重複、target位置飛び、予測欠損、予測sweep重複では権限を引き継がない。
- alternate sideやShiftOut再開へfront-cap解除状態を渡さない。
- 壁・solver・実接触のhard guardは変更しない。
- ROS 2 topic/serviceおよび評価インターフェースを変更しない。

## 完了条件

- full-closing prefix成立中は、locked targetとの縦距離だけを理由にSafetyBrakeへ落ちない。
- fresh same-side Pass continuationでfront-cap解除状態を引き継ぐ。
- 不確実性または重複時は従来どおりfail closedとなる。
- package buildと単体テストが成功する。
