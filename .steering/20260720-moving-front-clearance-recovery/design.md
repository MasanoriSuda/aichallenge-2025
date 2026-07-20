# Design

## 距離連動の速度差

動く前走車に対する許容速度差を次で求める。

```text
speed_margin = clamp(
  distance_gain * (front_center_distance - target_center_distance),
  -recovery_speed_margin,
  maximum_closing_margin)
speed_limit = front_speed + speed_margin
```

dev3暫定値は以下とする。

- hard center distance: 2.3 m
- target center distance: 3.0 m
- recovery speed margin: 0.6 m/s
- distance gain: 1.0 (m/s)/m
- maximum closing margin: 0.8 m/s（既存値）

したがって2.3 mでは前走車より0.6 m/s遅く、3.0 mでは同速、3.8 m以上では従来どおり
最大0.8 m/s速い速度を許可する。

## 適用範囲

Followでは既存の5 m距離gate内で距離連動capを使う。Overtake/ShiftOutでも3.8 m未満の
回復域だけ同じcapをhard上限として適用する。横クリア後のlocked targetは既存のfront overlap
除外latchにより対象外となるため、横に並んだ後の加速は妨げない。

中心間距離2.3 m以下のmoving frontは、required decelが小さくてもSafetyBrakeへ倒す。
nominalな前後車体占有長2.0 mより0.3 m手前で強制制動し、重なってからの回復に依存しない。
2.3 mを超えると連続capが前走車より低い速度を指示し、3.0 mまで車間を回復する。
追走目標3.0 mは追い越し継続下限2.5 mより0.5 m外側で、境界付近の状態チャタリングを抑える。

## 互換性

変更は参加者MPC内部とconfigに閉じ、ROS 2および評価インターフェースは変更しない。
値は2025 AWSIM dev3シミュレーション向け暫定値であり、2026公式仕様または実車設定ではない。
