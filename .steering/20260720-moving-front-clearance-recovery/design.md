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
- target center distance: 5.0 m
- recovery speed margin: 0.6 m/s
- distance gain: 1.0 (m/s)/m
- maximum closing margin: 0.8 m/s（既存値）

したがって2.3〜4.4 mでは前走車より0.6 m/s遅く、5.0 mでは同速、5.8 m以上では従来どおり
最大0.8 m/s速い速度を許可する。

## 適用範囲

Followでは既存の5 m距離gate内で距離連動capを使うため、Follow中は5.0 mで同速、
5.0 mより遠方では従来の追い上げ速度になる。Overtake/ShiftOutでは5.8 m未満の回復域だけ
同じcapをhard上限として適用する。横クリア後のlocked targetは既存のfront overlap
除外latchにより対象外となるため、横に並んだ後の加速は妨げない。

中心間距離2.3 m以下のmoving frontは、required decelが小さくてもSafetyBrakeへ倒す。
nominalな前後車体占有長2.0 mより0.3 m手前で強制制動し、重なってからの回復に依存しない。
2.3 mを超えると距離capが前走車より低い速度を指示し、5.0 mまで車間を回復する。
中心間5.0 mはnominalな前後車体占有長2.0 mを引くと、約3.0 mのバンパー間距離に相当する。

## 互換性

変更は参加者MPC内部とconfigに閉じ、ROS 2および評価インターフェースは変更しない。
値は2025 AWSIM dev3シミュレーション向け暫定値であり、2026公式仕様または実車設定ではない。
