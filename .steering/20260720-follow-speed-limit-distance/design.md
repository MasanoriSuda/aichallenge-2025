# Design

## 状態分離

```text
0..24 m  前方車の検出、front risk、追い越し可否計算
0..5 m   generic Follow速度制限
動的停止距離内  SafetyBrake
```

検出距離を縮めず、速度制限だけを距離でgateする。これによりヘアピンをまたぐ共通進捗の
早期検出を失わず、直線で前車へ接近して追い越し機会を作れる。

## 速度上限

- 移動前車: `min(v_max, front_speed + v2x_moving_follow_speed_margin)`
- 低速前車: `min(v2x_follow_velocity, front_distance_velocity_limit)`
- 5 mより遠方: generic Follow capなし
- risk / curve / decel guard: 従来どおり独立して適用

## 互換性

`v2x_follow_speed_limit_distance=0.0`をlegacyの距離無制限として扱う。古いYAMLに新規キーが
なくても距離gateは従来どおりとなる。移動前車へ固定`follow_velocity`を重ねない点は意図した
挙動変更である。ROS 2 I/O、launch、package境界は変更しない。

## 観測

V2X debugへ`follow_cap`、`follow_moving`、`follow_cap_dist`を追加し、遠方Followと5 m以内の
速度制限をログだけで区別できるようにする。
