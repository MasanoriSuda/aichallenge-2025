# Design

## 前方危険応答の分離

```text
EmergencyBrake                         -> SafetyBrake (0 m/s)
stopped/slow front + inside stop range -> SafetyBrake (0 m/s)
moving front + inside stop range       -> relative-speed limit
```

従来の`has_danger_vehicle`は停止距離内という幾何条件だけを表し、必ず全停止する条件として
使われていた。pure coreの`resolve_front_danger_action()`で`None`、
`RelativeSpeedLimit`、`SafetyBrake`へ分け、moving frontは通常のFollow/Overtake選択まで進める。
そこでfront-risk cap、5 m Follow cap、ShiftOut capのいずれかを適用する。

## Hazard hold

holdはV2X分類境界の短い欠落を吸収するためだけに使う。moving frontの非Emergency条件では
deadlineを延長せず、同じ対象が観測されてclosing speedが0以下なら即解除する。dev3設定値は
1.0秒から0.25秒へ短縮する。

## 追い越し継続

新規追い越しのreachable-gap横加速度上限と、実際のOvertakeLine上限を4.0 m/s²で一致させる。
Pass中は既存のlocked-target/side保持を使い、一時欠落だけを0.75秒まで許容する。
EmergencyBrake、明示禁止WP、解消不能なhard curve、position jump、solver failureは従来どおり
Recovery/SafetyBrakeを優先する。

## 互換性

新しい判定は参加者MPC内部に閉じる。ROS 2 I/Oと評価契約に変更はない。設定値は2025 AWSIM
dev3シミュレーション用の暫定値であり、2026公式仕様または実車設定として扱わない。

