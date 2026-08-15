# Requirements

## 背景

`6e01605` の試走 `output/20260815-123901` では、DP execution authority の保持ログは
63 回出た一方、解除も 24 回発生し、DynamicMissionWait で `continuous_dp=1` が成立したのは
1 回だけだった。新しい rolling refresh は実行中経路を直ちに置換して runtime validation
lease を消すため、未検証候補が旧検証済み経路の継続を妨げている。

## 目的

- 実行中の検証済み DP 経路を、新しい未検証候補から分離する。
- 新候補は現在状態から wall/kinematic/target hard guard を通過した場合だけ原子的に昇格する。
- 候補棄却時は、旧経路、進捗、runtime validation lease を変更しない。
- 検証済み連続 DP prefix が前進出力を所有している間は、短い DynamicMissionWait
  reselect timeout だけを理由に Mission を捨てない。
- hard fault、対象不連続、壁接触、solver recovery、総 Mission budget は従来どおり優先する。

## 変更範囲

- `multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
- `multi_purpose_mpc_ros/include/multi_purpose_mpc_ros/v2x_overtake_core.hpp`
- `multi_purpose_mpc_ros/src/v2x_overtake_core.cpp`
- `multi_purpose_mpc_ros/test/test_v2x_overtake_core.cpp`
- 通常用・提出用 config の DP runtime validation lease

## 非対象

- Stuck Recovery の方策変更
- V2X/ROS 2 topic・message 契約変更
- 評価基盤、result schema、AWSIM の変更
