# Design

## 方針

`v2x_overtake_core`へ次のpure判定を追加し、ROS adapter側の状態遷移から利用する。

1. ShiftOut完了判定
   - phase hold時間、`shift_distance`、横目標誤差の全条件を満たしてPassへ進む。
2. front cap解除判定
   - Pass phase、横移動完了、同一locked targetの観測、targetが前方でないことを全て要求する。
3. active execution latch
   - 安全条件を満たすlocked target追い越し中は、1周期のgap再評価不成立をline stateでHoldする。
   - hard curve、明示禁止WP、completion不可、EmergencyBrakeなどはlatchしない。

速度上限はbehaviorの速度参照に加え、`OvertakeLineOutput::target_velocity_limit`からも
MPC制約へ適用する。これによりbehaviorが一時的にFollowへ戻っても、active lineが保持される間に
速度上限だけが外れることを防ぐ。

ShiftOutのclosing speedは既存の距離適応helperを使用し、最小値を0 m/sへ下げる。
Pass開始後、横クリアランスlatch前は専用上限0.5 m/sを使い、共通コース座標上で
対象との前後重なりを解除した後にベースtrajectory速度へ戻す。

追加の実走切り分けから、明示OvertakeLineをShiftOut / Pass中の唯一の横計画ownerとし、
phase累積走行距離でhorizon rampを進める。Pass目標はlocked targetのコース横位置を基準に置く。
すでに後方となったside targetでは新規ShiftOutを開始せず、横クリアランスlatch済みPassは
hard境界でも継続可能とする。新規開始、EmergencyBrake、明示禁止WP、cooldownは緩和しない。

## 影響範囲

- `include/multi_purpose_mpc_ros/v2x_overtake_core.hpp`
- `src/v2x_overtake_core.cpp`
- `src/mpc_controller_cpp.cpp`
- `test/test_v2x_overtake_core.cpp`
- `config/config.yaml`
- `docs/spec/mpc-integration.md`

既存のtopic、message、launch entry、Domain構成は変更しない。
