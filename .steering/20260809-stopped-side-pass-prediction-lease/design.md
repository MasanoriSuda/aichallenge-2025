# Design

## 原因

OvertakeLineのtarget continuityには `v2x_overtake_target_hold_sec` があるが、
Pass horizon更新はcurrent cycleのfresh target predictionを必須としている。
そのためBehaviorがhard curve等で一周期だけlocked targetを外すと、target hold中でも
horizon更新が失敗し、SafeSeparationへ入れない場合はMissionを即破棄していた。

## 方針

`StoppedSidePassPredictionLeaseRequest` をpure coreへ追加する。次をすべて満たす場合だけ、
`fresh target prediction unavailable` を短時間保持として扱う。

- active Passかつfrozen Mission
- 直近target速度が停止閾値以下
- targetが横並び近傍
- target最終観測と「車体非重複＋予測sweep clear」の最終確認がfresh
- 現在の車体非重複
- 壁接触、壁余裕違反、wall sample欠損、Emergency、solver recoveryなし
- Pass絶対時間・距離budget内
- leaseの時間・距離budget内

controllerは最終safe prediction時刻とlease開始時刻・距離をMission stateに保存する。
fresh predictionが復帰すれば即解除して通常のhorizon更新へ戻る。lease期限を超えた場合は
従来のSafeSeparation / DynamicMissionWait / Recoveryへ移る。

## 外部Stuck Recoveryとの境界

今回のログでは前進指令中に車両が物理的に動かず、Stuck Recoveryが約0.32 m Reverseした。
Reverse後は現在位置・姿勢が変わるため、古いfrozen Pass pathは保持しない。
既存の `reset_after_external_maneuver` による再計画境界を維持する。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`: lease判定
- `mpc_controller_cpp.cpp`: Mission state、最終safe prediction記録、horizon失敗時の有界保持
- `test_v2x_overtake_core.cpp`: admission/expiry/hard-fault試験

設定値は既存の以下を再利用し、新しい調整ノブは増やさない。

- `v2x_overtake_target_hold_sec`
- `v2x_overtake_pass_horizon_hold_max_distance`
- `v2x_low_speed_avoidance_max_front_speed`
- `v2x_overtake_safe_separation_forward_escape_max_front_distance`
