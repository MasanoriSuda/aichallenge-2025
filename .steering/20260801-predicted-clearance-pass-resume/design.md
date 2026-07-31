# 設計

## ログで確認した課題

`20260801-072353/d1`では同側直接復帰自体は成功した。

```text
FollowPrepare -> Pass
side=1, current_ey=0.55, goal_ey=0.40
target_relative_lateral=-1.65 m
```

しかし、自車が対象車より遅い状態で`Pass`へ入り、横目標も0.15 m対象車側へ戻っていた。その後`body_clear=0`となり、対象車との縦距離が約2.8 mから11 mへ拡大して`committed pass longitudinal progress stalled`に至った。

rosbagでは加速度指令が継続して`+1.0 m/s^2`であるため、速度capの強化ではなく直接復帰の成立条件不足が主課題である。

## 変更方針

### 1. 対象車の短時間横位置を予測する

V2Xの観測速度を対象車位置のreference-path接線へ投影し、横速度を求める。既存のdeadbandと最大横速度を適用し、`v2x_prediction_time`先の相対横位置を計算する。

速度観測が未成立、position jump、非有限値の場合は直接復帰を許可しない。

### 2. 直接Pass条件を強化する

直接復帰には以下をすべて要求する。

```text
same mission side
execution corridor valid
target observation valid
current directional clearance valid
goal directional clearance valid
predicted goal directional clearance valid
goal does not retreat inward
ego_speed - target_speed >= 0
```

### 3. 条件未成立時はFollowPrepareを保持する

条件未成立時に`ShiftOut`へ即遷移すると、横目標到達だけで`Pass`へ進むため、再び速度条件を迂回できる。したがって、再開時だけは`FollowPrepare`を保持する。

`FollowPrepare`中もBehaviorは同じsideのOvertake corridorを出力し、既存のgap plannerは現在位置から外向きに単調な横目標を作るため、横準備と加速は継続する。

### 4. 再開固定目標を外向きに制限する

Behaviorの候補goalが現在位置より対象車側の場合、再開goalを現在`e_y`へclampする。これにより直接Passへ復帰した周期で横余裕を自ら減らさない。

## 変更対象

- `v2x_overtake_core.hpp/.cpp`
  - 予測横離隔・closing speed・非内向きgoalを含む直接復帰判定
- `mpc_controller_cpp.cpp`
  - locked targetの横速度・予測相対横位置
  - 条件未成立時の`FollowPrepare`保持
  - 再開goalの外向きclamp
- `test/test_v2x_overtake_core.cpp`
  - 各拒否条件と成立条件

## 対象外

- 通常`Pass`開始後の動的再シフト
- 事故後のsolver/stuck recovery
- SafetyBrake距離の変更
- globalなcourse lateral velocity predictionの有効化
