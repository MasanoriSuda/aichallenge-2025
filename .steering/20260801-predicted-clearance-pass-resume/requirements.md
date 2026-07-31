# 要件

## 目的

SafetyBrake後の同側Overtake再開で、瞬間的な横離隔だけを根拠に`Pass`へ戻り、対象車に再び塞がれて縦進捗を失う現象を抑止する。

## 必須挙動

1. `FollowPrepare`からの直接`Pass`復帰は、保存済みpass sideと再検証sideが一致する場合だけ許可する。
2. 現在だけでなく、設定時間先の対象車予測位置でも必要横離隔を満たすことを要求する。
3. 直接復帰に使う固定横目標は、現在位置より対象車側へ戻さない。
4. 自車速度が対象車速度を下回る間は、直接`Pass`へ復帰しない。
5. 直接復帰条件が未成立でも、反対側や`Recovery`へ移行せず`FollowPrepare`を維持する。
6. `FollowPrepare`中は既存の同側gap plannerによる横準備と加速を継続できる。
7. 新規Overtake開始、物理壁判定、SafetyBrake、solver Recoveryは従来どおり維持する。

## 設定

- 予測時間は既存の`v2x_prediction_time`を使用する。現行値は1.0秒。
- 横速度のdeadbandと上限は既存の`v2x_prediction_course_lateral_velocity_deadband/max`を使用する。
- 直接復帰の最低closing speedは0.0 m/sとし、対象車より遅い状態を拒否する。

## 制約

- ROS topic/service/message契約は変更しない。
- `a_max`、最高速度、ShiftOut/Pass closing speedは変更しない。
- 通常Pass中の経路更新とearly side replanは対象外とする。
- ユーザーの`aichallenge/result-summary.json`変更には触れない。

## Definition of Done

- 現在・予測横離隔、非内向き目標、closing speedを含む直接復帰判定を純粋関数でテストする。
- 条件未成立時に`FollowPrepare`を保持する制御経路が明示される。
- 再開時の固定横目標が現在位置より対象車側へ戻らない。
- 対象packageのテスト、ビルド、`git diff --check`が成功する。
