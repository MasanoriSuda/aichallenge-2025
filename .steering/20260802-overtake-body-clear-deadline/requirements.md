# Requirements

## 目的

追い越し開始後、locked targetとの車体横離隔が成立する前に
`v2x_moving_follow_hard_distance`へ到達し、SafetyBrakeから
FollowPrepareへ落ちる失敗を減らす。

## 変更範囲

- ShiftOut候補ごとの車体横離隔成立時刻を予測する
- 危険車間到達より前に横離隔を作れない候補を開始前に棄却する
- 成立候補は最短ShiftOut距離より横離隔成立の早さを優先する
- 棄却数と選択候補の予測値を既存の集約ログへ追加する
- 設定でA/Bを切り替えられるようにする

## 制約

- 制御周期40 Hz、加速度上限1.0 m/s^2を変更しない
- 壁余裕0.15 m、横加速度上限6.0 m/s^2を変更しない
- ROS topic/service/message契約を変更しない
- 実行中のSafetyBrake、wall、MPC hard constraintは無効化しない

