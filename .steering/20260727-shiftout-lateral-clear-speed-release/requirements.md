# Requirements

## 目的

`output/20260727-230821/d1`で、P1が壁側へ逸れたP2との実横離隔を確保し、
前方車集合からP2が外れた後も、OvertakeLineが`ShiftOut`であることだけを理由に
前車速度由来capを保持した問題を解消する。

## 根拠

- `1785161367.193`: `Idle -> ShiftOut`
- `1785161370.993`: `front=0`、`fd=inf`、locked target横差`-2.41 m`
- 同時刻でも`speed_cap=1`、`cap_release=0`
- `1785161375.218`: `ShiftOut -> Pass`
- Pass移行後に初めて`desired_v=11.11 m/s`、`cap_release=1`

実際の前方が空いてからcap解除まで約4.2秒、ShiftOut全体では約8.0秒を要した。

## 変更範囲

- `multi_purpose_mpc_ros`のlocked target実横離隔判定
- ShiftOut / Pass共通のfront-cap解除条件
- 解除閾値と再適用閾値の既存ヒステリシス
- pure helper単体テスト
- MPC暫定仕様

## 制約

- `v2x_overtake_pass_front_overlap_lateral_clearance=1.50 m`を解除閾値として維持する。
- `v2x_overtake_pass_front_cap_reapply_lateral_clearance=1.30 m`を再適用閾値として維持する。
- EmergencyBrake、front risk、別の前方車、wall、corridor、solver/odometry guardは緩和しない。
- `a_max=1.0 m/s²`、domain/global速度上限、curve速度制約は変更しない。
- ROS topic/service、Domain、評価成果物の契約は変更しない。

## 完了条件

- committed ShiftOut中でも、locked targetとの現在横離隔が解除閾値以上なら
  前車由来capを解除する。
- 解除後に現在横離隔が再適用閾値未満へ縮んだ場合はcapを再適用する。
- 横離隔未成立、対象観測欠損、Emergency、安全guard発動中は解除しない。
- 対象packageの単体テストとビルドが成功する。
- 実走効果はユーザーの`make dev2`で確認する。
