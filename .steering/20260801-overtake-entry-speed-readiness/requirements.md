# Requirements

## Purpose

`output/20260801-074818/d1`で、自車が1.39 m/s、対象車が約3.1 m/sの速度劣勢にもかかわらず、
inner curve entryから新規Overtakeへ入り、長時間の並走後に壁Recoveryと接触へ至った。
新規Overtakeの全入口に共通の速度準備ゲートを設け、対象車が明確に離れていく状態では
`Idle -> ShiftOut`を開始しない。

## Functional requirements

- 新規Overtake要求は、対象車に対する実測相対速度が設定閾値以上である状態を連続確認してから許可する。
- 対象車IDが変化した場合、連続確認時間をリセットする。
- 相対速度が閾値を下回った場合、連続確認時間をリセットする。
- `20260801-074818/d1`の再現値（ego=1.39 m/s、target=3.10 m/s）は必ず棄却する。
- 小さな速度劣勢は許容可能とし、既定値は相対速度-0.5 m/s、連続確認0.3秒とする。
- 既にShiftOut/Pass/FollowPrepareへコミット済みのmissionと、BehaviorからLineへの1周期handoffは新規入口として遮断しない。
- 条件未成立時はFollowを維持し、ShiftOut、Recovery、反対側選択を開始しない。
- 既存V2X debugに、速度準備不足の理由、相対速度、連続確認時間を残す。

## Constraints

- 壁余裕、gap幅、front cap、Pass未latch速度、Recovery経路、`a_max`は変更しない。
- start-grid、ROS topic/service/message、Domain、提出物の契約を変更しない。
- `aichallenge/result-summary.json`の既存ユーザー変更を変更しない。
- 既存の未コミット追い越し修正を巻き戻さない。

## Definition of Done

- 共通速度準備判定をpure helperとして単体テストできる。
- 再現速度ケース、連続確認、対象変更、速度低下、committed/handoffをテストする。
- `make autoware-build`が成功する。
- `multi_purpose_mpc_ros` packageテストが全て成功する。
- `git diff --check`が成功する。

