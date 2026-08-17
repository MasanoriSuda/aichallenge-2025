# Requirements

## 目的

MPCCが生成した可行な追い越し軌道を、一周期の数値誤差・phase遷移・短い更新欠落で
失わないようにする。離散Missionへの瞬間的なフォールバックを減らし、ShiftOutから
Passまで連続して追い越しを実行する。

## 対象

- MPCC primalからの実行軌道抽出
- 保存済み実行軌道のcontext・進捗・age判定
- 現在のstatic mapに対する物理再検証
- DP実行authorityが短時間欠けた場合のlast-feasible解による橋渡し

## 制約

- actual wall contact、緊急制動、target discontinuityは従来どおり即時失効させる。
- 別target、別side、別Mission generationの解を流用しない。
- ユーザーが変更中の `config.yaml` と結果JSONは変更しない。
- ROS 2 topic、message、launch契約は変更しない。

## Definition of Done

- OSQPが受理した数値許容差内の解を実行軌道として抽出できる。
- 同一MissionのShiftOutからPassへの遷移でlast-feasible軌道を再整列できる。
- fresh solveが短時間欠けても、現在の物理再検証を通る軌道だけ継続できる。
- hard wallおよびruntime hard guardは橋渡しで覆わない。
- package buildと単体テストが成功する。
