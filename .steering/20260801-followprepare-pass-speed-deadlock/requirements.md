# 要件

## 目的

SafetyBrake後に保持した追い越しミッションが、`FollowPrepare`中の低速状態を理由に
`Pass`へ復帰できず、さらに先行車から離される負のループを解消する。

## ログ根拠

`20260801-081138/d1`では、`Pass -> FollowPrepare`後に直接再開判定が53回保留された。
時刻`1785539613.732`では、同じpass side、validated corridor、現在横離隔1.84 m、
1秒先予測横離隔2.60 mが成立していたが、`ego=1.39 m/s`、`target=3.00 m/s`のため
再開できなかった。`FollowPrepare -> Pass`まで約66秒を要した。

## 必須挙動

1. 保存済みpass sideと再検証sideが一致することを引き続き要求する。
2. 現在・再開goal・予測goalの方向付き横離隔を引き続き要求する。
3. 再開goalを現在位置より対象車側へ戻さない。
4. validated execution corridor、target observation、position jump判定を維持する。
5. 上記横安全条件が成立した場合、実測の`ego_speed < target_speed`だけを理由に
   `FollowPrepare`を保持しない。
6. 復帰後の縦加速は既存のPass速度ポリシーに所有させる。

## 対象外

- 新規`Idle -> ShiftOut`のentry speed readiness
- 壁際へ過大に張り出すPass経路
- wall Recovery、solver/stuck recovery
- gap幅、壁余裕、SafetyBrake距離、最高速度、加速度上限
- ROS topic/service/message契約
- ユーザーの`aichallenge/result-summary.json`変更

## Definition of Done

- 直接Pass再開APIから実測closing-speed入力と判定を除去する。
- 自車1.39 m/s、対象車3.00 m/sでも、全横安全条件成立時は再開可能とテストする。
- 横離隔、予測離隔、side、goal、corridorの既存拒否テストを維持する。
- `make autoware-build`、packageテスト、`git diff --check`が成功する。
