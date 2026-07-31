# 要件

## 目的

SafetyBrakeなどで一時停止した通常Overtakeを再開するとき、コミット済みのpass sideを維持し、反対側への大横断を発生させずに抜き切る。

## 必須挙動

1. `ShiftOut`または`Pass`から`FollowPrepare`へ入ったミッションは、再開時も保存済みのpass sideを使用する。
2. 再開時のBehavior再評価が反対側を提示しても、その周期で左右を切り替えない。
3. 保存済み側に実行可能corridorがある場合、その側で現在位置から最小横移動となる固定目標を使用する。
4. 保存済み側が実行不能の場合は`FollowPrepare`を維持し、反対側へ直接`ShiftOut`しない。
5. 現在の実横離隔が成立し、保存済み側のcorridorが再検証できた場合は、不要な再`ShiftOut`を省いて`Pass`へ復帰できる。
6. 物理接触、壁余裕違反、position jump、solver異常に対する既存の停止・Recoveryは維持する。

## 制約

- ROS topic/service/message契約は変更しない。
- start-grid専用のside再選択は対象外とする。
- ShiftOut closing speed、wall margin、加速度などのパラメータは変更しない。
- 新規Overtake開始前の左右比較と、明示的なearly ShiftOut replanは変更しない。

## Definition of Done

- FollowPrepare再開時はミッションsideがBehavior sideより優先される単体テストがある。
- 同側corridor不成立時に反対側へ復帰しない経路がコード上で明示される。
- 横離隔成立時の直接Pass復帰条件が純粋関数でテストされる。
- 対象packageのテスト、ビルド、`git diff --check`が成功する。

