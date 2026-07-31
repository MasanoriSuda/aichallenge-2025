# 設計

## 現行課題

最新ログでは、`Pass -> FollowPrepare`後に右側`e_y=+1.97 m`から左側`goal=-1.26 m`へ再開し、3.23 m横断する例がある。

原因はside所有権が二重化していることにある。

1. BehaviorがOvertakeから一時的にFollowへ落ちると、curve cooldown処理がBehavior側のside lockを解除し得る。
2. `FollowPrepare`中のOvertake評価がOvertakeLineに保存されたmission sideではなくBehavior側lockだけを参照する。
3. 再開時はBehavior再評価sideがmission sideより優先される。

この組み合わせにより、OvertakeLineには元のsideが残っていても、Behaviorが反対側の最小横目標を作り、そのまま再`ShiftOut`できる。

## 変更方針

### 1. mission sideを正本にする

`FollowPrepare`中は`OvertakeLineState::pass_side_sign`をside選択のlockとして使用する。同じ側だけを再評価するため、minimum-lateral goalも保存済み側の安全区間から生成される。

### 2. active mission中はBehavior lockを解放しない

Behaviorが一時的にFollowへ落ちても、OvertakeLineが`ShiftOut`、`Pass`または`FollowPrepare`でtarget/sideを所有している間はcurve cooldownによるside lock解除を行わない。

### 3. FollowPrepare再開はmission side優先

再開sideの解決順を次に変更する。

```text
mission side
  -> 同じsideのBehavior再検証
  -> 新規Behavior side
```

Behavior側が反対sideを提示した場合は再開せず`FollowPrepare`を維持する。反対側を試すには、現在ミッションがIdleへ終了して新規entryになることを必要条件とする。

### 4. 横離隔成立時はPassへ直接復帰

保存済みsideとBehavior sideが一致し、corridorが実行可能で、現在の対象車との実横離隔がfront-cap解除閾値以上なら`FollowPrepare -> Pass`とする。それ以外は同じsideの最小目標へ`ShiftOut`する。

## 変更対象

- `v2x_overtake_core.hpp/.cpp`
  - 再開sideの優先順位
  - 直接Pass復帰判定
- `mpc_controller_cpp.cpp`
  - FollowPrepare中のeffective lock
  - active mission中のBehavior lock保持
  - 同側再開と直接Pass復帰
- `test/test_v2x_overtake_core.cpp`
  - mission side優先、反対side拒否、直接Pass条件

## 対象外

- SafetyBrakeそのものの距離閾値
- closing speedの再調整
- minimum-motion境界の内側マージン
- `a_max`および最高速度

