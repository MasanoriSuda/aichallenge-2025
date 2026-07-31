# 設計

## 原因

`can_resume_paused_pass_directly()`は横方向の安全成立性に加え、次も要求している。

```text
ego_speed - target_speed >= 0.0
```

一方、SafetyBrake後の`FollowPrepare`ではFollow系の速度制約が残るため、自車が低速なまま
相手だけが加速するとこの条件を満たせない。Passへ戻れなければPass速度ポリシーも使えず、
速度を回復するための状態へ遷移できない循環依存になる。

## 変更方針

`FollowPrepare`から同じ側へ直接`Pass`復帰する判定を横方向の実行成立性へ限定する。

維持する条件:

```text
paused mission
same mission/behavior side
execution corridor valid
target seen and no position jump
target lateral prediction valid
current directional lateral clearance valid
goal directional lateral clearance valid
predicted goal directional lateral clearance valid
current/goal are on committed side
goal does not retreat inward
```

除去する条件:

```text
measured ego speed >= measured target speed
```

これにより横衝突回避条件は緩和せず、縦速度だけを理由にしたデッドロックを解消する。
新規追い越し進入には別のentry speed readinessがあるため、本変更はコミット済みミッションの
再開にだけ作用する。

## 変更対象

- `v2x_overtake_core.hpp/.cpp`
  - `PausedPassDirectResumeRequest`から速度3項目を削除
  - 直接再開判定からclosing-speed条件を削除
- `mpc_controller_cpp.cpp`
  - 直接再開requestの速度引数を削除
- `test/test_v2x_overtake_core.cpp`
  - 低速自車の再開許可を再現テスト化

## 動的確認

- `OvertakeLine paused resume guard held`の長時間連続が減ること。
- 横離隔成立後、`FollowPrepare -> Pass`が速やかに出ること。
- `Pass`復帰後に速度が回復し、対象車との縦距離を失わないこと。
- 壁Recoveryは別課題として件数を併記する。
