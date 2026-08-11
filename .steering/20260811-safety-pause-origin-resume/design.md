# Design

## 観測した負のループ

現行では`Pass -> FollowPrepare`へ入るとPass用runtime stateが解除される。
再開時はfull lateral clearanceを満たす`direct_same_side_resume`だけが許可されるため、
まだ横移動途中だった車両は明確なfrozen pathを持っていてもlineを再発行できない。
その間にFollow capが効き、車間が広がって再試行がさらに難しくなる。

## 1. pause cause

`FollowPrepareCause`を状態に保存する。

- `SafetyBrake`
- `DynamicMissionWait`
- `TacticalRevalidation`
- `RecoveryRetention`
- `Unspecified`

origin phaseとcauseの両方を要求し、他用途のFollowPrepareを誤って攻撃的に再開しない。

## 2. origin-aware resume lease

pure coreの`resolve_paused_execution_resume()`が次を判定する。

- SafetyBrake由来
- originが`ShiftOut`または`Pass`
- frozen path、side、body-clear deadlineが有効
- targetが連続観測されている
- hard faultがない

結果は`Hold`、`ResumeShiftOut`、`ResumePass`とする。現在位置・予測位置とも
lateral clearanceを満たせば`ResumePass`、それ以外は同じfrozen goalへの
`ResumeShiftOut`とする。

## 3. BehaviorとLineの二段階適用

Behavior FSMではresume leaseを使い、SafetyBrake解除後にentry-only front-distance
判定でFollowへ戻されないようOvertake ownershipを復元する。

OvertakeLineでは同じleaseにcurrent wall/corridorのhard faultを追加し、実際に
`ShiftOut`または`Pass`へ遷移する。Behaviorだけが再開してlineがない状態を
継続させない。

## 4. 安全境界

EmergencyBrake中は再開しない。actual wall contact/margin/unknown、target side
intrusion、Mission invalidationでは再開せず、既存のRecovery経路へ渡す。
反対側candidateが良くても同一Mission中の全幅横断は行わない。

## 影響範囲

- `v2x_overtake_core.hpp/.cpp`
- `mpc_controller_cpp.cpp`
- `test_v2x_overtake_core.cpp`

設定ファイル、launch、topic/service契約は変更しない。
