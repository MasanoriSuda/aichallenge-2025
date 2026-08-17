# Design

## 原因

既存処理はaggressive retryへ入った瞬間に、

1. `recovery_last_output_->state_reason == ForwardDurationLimit`
2. `recovery_last_maneuver_direction_ == Forward`

を要求していた。しかしstepwise Forwardでは、失敗後に
`StopAndReassess -> CheckClearance -> SafeStop(EscapeStepLimitReached)`へ遷移する。
その過程でreasonは上書きされ、maneuver directionも再判定時にresetされるため、
Forward失敗回数は増えなかった。`collision_worsening`も対象外だった。

## 方針

pure C++ coreへ`ForwardRecoveryFailureTracker`を追加する。

- `ForwardManeuver`から`CollisionWorsening`または`ForwardDurationLimit`へ遷移した時点で、
  現cycleの失敗をlatchする。
- aggressive retry開始時にlatched failureを1回だけ消費する。
- 連続失敗数がconfig閾値へ達したら、controllerの`forced_reverse_retry`を有効にする。
- Reverse maneuver終了またはForward escape成功でtrackerをresetする。

controllerは文字列化済みの最後の状態を逆算せず、trackerの結果だけを方向切替へ使う。

## 安全境界

force Reverseは既存の`resolve_recovery_candidate_direction_policy()`を使う。
このpolicyはForward probeとForward preferenceを無効化するだけであり、Reverseの
swept-footprint、V2X、gear、距離・速度制限は従来どおり評価される。

## 非対象

- Overtake経路が壁へ入る上流原因
- Recovery incident ledgerの診断用寿命
- Recovery距離・速度・接触marginの変更
