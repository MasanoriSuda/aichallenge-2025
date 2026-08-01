# Design

## Scope

- `include/multi_purpose_mpc_ros/stuck_recovery_core.hpp`
- `src/stuck_recovery_core.cpp`
- `src/mpc_controller_cpp.cpp`
- `test/test_stuck_recovery_core.cpp`

## Separation

### RejoinAlignmentProgressTracker

正規化誤差、best値、最終material progress時刻を所有する。
Supervisorは観測値を渡し、返されたno-progress時間で既存timeout判定を行う。
状態遷移時のresetタイミングは現行と同じにする。

### RecoveryCandidateDirectionPolicy

次の2つを別フィールドで表現する。

- `forward_probe_allowed`: Reverse-only状態でもForward rolloutを評価可能か
- `prefer_forward_course_escape`: Forward rolloutをReverseより先に選ぶか

既存の論理式をpure functionへ移し、`evaluate_recovery_safety()`には解決済み方針を渡す。
今回の段階では、fallback unlockとcourse guardによるForward優先を含めて現行挙動を保存する。

## Non-goals

- Rejoinの中心線通過・横偏差悪化guard追加
- Forward固定の解除、Forward/Reverseのスコア比較
- retry cadenceや回数の変更
- パラメータ変更

これらは本リファクタリングの動作不変を確認した後の性能修正とする。
