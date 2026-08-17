# Design

## 方針

AWSIM 復帰を Recovery episode の終了とはみなさず、Recovery 内の「外部 pose handoff」として扱う。

1. `WaitAwsimRecovery` へ入った時点の pose を保存する。
2. 待機中に位置または yaw が閾値を超えて変化した場合、現在 pose で waypoint を global 再対応し、MPC の古い warm start / tactical mission を破棄する。
3. `WaitAwsimRecovery` を抜ける際は、走行距離、contact baseline、選択済み primitive、direction latch、Forward 失敗履歴を現在 pose から再構築する。
4. AWSIM resolved の Normal 復帰には footprint clear に加え、既存の `max_rejoin_lateral_error_m` と `max_rejoin_heading_error_rad` を要求する。
5. 許容外の姿勢では `StopAndConfirm -> CheckClearance` を通し、既存の current-pose safety rollout に方向を選ばせる。

## 変更コンポーネント

- `stuck_recovery_core.hpp/.cpp`
  - pose handoff 判定を純粋関数として追加する。
  - `WaitAwsimRecovery` の Normal 復帰へ姿勢整合条件を追加する。
- `mpc_controller_cpp.cpp`
  - AWSIM wait anchor と一回限りの global waypoint 再対応を追加する。
  - handoff 後の Recovery 局所状態を一括で張り直す helper を追加する。
- `test_stuck_recovery_core.cpp`
  - aligned / relocated / reversed の判定と Supervisor 遷移を検証する。

## 非対象

- AWSIM 側の自動復帰機能の無効化・変更。
- 通常走行、追い越し、MPCC の評価関数変更。
- Recovery の Reverse 距離・速度・加速度の再調整。
