# Design

## 原因

Recoveryの`traveled_distance_m`は1回のmaneuverごとの距離であり、
`StopAndReassess -> CheckClearance -> ForwardManeuver`のたびに0へ戻る。
一方、復帰確認は`forward_escape_distance_m - tolerance`以上の物理移動を要求する。
今回のrunでは、clear後の各maneuverが閾値直前でduration limitとなり、合計では十分
移動していても永続的に復帰不能となった。

## 方針

`ClearForwardEscapeProgressTracker`をpure C++ coreへ追加する。

- Recovery中かつ現在footprintがclearなときだけ状態を保持する。
- ForwardManeuver中、前周期と現在周期の両方がclearで、runtime motion guardを通った
  odometry stepだけを累積する。
- HoldStop / StopAndReassess / CheckClearanceでは累積値を保持する。
- 接触、Reverse、Recovery終了、無効motion sampleで即resetする。

Forwardのescape確認には、maneuver距離とclear累積距離の大きい方を使う。
Reverseの4～8 m確認は従来のmaneuver距離のみを使う。

## 局所リファクタリング

motion guardを通過した1周期の距離を明示的な`accepted_motion_step_m`として保持し、
episode距離・incident ledger・clear Forward進捗の共通入力にする。これにより、同じ
pose差分を別々に再計算せず、妥当性確認済みの物理移動だけを扱う。

## 非対象

- 追い越しMissionが壁際へ入りすぎる上流問題
- Recovery速度・加速度・最大距離の攻撃化
- MPC/MPCC solver設定
