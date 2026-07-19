# Design

## Observed behavior

`output/20260719-175431`ではP1がOvertake ShiftOut中に8周期連続でOSQP失敗した。
P2は停止中に300周期、走行中に2周期と1周期のOSQP失敗があった。
一方、最も大きい速度低下はSafetyBrakeの0 m/s上限と、OvertakeからFollowへ戻る際の
3 m/s上限で発生し、OSQP失敗を伴わない速度低下もあった。

## Approach

1. OSQP wrapperは成功結果だけでなく失敗detailを返す。
2. validation/setup/solve/status/solution/constraint各stageを区別する。
3. solver情報が存在する場合はstatus文字列、status value、iteration、primal/dual residualを記録する。
4. 動的速度上限は現速度から`abs(a_min) * Ts`ずつ下げたreachable envelopeとしてMPCへ渡す。
5. 速度上限そのものとpublished accelerationの`a_min`制限は変更しない。

## Compatibility

- ROS topic/service/message契約は変更しない。
- `v2x_safety_brake_velocity=0.0`と`v2x_follow_velocity=3.0`の最終値は維持する。
- P1 start-grid lateral filterとGate2 LowSpeedAvoidance direct controlは変更しない。
- 変更は参加者package内に閉じる。

## Verification result

- `make autoware-build`: 成功（25 packages）
- `test_mpc_velocity_limit`: 4 tests全件成功
- `test_start_grid_grace`: 14 tests全件成功
- 採用run: `output/20260719-181214`
  - P1: 走行中に8周期の最大iteration到達、停止直前に4周期。詳細は
    `status=maximum iterations reached`, `status_val=-2`, `iter=4000`と判明した。
  - P2: 修正前の走行中2周期+1周期のOSQP失敗が0件になった。
  - P3: OSQP失敗0件を維持した。
  - 最大速度低下はP1約1.32 m/s/s、P2約1.20 m/s/sで、既存`a_min=-1.35 m/s^2`以内だった。

QP目的関数の共通スケール正規化も`output/20260719-181720`で比較したが、
P1の最大iteration到達を解消せずP2にも8周期の失敗が出たため不採用とし、最終ソースから除外した。

今回の変更は速度上限の不連続と原因不明ログを正式に修正した。P1に残る失敗は、
Overtake ShiftOut中の横制約/数値収束に範囲を絞れた別課題として扱う。
