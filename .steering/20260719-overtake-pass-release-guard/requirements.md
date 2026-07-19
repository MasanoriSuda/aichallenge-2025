# Requirements

## 目的

Start grid grace終了後の通常走行で、追い越し横移動が完了する前にPassへ移行して
前方車基準の速度上限が解除され、SafetyBrakeまたはRecoveryへ落ちる問題を修正する。

## 要件

- Start grid graceの動作と開始直後の判定は変更しない。
- ShiftOutからPassへの移行には、最低移動距離と横目標への到達を両方要求する。
- Passへ入ってもlocked targetがまだ前方にいる間は前方車基準の速度上限を維持する。
- ShiftOut中は残り横移動時間と前方距離余裕からclosing speedを縮める。
- 追い越し開始後の一時的なgap再評価不成立だけでは、locked targetへの横移動を即中断しない。
- SafetyBrake、EmergencyBrake、solver failure、position jump、明示的追越禁止WP、hard curve、
  completion不可は従来どおり中断条件として維持する。
- ROS 2 topic/service/messageと評価インターフェースは変更しない。

## 完了条件

- 移動距離だけを満たし横偏差が残る場合、ShiftOutを維持する単体テストが成功する。
- locked targetが前方にいるPassでは速度上限を維持し、横並び以降にだけ解除する。
- 安全条件内のactive passはgapの一時消失をHoldし、hard条件ではRecoveryする。
- `test_v2x_overtake_core`と`make autoware-build`が成功する。
