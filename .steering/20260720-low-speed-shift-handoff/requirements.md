# Requirements

## 目的

停止・低速車両を直接操舵で回避した後、最初に選んだ通過横位置が残り続けて
MPC復帰が遅れる問題を解消する。

## 要件

- `v2x_low_speed_avoidance_shift_velocity=3.0 m/s` は維持する。
- 前方・側方・clearance対象が残る間は、直接操舵を解除しない。
- 対象車両列が設定時間clearになったら、通過横位置ではなく現在車線の有効範囲内へ
  再合流する。
- 再合流姿勢が整っても、MPC solverが成立しない周期では直接操舵を維持する。
- solver成功時だけMPCへ所有権を渡す。
- V2X topic、control topic、launch、result schemaは変更しない。

## Definition of Done

- 車両列clear後に再合流フェーズへ移る単体テストが通る。
- vehicle再検出時にpass targetへ戻る実装となる。
- handoff solver failureで通常の連続failure fallbackへ入らない。
- 対象パッケージのテストと`make autoware-build`が成功する。

