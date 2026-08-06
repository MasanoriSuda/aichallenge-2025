# Requirements

## 目的

`output/20260806-223349` で確認した、前車より遅い状態から完全Missionの机上成立だけを根拠に
ShiftOut/Passへ入り、前後関係を反転できないままSafeSeparationまたはRecoveryへ落ちる事象を減らす。

## 対象事象

- 自車約2.38 m/s、対象約2.94 m/s、前方距離約6.0 mでPassを開始した。
- Mission rolloutは対象速度一定を仮定してrear-clear成立を予測したが、実走では対象も加速し、
  約5秒後も対象が6〜8 m前方に残った。
- 現行のentry speed gateは、完全Mission成立時の`validated_mission_ready`で迂回されるため、
  この速度不足を止められない。

## 制約

- 横経路が未成立の対象へ速度だけで突進しない。pre-armは完全Missionと非Emergencyの前方条件が
  現周期でも成立している間だけ有効にする。
- pre-arm中はShiftOut/Passを開始せず、基準走行線を維持する。
- 前車との実測相対速度が正の閾値を連続確認できてから横Missionへhandoffする。
- start-grid breakoutは専用の観測・corridor検証を持つため、既存の即時handoffを維持する。
- `a_max=1.0 m/s^2`、ROS 2 topic/service、評価結果schemaは変更しない。

## 完了条件

- 完全Mission成立だけではentry speed gateを迂回しない。
- 速度未達かつ完全Mission成立時は、横移動なしのpre-armとして縦加速する。
- pre-arm中はgeneric Follow capとfollow gap/prepositionが横・縦ownershipを奪わない。
- 相対速度条件を連続確認後、最新周期で再検証したMissionだけを実行する。
- Emergency、Mission不成立、target変更ではpre-armを解除する。
- core unit testとpackage buildが成功する。
