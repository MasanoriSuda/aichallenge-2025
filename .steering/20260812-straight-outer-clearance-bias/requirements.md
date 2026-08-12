# Requirements

## 目的

現行minimum-motionは、通行可能区間の中央ではなくレーシングラインに最も近い
安全点を選び、通常の対象車clearance bufferを0.20 mとしている。このため直線・
外まくりでも対象車寄りの小まくりになり、横接触から並走・壁接触へ進む余地がある。

直線と現在カーブ外側の追い越しだけ、通路に余裕がある場合は対象車からさらに
0.10 m離れたMissionを優先する。

## 要件

- 基本の `v2x_overtake_minimum_motion_preferred_clearance_buffer: 0.20` は維持する。
- 直線および現在カーブ外側だけ0.10 mの追加clearanceを要求する。
- イン差し、start-grid breakout、既存Mission継続は変更しない。
- 追加clearance候補もShiftOut/Pass/Return全体の既存検証を通す。
- 追加分を適用した後も、既存壁余裕に追加分相当のreserveを残す。
- 追加候補が不成立なら同一周期で通常0.20 m候補へ戻す。
- 左右の戦略選択、速度、SafetyBrake、Recoveryの設定は変更しない。
- 適用量とfallback有無をMission選択ログへ出す。

## Definition of Done

- config/cloud configの値が一致する。
- 直線・外側で追加候補、イン側で通常候補が生成される。
- 狭いcorridorでは通常候補へfallbackする。
- 対象packageの単体テストとbuildが成功する。
- 動的効果確認はユーザー試走で、Pass完遂率、横接触、壁Recovery、追い越し時間を
  前走と比較する。
