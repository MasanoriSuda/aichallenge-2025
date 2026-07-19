# Requirements

## 目的

追い越しのPass開始後、先読みsoft curveでロック済みpass sideが内側と判定された瞬間に
`Overtake -> Follow -> Recovery`へ落ちる挙動をA/B評価する。

## 要件

- 新規追い越しでは従来どおり内側passを禁止する。
- 設定を有効にした場合だけ、開始済みOvertakeのロック側をsoft curve内でも継続できる。
- 明示禁止WP、hard curvature、curve cooldown、EmergencyBrakeでは継続を許可しない。
- gap幅、横移動到達性、wall clearance、SafetyBrakeを維持する。
- 設定省略時は従来挙動を維持する。

## 成功条件

- 前回run `output/20260719-185846` のD2 WP148相当で、soft-inner判定だけを理由に中断しない。
- hard curveまたはEmergencyBrakeではOvertakeが継続されない。
- solver failureや接触停止が増えていないかをログで確認する。
