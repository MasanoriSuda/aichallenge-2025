# Requirements

## 目的

dev3で開始済みの追い越しが、hard curvatureをMPC horizon内に検出しただけで即座に
`Follow`へ戻る挙動をA/B評価する。hard curvature閾値は変更せず、hard境界までの残距離と
OvertakeLine進捗から、境界前に対象車を抜ける場合だけ`Pass`継続を許可する。

## 制約

- 新規追い越しのcompletion guardは変更しない
- OvertakeLineが`Pass`以外ならhard curve継続を許可しない
- 明示禁止WP、curve cooldown、EmergencyBrake、SafetyBrakeは緩和しない
- gap到達性、locked side、wall clearanceは毎周期再評価する
- ROS 2 topic/service/message契約は変更しない
- 設定省略時は従来どおりhard curveを即ブロックする

## 完了条件

- pure policyのunit testが通る
- `make autoware-build`が通る
- dev3でWP130〜hard boundaryの遷移理由と完了予測距離を取得する
- 3台のlap、停止、衝突、OSQP fallbackを記録する
