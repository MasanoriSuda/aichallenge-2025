# Design

## 方針

`PassRefreshFailureReason`を利用し、`ExecutionCorridorBlocked`、`WallOrBodyFault`、`Other`のうち、現在の車体・予測sweep・壁・solver・target continuityが安全な場合だけbounded replan graceへ入る。

graceは新しい速度モードではない。通常Passの速度referenceをそのまま維持し、同じsideとMission generationのまま毎周期の経路更新を再試行する。

graceを許可した後も、既存の`evaluate_overtake_line_horizon()`が現在のMPC実行ホライズンを同じ周期で検証する。現在経路が壁clamp、物理footprint、横加速度上限のいずれかで実行不能なら、従来どおり即Recoveryへ遷移し、不成立経路はMPCへ渡さない。したがってgraceが有効になるのは「将来の延長・更新は不成立だが、現在のコミット済みprefixはまだ成立」の場合だけである。

## 状態

- grace開始時刻
- grace開始時のPass走行距離

## 終了条件

- Pass経路更新成功
- 時間または距離上限
- コミット済みstatic horizon到達
- target/course progress不連続
- 現在車体または予測sweep不成立
- 壁、EmergencyBrake、solver recovery
- phase変更

grace終了後は既存のprediction lease、SafeSeparation、Recoveryの順序へ戻す。

## ログ

- `Pass refresh replan grace started`: 通常Pass速度のまま再計画開始
- `Pass refresh replan grace resolved`: 経路更新成功
- `Pass refresh replan grace resolved without replacement`: rear-clearまたは既存prefix内で解決
- `Pass refresh replan grace ended`: 上限・安全境界へ到達し既存fallbackへ移行

## 非対象

- 左右candidate rankingの変更
- MPCの加速度上限変更
- 壁クリアランスparameter変更
- SafeSeparation自体の削除
