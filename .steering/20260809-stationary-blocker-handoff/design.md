# Design

## 1. Mission total clock initialization

Mission総時間budgetが有効な実行フェーズに入った時、開始時刻が未設定なら現在時刻で
一度だけ初期化する。通常Missionで既にfreezeされた開始時刻は保持する。

これにより、独自のstart-grid breakout経路などMission candidate freezeを経由しない入口でも、
`NaN`をfail-closed abortへ渡さず、設定された15秒budgetを正しく消費する。

## 2. Confirmed stationary blocker handoff

通常の新規Overtakeは、完全Mission成立後も実測相対速度が設定値以上で0.3秒安定するまで
base line上でpre-armする。この待機は走行車には必要だが、Reverse直後など自車も低速な時に
停止車を回避する窓を失う。

次の全条件を満たす場合だけentry-speed gateを省略する。

- 機能設定が有効
- 現周期に完全Missionが成立
- 前方車を観測
- 停止速度閾値以下の観測が必要sample数継続
- 前方距離が通常entry guard以上
- V2X、位置jump、禁止区間、EmergencyBrake、solver recoveryのhard guardがclear

この例外は横経路の成立判定を代替しない。速度差の観測待ちだけを省略する。

## Diagnostics

例外が新規実行を許可した周期に、target、距離、速度、停止確認sample数、pass sideを
INFOログへ出す。

