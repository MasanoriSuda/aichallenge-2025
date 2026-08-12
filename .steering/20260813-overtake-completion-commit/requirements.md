# Requirements

## 目的

実行中の追い越しMissionで反対側の完全経路が見つかったにもかかわらず、ヘアピンで既に低下している自車速度より高い前車速度を全horizonの最低速度として要求し、`minimum_speed_insufficient`で候補を棄却する不整合を解消する。

## 観測事象

- 最新走行では、反対側候補がrear-clear時間、残時間、壁余裕を満たしていても、予測最低速度1.39 m/sに対して前車速度3.02 m/sを要求して棄却された。
- 上位車ログでは、曲線中に1～2 m/s台まで低下しても追い越しMissionを維持し、rear-clear時の速度回復を重視している。

## 要件

1. cross-side Mission replacementのhorizon最低速度は、前車速度ではなく`min(現在自車速度, 前車速度)`を要求値とする。
2. rear-clear終端速度は従来どおり`前車速度 + terminal closing speed`を要求する。
3. 壁余裕、rear-clear予測、残時間、残距離、no-return、追加遷移preflightの判定は変更しない。
4. 棄却ログに現在自車速度と実際の最低速度要求値を出す。
5. 既存のforward completion、SafeSeparation、last-feasible maneuverを再利用し、重複した状態遷移を追加しない。

## 非対象

- 固定追い越しゾーン
- 20～30 m手前からのearly-arm
- 横オフセットの拡大
- Pass/Recoveryの全面的な状態設計変更

