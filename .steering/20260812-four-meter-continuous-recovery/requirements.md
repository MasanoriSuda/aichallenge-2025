# Requirements

## 目的

最新の `make dev2` で、最初のStuck Recoveryが約82秒継続し、短い
ReverseとSafeStop/aggressive retryを反復した。初回から4 mを狙う連続Reverseと、
再試行で消えない物理incident履歴により、同じ復帰戦略の反復を減らす。

## 要件

- simulation-only Recoveryの初回Reverse目標を4.0 mにする。
- 5 m未満の前進で再スタックした場合は4.0 mから8.0 mへ拡張する。
- 後方のswept footprintとV2X corridorが成立する場合は連続Reverseを優先する。
- 初期接触中も、4 m rollout全体で新規接触を作らず接触量を減らせる場合は
  0.4 m stepwiseではなく連続Reverseを許可する。
- aggressive retryやrejoin再評価で、物理incidentの累積距離、時間、retry、gear要求を
  リセットしない。正常前進5 mまたはrace session resetでのみincidentを終了する。
- 物理的に成立する40 Hz・2.0 m/s・操舵付きReverseをpose jumpと誤判定しない。
- Reverse/Driveのgear要求を最大3回までidempotentに再送できるようにする。
- 評価基盤、topic/service、result JSON契約は変更しない。

## 対象外

- Overtake candidate fault isolation
- V2X duplicate timestamp対応
- ContactContinuation acquisition/hold再設計
- Recovery MPCの本番有効化

