# Requirements

## 目的

停止車両を近距離で検出したとき、`LowSpeedAvoidance`と通常`Overtake`の間で
横計画の所有権が失われ、前車へ接触して停止する問題を解消する。

## 要件

- 共通コース進捗上で前方にいる停止車両は、通常の正面重複判定より広い走行回廊から
  停止車両回避候補として検出する。
- 近距離で停止車両を検出した場合も、通過可能なgapがあれば停止車両回避へ入る。
- 通常`Overtake`が停止車両に対して選択された場合、停止車両であることだけを理由に
  `OvertakeLine`を解除しない。
- `SafetyBrake`および停止車両回避が実際に横計画を所有している場合の優先順位は維持する。
- AWSIMレース開始前の待機時間で回避状態、line、stall timeoutを消費しない。
- ROS 2 topic、message、service、launch、result schemaは変更しない。

## Definition of Done

- 3〜6 m前方の停止車両が`LowSpeedAvoidance`候補になれる。
- 2.2〜3 mで通常`Overtake`が成立した場合、`OvertakeLine`が横計画を保持する。
- 前方危険判定は従来の狭い重複範囲を維持し、横に離れた停止車両だけで
  `SafetyBrake`へ入らない。
- 純粋判定関数の境界テストと対象パッケージのビルド・テストが成功する。
