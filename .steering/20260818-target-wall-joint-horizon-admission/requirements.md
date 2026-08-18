# Requirements

## 背景

`output/20260818-194513` では、wall-aware extended MPCC により直接の壁余裕違反と
solver failure は減少した。一方、完全 Mission の候補採用後に
`physical target separation conflicts with wall bounds` が繰り返され、
DynamicMissionWait / Recovery へ遷移している。

現行実装では、短い receding-prefix は予測対車クリアランスが負なら hard constraint
として棄却するが、完全 Mission の候補選別と shadow admission は有限値であることしか
要求しない。また、static-wall preflight が補正した最終 lateral profile と前車の時系列
footprint は同一 horizon 上で再検証されていない。

## 要求

- 予測最小対車クリアランスが負の完全 Mission を採用しない。
- complete Mission と receding-prefix で対車 hard constraint の意味を統一する。
- wall/横加速度 preflight が採用した最終 lateral profile を、同じ到達時刻モデルの前車
  footprint と重ねて検証する。
- hard physical separation は緩和しない。追加の robust reserve は候補順位のための soft
  指標として維持する。
- 新しい Behavior/OvertakeLine FSM state は追加しない。
- ROS 2 topic、service、message、提出インターフェースを変更しない。
- `aichallenge/result-summary.json` を変更・コミットしない。

## 完了条件

- selector が負の予測対車クリアランス候補を除外する単体テストが通る。
- complete shadow admission が同候補を hard constraint として棄却する単体テストが通る。
- target と wall の個別成立ではなく、wall-adjusted profile の共同成立を entry preflight が
  要求する。
- 対象 package のビルドと単体テストが通る。
