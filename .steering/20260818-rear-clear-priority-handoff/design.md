# Design

## 現象

`v2x_overtake_line_return_clear_distance=2.0 m`、確認時間0.10秒に対し、最新走行では
target_s=-1.44 m / -1.93 mで将来軌道のphysical revalidationが失敗し、約0.2秒後の
rear-clear成立より先にPassからFollowPrepareへ遷移した。これによりprogress-contouringと
warm startが一度解除され、完了済みに近い追い越しを再構築している。

## 方針

1. Pass、SideBySideCommitted、forward-completion latchを確認する。
2. targetが後方、現在bodyが分離、corridorが有効であることを確認する。
3. predicted sweepが分離しているか、既存SafeSeparationの新鮮な前進実績があることを要求する。
4. rear-clear残距離＋確認時間中の走行距離＋path解像度分だけ、現在横位置を保つ短期horizonを
   現在のwall/footprint/lateral-acceleration条件で再検証する。
5. target/physicalの将来replan failureだけをbounded Pass holdへ変換する。
6. 次周期以降にrear-clearが確認されたら既存のvalidated Returnを使う。

## 局所リファクタ

rear-clear後のReturn待ちとrear-clear直前のholdで重複するcurrent-side horizon生成を、
controller内の単一lambdaへ集約する。状態機械やRecoveryの責務は変更しない。

## 非対象

- ShiftOut中のphysical failure
- 実壁接触・実壁余裕違反
- rear-clear閾値そのものの緩和
- Return経路の安全条件緩和
