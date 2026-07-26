# Design

## 原因

事故後のRecoveryは一度Reverseを実行した後、後壁が明示されている状態へ
変化していた。しかし、solver fallback時の大きなheading errorで
`recovery_reverse_only_episode_`がラッチされたままになっていた。

`evaluate_recovery_safety()`のRear wall分岐は、Reverse-onlyでない場合だけ
`ForwardStraight`を評価する。この組合せにより候補を一件も評価せず、
方向UnknownのままSafeStopへ戻っていた。

ログの`static=invalid_grid, checked=0`は実際の地図不正ではない。
`RecoverySafetySnapshot`の初期値が`InvalidGrid`で、候補未評価時に更新されない
ための誤表示だった。

## 退避方向の優先順位

Recovery中に静的地図からRear wallが明示された場合は、
solver fallbackのheading errorによるReverse-only episodeとReverse intentを
解除する。Rear wallから離れる既存のForward方向選択を、過去の状態より優先する。

Front/Side/Mixed/Unknownについては従来のReverse優先とfail-closedを維持する。

## 診断値

静的候補またはrejoin候補を評価する前のreject reasonは`not_evaluated`とする。
実際にgridが不正な場合だけ、評価関数が返す`invalid_grid`を記録する。

## 安全境界

- occupancy gridの形状・原点・セル値は変更しない。
- Forward候補は既存の静的rolloutで評価する。
- V2X corridor、ギア確認、Boost抑制を省略しない。
- Forwardは既存の0.6 m/2.0 s/1.0 m/s上限を維持する。
- 各step後に停止・再評価する既存FSMを維持する。

## 互換性

- ROS interface: 変更なし
- 評価成果物: 変更なし
- パラメータ追加: なし
- 2025由来の現行契約: 変更なし
