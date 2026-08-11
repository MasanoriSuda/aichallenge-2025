# Requirements

## 目的

追い越し実行中の一時停止状態 `FollowPrepare` に関する状態と判定を整理し、
次の性能修正で「再開・別候補選択・終了」を追加しても既存の判定が分散しない
構造にする。

## 対象

- `ShiftOut` / `Pass` / `Recovery` のどこから `FollowPrepare` に入ったかを保持する
- pause の期限、hard fault、rear-clear を一つの pure function で分類する
- controller は pure function の結果を既存の遷移へ変換するだけにする
- 分類の優先順位を単体テストで固定する

## 制約

- 追い越し可否、速度、距離、クリアランスなどのパラメータは変更しない
- `FollowPrepare` からの再開条件や再開先は変更しない
- ROS 2 topic / service / message 契約は変更しない
- `aichallenge_system` は変更しない

## 完了条件

- pause origin が状態として保持され、ログで確認できる
- timeout/distance expiry、hard fault、rear-clear、hold の分類が pure core に集約される
- 既存優先順位を覆う組合せテストがある
- 対象 package の build と test が成功する
