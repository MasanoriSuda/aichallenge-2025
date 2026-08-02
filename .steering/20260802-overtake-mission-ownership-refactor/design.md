# 設計

## 方針

`v2x_overtake_core` に副作用を持たない mission ownership resolver を追加する。controller は現在の OvertakeLine phase を boolean 入力へ変換し、以下を一度だけ解決する。

- mission が存在するか
- ShiftOut / Pass の committed execution 中か
- FollowPrepare の pause 中か
- Behavior が新規 Entry と continuation のどちらを評価しているか
- locked target に対する縦速度を OvertakeLine と generic Follow のどちらが所有するか

## 挙動互換

resolver は現行の条件式をそのまま集約する。

- continuation = ShiftOut または Pass または FollowPrepare または直前 Behavior が Overtake
- OvertakeLine の速度所有 = ShiftOut / Pass 中かつ nearest front が locked target
- generic Follow の速度所有 = 上記以外

今回は continuation 時の guard を緩和しない。次段で `entry_assessment_active` と `committed_execution_active` を使い分けるための構造だけを作る。

## 変更範囲

- `v2x_overtake_core.hpp/.cpp`: ownership request / resolution / resolver
- `mpc_controller_cpp.cpp`: 分散していた主要 phase・速度所有判定を resolver 結果へ置換
- `test_v2x_overtake_core.cpp`: phase と速度所有権の組合せテスト
