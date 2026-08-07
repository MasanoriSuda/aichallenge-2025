# Design

## 1. locked target幾何の観測範囲

速度・front brakeの所有権に使う`active_overtake_execution`はShiftOut/Passのまま維持する。
別にlocked Mission targetの幾何観測条件を設ける。

観測対象は次のいずれかとする。

- ShiftOut
- Pass
- frozen Missionを保持したFollowPrepare

これによりFollowPrepare中もcurrent body footprint、1秒予測sweepを更新するが、Pass専用のfront-overlap除外や速度cap解除は有効化しない。

## 2. dynamic wait入場条件

dynamic waitは「現側を保持しながら反対側Missionも比較できる状態」に限定する。

- active ShiftOut/Pass
- frozen Missionとlocked targetあり
- target continuity有効
- current body footprint非重複
- footprint prediction有効
- targetがno-returnより前
- alternate replacement回数に余裕あり
- hard faultなし
- rear-clear未成立

条件を満たさない場合は、呼び出し元の既存SafeSeparation／Recovery判断を使う。

## 3. no-return診断

`opp_no_return`は`target_s < no_return_front_distance`だけを表す。
幾何未観測、target不連続、replacement回数切れは`opp_reason`と`opp_eligible`で区別する。

## 4. 回帰防止

pure coreへ次を追加して単体テストする。

- paused frozen Missionを幾何観測対象へ含める判定
- dynamic wait入場判定
- no-return後、予測無効、車体重複、replacement回数切れの拒否

## 非対象

- no-return距離3.5 mのパラメータ変更
- lateral acceleration、wall clearance、closing speedの変更
- 複数lateral knotを持つ新planner
