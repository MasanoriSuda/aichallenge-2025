# Requirements

## 目的

追い越し中の rolling Frenet-DP 更新が、旧実行経路の prefix/tail を残したために壁・横加速度・相手車制約の実行 horizon 検証へ失敗する場合、現在計測状態から再基準化した候補を同じ周期で再評価し、実行可能なら原子的に置換する。

## 背景

- `20260816-081446` では target-bound 付き候補は 18/18 生成できた。
- しかし通常更新は旧 prefix/tail を保持した全件で execution horizon が不成立だった。
- target-bound hold 中に限って行われた measured-state rebase は 1 件成功し、実行経路へ昇格できた。
- 現状は「新しい候補は成立しているが、古い経路を残すことで更新できない」状態が残る。

## 要求

1. 通常の旧経路 stitch を第一候補として維持する。
2. 通常候補が atomic promotion に失敗した場合、同一 target・同一 side・連続した予測が有効で hard fault が無ければ、現在 `e_y` から再基準化して一度だけ再評価する。
3. 再基準化候補にも壁、静的壁、横加速度、相手車の時系列 physical separation、target continuity の全検証を適用する。
4. 再基準化候補も不成立なら、既存の last-feasible path を変更しない。
5. 実接触、壁 margin 違反、壁情報欠損、EmergencyBrake、solver recovery、禁止 waypoint では再基準化しない。
6. 機能は yaml パラメータで切り戻せるようにする。
7. ROS 2 topic/service/message、評価成果物の契約は変更しない。

## 非対象

- 本格的な非線形 MPCC solver への全面置換
- no-return 後の左右切替
- Recovery / Reverse の変更
- 評価基盤 (`aichallenge_system`) の変更
