# Design

## 原因

現行の stepwise escape は候補の接触セル削減量を最優先し、基準経路からの横偏差を評価しない。そのため壁際では、接触を減らしてもコース外へ離れる Reverse 候補を連続選択できる。コースから離れすぎて waypoint 対応を失うと候補方向が `Unknown` となり、aggressive retry が同じ SAFE_STOP を反復する。

## 方針

1. 純粋関数で rollout 終端のワールド座標変位を Frenet 横方向へ射影する。
2. 現在の横偏差が `rejoin.max_lateral_error_m` を超えるときだけ course-progress guard を有効化する。
3. guard 有効中は、微小な数値許容差を除き `abs(e_y)` を増やす候補を棄却する。
4. 接触削減量が同じ候補では、横偏差の改善量が大きいものを優先する。
5. Reverse-only の原因が一時的な前方障害物だけで、厳密な後退必須条件がない場合は、同じ footprint/V2X gate を通る前進候補も評価する。

## 互換性・安全性

- 既存 YAML の `rejoin.max_lateral_error_m` を再利用し、新規パラメータは追加しない。
- 中心付近の通常 Recovery 選択は変更しない。
- 安全候補が存在しない場合は停止を維持し、未検証方向への強制移動は行わない。

