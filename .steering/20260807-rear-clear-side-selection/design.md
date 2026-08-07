# Design

## 基本方針

「外まくり」または「イン差し」を固定の優先規則にしない。物理的に成立した候補のうち、
rear-clear まで全幅切替を必要とせず、短時間かつ速度を維持して完遂できる側を選ぶ。

## コース役割

各候補について基準軌道の曲率を rear-clear とその先の予約距離まで走査し、次を記録する。

- 入口側の最初の有意な役割: `Inner` / `Outer` / `StraightOrUnknown`
- rear-clear 付近の最後の有意な役割
- 入口外側が rear-clear 付近で内側になるか
- 入口内側が rear-clear 付近で外側になるか
- 最初の役割反転距離

直線では役割を新しく決めず、次に現れる有意な曲率を入口役割として使用する。短い曲率ノイズは
既存の `v2x_overtake_max_curvature` 閾値以下として無視する。

## 候補順位

既存 hard gate 通過後、機能有効時は以下を候補比較へ追加する。

1. body-clear deadline と最低 slack
2. rear-clear 前に全幅切替が不要
3. 物理余裕に設定値以上の差がある場合は余裕の大きい候補
4. 既存 horizon progress（rear-clear 時間、最低速度、closing、横移動、横加速度）
5. rear-clear 時間
6. direct pass、body-clear、横移動など既存 tie-break
7. それでも同等なら rear-clear 付近で外側になる候補

入口内側から出口外側へ自然に変わる候補は全幅切替不要として扱う。入口外側から出口内側へ変わる
候補は棄却せず、反対候補も成立する場合の優先度を下げる。

## 設定

- `v2x_overtake_rear_clear_side_selection_enabled`
- `v2x_overtake_rear_clear_role_reserve_distance`

初期予約距離は 2.0 m とし、予測 rear-clear の境界直後にある曲率反転も選択時に認識する。

## ログ

選択候補へ入口役割、rear-clear 役割、反転距離、全幅切替要否を出力する。
