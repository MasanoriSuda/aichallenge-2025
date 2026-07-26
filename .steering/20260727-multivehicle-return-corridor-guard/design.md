# Design

## 復帰経路占有判定

各V2X車両を基準経路座標へ投影し、現在の自車横位置から基準線 `e_y=0` までの
横方向sweepを車両inflation分だけ広げる。ロック対象以外の車両がこのsweep内にあり、
かつ自車前後の近傍範囲にいる場合、復帰経路をoccupiedとする。

投影が使えない場合は、現在の自車 `e_y` とローカル相対横位置から車両横位置を求める。
縦方向は共通経路距離を優先し、利用できない場合はローカル相対距離を使う。

## 状態遷移

- Pass完了条件を満たしてもoccupiedならPassを維持する。
- 壁余裕のみが不足した場合も、物理接触がなく復帰経路がoccupiedなら、
  static-mapでclampされたPass目標を維持する。
- Return開始直後にoccupiedへ変化し、Return進捗が既存のreacquire上限以下ならPassへ戻す。
- 復帰待ち中は `v2x_overtake_recovery_velocity` を速度上限に使う。
- 物理壁接触、static-mapの物理経路不成立、緊急制動は従来のRecovery/SafetyBrakeを維持する。

## ログ

復帰経路のBlocked/Released変化時だけ、blocker ID、縦距離、横位置、phaseを出す。
周期ログは追加しない。

## パラメータ

直前の実験値を以下へ戻す。

- `v2x_overtake_guard_max_lateral_accel`: 5.0 -> 6.0
- `v2x_overtake_line_shift_distance`: 5.0 -> 4.0
- `v2x_overtake_line_min_wall_clearance`: 0.3 -> 0.2
