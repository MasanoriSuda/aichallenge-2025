# Design

## 方針

新しい追い越し例外処理は増やさず、既存の成立判定を一つの不変条件へ揃える。

1. `OvertakeMissionCandidate` の `pass_target_clearance_checked` が true の場合、
   `predicted_minimum_pass_target_surface_clearance_m >= 0` を selector の数値・物理成立条件に
   含める。
2. complete shadow と receding-prefix shadow の両方で、負の対車クリアランスを
   `HardConstraint` とする。
3. `evaluate_overtake_line_entry_preflight()` が static-map footprint、壁余裕、横加速度を
   適用して得た `horizon.target_ey` を exact lateral profile として扱う。
4. その profile を既存の `constrain_frenet_dp_corridor_to_target()` に渡し、同じ
   distance-to-time 変換と target prediction で physical separation を再検証する。
5. complete Mission と bounded progressive prefix の候補生成から、この joint horizon
   context を渡す。入力が不正または予測が利用不能なときは従来どおり既存判定へ委ねる。

## 影響範囲

- `multi_purpose_mpc_ros/src/mpc_controller_cpp.cpp`
  - entry preflight の joint target-wall validation
  - complete Mission 実行資格の対車クリアランス条件
- `multi_purpose_mpc_ros/src/v2x_overtake_core.cpp`
  - selector / complete shadow の hard constraint 統一
- `multi_purpose_mpc_ros/test/test_v2x_overtake_core.cpp`
  - 回帰テスト

設定値、topic 契約、評価基盤は変更しない。

## 非目標

- robust margin の攻撃化・保守化
- Recovery FSM の変更
- 新規 solver または全面的 MPCC 置換
- 接触ペナルティ後の速度復帰変更
