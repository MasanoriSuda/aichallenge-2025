# Requirements

## 目的

rolling Frenet DP の候補を現行経路へ接続するとき、実測横速度を無視した参照を生成して後段の横加速度検証に棄却される不整合を解消する。

## 要求

- measured-state rebase は、現在横位置だけでなく実測横速度から到達可能なprefixを生成する。
- active pathを使う通常refreshも、現在の実測状態から到達不能なstitch結果を出さない。
- raw DP、stitch、実行前validationで同じ等加速度到達可能性モデルを使う。
- DP計画用の横加速度余裕率をstitchにも適用し、実行上限は緩和しない。
- atomic promotionとlast-feasible path保持は維持する。
- rate-limit済みログからstitchの横加速度制約適用を確認できるようにする。
- ROS 2 topic、message、launch、評価結果schemaは変更しない。

## 制約

- 変更は `aichallenge_submit/multi_purpose_mpc_ros` に閉じる。
- solver不収束、Reverse、Recovery戦略は今回の対象外とする。
- 既存設定 `v2x_overtake_mpcc_frenet_dp_execution_lateral_accel_reserve_ratio` を再利用し、新しい調整パラメータは増やさない。

## Definition of Done

- stitch pure policyが実測横速度・速度・計画上限を検証する。
- active pathなしのprefixがゼロ横加速度の予測位置を基準にする。
- stitch全点が計画横加速度上限内へ射影される。
- 無効時は従来のstitch結果を維持する。
- 対象packageがビルドし、関連テストが成功する。
- 変更をコミットする。
