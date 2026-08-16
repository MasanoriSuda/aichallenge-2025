# Requirements

## 目的

rolling Frenet DP が実行上限ちょうどの横加速度を使い、補間後の実行検証で微小補正されて atomic promotion を失う不整合を減らす。

## 要求

- 車両の実行上限 `v2x_overtake_line_max_lateral_accel` は変更しない。
- DP の到達可能区間だけに設定可能な横加速度余裕を持たせる。
- 後段の実行検証、atomic promotion、last-feasible path 保持は緩和しない。
- 設定値と実効DP横加速度を起動ログ・候補棄却ログで確認可能にする。
- ROS 2 topic、message、launch、評価結果 schema は変更しない。
- ローカル用と提出用 config は同一設定にする。

## 制約

- 変更は `aichallenge_submit/multi_purpose_mpc_ros` に閉じる。
- MPC不収束、Reverse、Recoveryの戦略変更は今回の対象外とする。
- 2025由来の競技シミュレーション向け暫定設定とする。

## Definition of Done

- execution envelope pure policy が余裕率を検証し、実効横加速度へ反映する。
- 90%設定時、6.0 m/s²の実行上限に対して5.4 m/s²でDP到達可能区間を作る。
- 既存の後段6.0 m/s² hard validationは維持される。
- 対象packageがビルドし、関連テストが成功する。
- 変更をコミットする。
