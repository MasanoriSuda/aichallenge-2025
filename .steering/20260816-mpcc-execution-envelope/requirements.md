# Requirements

## 目的

ShiftOut～rear-clear の rolling Frenet DP が、生成後の実行検証で壁・横加速度制約により棄却され続ける不整合を減らす。

## 要求

- DP の各距離サンプルへ、実行時と同じ車体 footprint による静的壁制約を入力する。
- 現在の横位置・横速度・速度・最大横加速度から得られる到達可能区間を hard bounds にする。
- target の時系列制約、左右 topology、atomic promotion、last-feasible path 保持は維持する。
- 新規解が不成立でも、現在の実行可能解を破棄しない。
- ROS 2 topic、message、launch、評価結果 schema は変更しない。
- ローカル用と提出用 config は同一設定にする。

## 制約

- 変更は `aichallenge_submit/multi_purpose_mpc_ros` に閉じる。
- Recovery / Reverse は既存 FSM に残す。
- 実車向け調整ではなく、2025由来の競技シミュレーション向け暫定実装とする。

## Definition of Done

- 静的壁の連続自由区間を求める pure utility と単体テストがある。
- DP execution envelope の到達可能区間を求める pure policy と単体テストがある。
- 有効時、DP hard/preferred bounds に静的壁、hard bounds に到達性が反映される。
- rolling candidate 棄却ログから実行horizonの具体的な制約原因を識別できる。
- 対象 package がビルドし、関連テストが成功する。
