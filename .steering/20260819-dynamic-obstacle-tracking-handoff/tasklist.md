# Tasklist

- [x] 最新 run と直前 run の solver failure / authority 遷移を照合する
- [x] GapPlanner から tracking MPC までの所有権経路を確認する
- [x] reachability bridge を実装する
- [x] first-solve qualification と Follow cap の二段階解除を実装する
- [x] solver failure quarantine を実装する
- [x] scoped planner の下流適用を authority 成立時だけに限定する
- [x] core unit test を追加する
- [x] package build / test を実行する
- [x] 実走確認項目を記録する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 758 tests passed
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28 test targets passed

## 次回実走で見る値

- `Dynamic-obstacle lateral escape authority` が `qualified=0 -> 1` へ進むこと
- `active=1` 直後の solver failure 比率が、修正前の `179 / 200` から大幅に減ること
- failure 時に quarantine が発火し、一周期ごとの active/inactive 交互動作にならないこと
- bridge reject 中は通常 Follow を維持し、scoped planner の hard bounds が漏れないこと
- `Pass -> Return -> Idle` の完遂が発生すること
