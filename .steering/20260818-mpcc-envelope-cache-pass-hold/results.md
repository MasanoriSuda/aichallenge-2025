# Results

## 実装結果

- live MPC に上限4096件の physical wall envelope cache を追加した。
- key は waypoint、探索区間、preferred offset、heading、clearance、sample step で構成した。
- tactical async snapshot は cache を共有せず、controller thread との競合を避けた。
- cache hit時は区間を2 mm収縮して現在boundsと交差し、既存の完全なfootprint再検証を維持した。
- Pass横離隔latch後は、将来target-wall競合だけで短期holdを使い切っていても、Mission絶対
  time/distance上限内なら同側の物理検証済みprefixを保持できるようにした。
- actual wall contact/margin/sample fault、現在車体重複、target不連続、EmergencyBrake、
  solver recovery、forbidden waypointは従来どおり保持を拒否する。

## 静的検証

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28成功
- `colcon test-result --verbose`: 1239 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功

## 次回試走で見る値

比較元は `output/20260818-084301`。

- control callback overrun: 246回から明確に減ること
- ShiftOut callback平均: 7.73 msから低下すること
- Pass callback平均: 5.66 msから低下すること
- `physical target separation conflicts with wall bounds` による
  Dynamic Mission Wait遷移: 19回から減ること
- `Pass -> Return -> Idle` 完遂数: 1回より増えること
- actual wall contact / EmergencyBrakeが増えていないこと

動的な効果確認は未実施。次回 `make dev2` のログで判定する。
