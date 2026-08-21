# Tasklist

- [x] 最新試走のdecision traceとstuck開始を照合する
- [x] インターフェース契約と既存GapPlanner callerを確認する
- [x] 要件・設計・非対象を文書化する
- [x] forced-side transition policyをpure function化する
- [x] GapPlannerへ連続prefix/gatewayを実装する
- [x] DynamicEscape alternateだけへtransition期限を接続する
- [x] 即時重大壁リスクのprimary抑止を実装する
- [x] decision traceへtransition/suppression情報を追加する
- [x] 単体テストを追加する
- [x] package build/testを実行する
- [x] 動的確認項目を記録する
- [x] 変更をコミットする

## Verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 32/32成功、1483 tests、0 failures
- `test_v2x_overtake_core`: 797/797成功
- `test_overtake_decision_trace`: 8/8成功
- `git diff --check`: 成功

## 次回試走で見る項目

`Overtake decision trace`のalternateについて、次の順で確認する。

1. `side_transition=1/1`となり、gatewayが見つかること
2. `side_transition_gateway=<index>@<distance>m`が6 m以内であること
3. `branch_selection=lower-risk-tier`または`corridor-reserve-advantage`で反対側が採用されること
4. gatewayが無い場合、`forced-side-transition-expired`または
   `forced-side-transition-incomplete`へ分類されること
5. 元経路が開始点から壁余裕を消費し、代替も無い場合、
   `primary_suppressed=1/immediate-wall-threat-without-alternate`となること
6. 変更前に見られた`branch_selection=invalid-side`が、単なるalternate不成立時には
   `alternate-unusable`へ変わること
