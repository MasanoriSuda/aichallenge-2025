# Tasklist

- [x] 実走ログと置換経路を照合する
- [x] 要件と設計を記録する
- [x] monotonic no-return latchを実装する
- [x] cross-side completion admissionを実装する
- [x] 許可された早期置換をShiftOutとして実行する
- [x] unit testを追加する
- [x] build/testを実行する

## Definition of Done

- `opp_no_return=1`後に反対side replacementが発生しない
- SafeSeparationが反対side全幅切り返しを行わない
- cross-side candidateはtime/distance/speed完遂条件を通る
- 許可された早期置換でPass-unlatched 0.5 m/s policyを誤用しない
- `multi_purpose_mpc_ros`のbuild/testが成功する

## Dynamic verification checklist

- `opponent side PassPlan replacement rejected` reason別回数
- `opp_no_return=1` 後のside replacement回数（期待0）
- side replacement後のphase（許可時はShiftOut）
- side replacement後のclosing capと最低実速
- Pass -> Return -> Idle完遂率
- SafetyBrake / Recovery / wall contact回数

## Static verification result

- `make autoware-build`: success（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: success
- test summary: 998 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: success
- `joycon_contract_guard/package.xml`の既存欠損に関する`colcon test-result`警告は発生したが、対象packageのtest resultには影響なし

動的チェックリストは`make dev2`試走後に確認する。
