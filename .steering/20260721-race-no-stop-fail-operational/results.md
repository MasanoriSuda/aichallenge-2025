# Results

## Implementation

- 前方車なしのV2X Cruiseに限定したsimulation-only solver failure crawlを追加した。
- crawl操舵は既存の低速横偏差・heading feedbackで基準線へ戻す。
- 停止中の連続solver fallbackではAWSIM pose correction / waypoint progress jumpを走行再開と扱わない。
- coordinated stopは初回Reverse-first後の明示失敗でForward fallbackを解禁する。
- aggressive simulation recoveryでは接触セルを増やさないbounded候補まで探索する。
- Forward creepを1.0 m/s、1.0 m/s2、最大2.0秒へ変更した。

## Verification

- `make autoware-build`: success、25 packages。
- `colcon test --packages-select multi_purpose_mpc_ros`: success。
- `colcon test-result --verbose`: 568 tests、0 errors、0 failures、0 skipped。
- `colcon test-result`は無関係な古い`build/joycon_contract_guard/package.xml`欠損を警告したが、対象test resultには影響しなかった。

## Remaining runtime check

次の`make dev3`で以下を確認する。

- `MPC solver fail-operational crawl entered/exited`が前方車なしのsolver failureだけで出ること。
- D1/D2が`maneuver_direction_unknown`を反復せず、Reverse不成立後に`forward_fallback=1`となること。
- 全車が恒久停止せず走行を再開すること。
