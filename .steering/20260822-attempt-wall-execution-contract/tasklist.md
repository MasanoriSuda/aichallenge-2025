# Tasklist

- [x] 最新ログと現行実装の因果を照合する
- [x] 修正範囲と非対象を文書化する
- [x] Dynamic Escape exit gate を attempt-scoped にする
- [x] ShiftOut rollback を publish-aware にする
- [x] physical wall 境界許容と決定ログを追加する
- [x] unit test を追加・更新する
- [x] package build/test を実行する
- [x] 変更をコミットする

## Definition of Done

- replacement 採用後、同一 attempt の `entered` が再発しない。
- 未 publish ShiftOut の final wall reject は `entry-rollback` になる。
- 0.01 m 以内の非接触境界差だけを受理し、それ以上の不足は拒否する。
- 上記の判定根拠がログ一行から追跡できる。

## Verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_overtake_execution_orchestrator --output-on-failure`: 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`: 1446 tests, 0 errors, 0 failures

## Dynamic acceptance

次回 `make dev2` では以下を確認する。

- 同じ `attempt/latched` で `replacement-adopted` の直後に `entered` が再発しない。
- replacement が消えた場合は同一契約の `reason=replacement-lost` になる。
- 未 publish の ShiftOut 壁棄却は `action=entry-rollback, command_published=0` になる。
- `boundary_accept=1` は shortfall 0.01 m 以下かつ contact=0/out=0 に限定される。
- 0.256 m、0.114 m 級の不足は引き続き `predicted-wall-clearance` で拒否される。
