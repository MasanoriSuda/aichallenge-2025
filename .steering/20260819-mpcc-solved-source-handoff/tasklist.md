# Tasklist

- [x] 最新走行ログと現行 execution authority 経路を確認する
- [x] 要件と設計を記録する
- [x] solved source handoff の純粋判定を追加する
- [x] 最新 QP 解の atomic promotion を実装する
- [x] Mission reset / telemetry を更新する
- [x] 単体テストを追加する
- [x] 対象 package を build/test する
- [x] コミットする

## Definition of Done

- 新しい solved source だけが promotion される
- stale / same-source / context mismatch / hard fault は promotion されない
- promotion 後の source age は `solved_sec` を基準にする
- 既存インターフェースを変更しない
- build と unit test が成功する

## Verification

- `make autoware-build`: 25 packages successful
- `colcon test --packages-select multi_purpose_mpc_ros`: 28/28 test targets passed
- `colcon test-result --verbose`: 1338 tests, 0 errors, 0 failures, 0 skipped
- final local refactor後に `mpc_controller_cpp` targetを再ビルド: successful
- aggregate result collection still reports the pre-existing missing
  `build/joycon_contract_guard/package.xml`; it does not belong to this change
