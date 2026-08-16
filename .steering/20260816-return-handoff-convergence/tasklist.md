# Tasklist

- [x] `20260816-101204`のReturn完了後ログを確認する
- [x] 現行Return完了条件とReturn参照の寿命を確認する
- [x] steering requirements/designを作成する
- [x] 収束ベースhandoff pure policyと単体テストを追加する
- [x] controllerのReturn lifecycleへ適用する
- [x] config、起動ログ、イベントログを追加する
- [x] build / testを実行する
- [x] 変更をコミットする

## 検証結果

- `make autoware-build`: 25 packages successful
- `V2XOvertakeCoreReturn.*`: 4/4 passed
- `test_v2x_overtake_core`: 663/663 passed
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 targets passed
- `colcon test-result --verbose`: 1205 tests、0 errors、0 failures、0 skipped
- `test-result`は既存のstale `build/joycon_contract_guard/package.xml`をskipする診断を出すが、対象packageの結果は成功
