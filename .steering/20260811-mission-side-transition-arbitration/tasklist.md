# Tasklist

- [x] 実走ログと静的レビューを照合する
- [x] requirements/designを記録する
- [x] Mission-wide cross-side latchを実装する
- [x] prepared Mission再gateと専用パラメータを実装する
- [x] dynamic waitのfault優先順位を修正する
- [x] 回帰・左右対称・fault injectionテストを追加する
- [x] build/testを実行する

## Dynamic verification checklist

- `continuous outer`または`scheduled outer`後のopponent side replacement回数（期待0）
- opponent side replacement後のouter transition回数（期待0）
- `cross-side replacement rejected`のreason
- `Pass -> Return -> Idle`完遂率
- `actual footprint wall margin violated`回数
- dynamic Mission wait中のwall/emergency/solver Recovery遷移

## Static verification result

- `make autoware-build`: success（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: success
- test summary: 1002 tests, 0 errors, 0 failures, 0 skipped
- `git diff --check`: success
- `colcon test-result`は既存の`build/joycon_contract_guard/package.xml`欠損警告を出すが、対象packageのtest resultは全件成功
