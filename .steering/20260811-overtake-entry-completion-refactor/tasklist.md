# Tasklist

- [x] 現行entry / rearward completion条件を確認する
- [x] entry stage pure policyを追加する
- [x] rearward completion context pure policyを追加する
- [x] controllerをpure policy利用へ置換する
- [x] pure core testを追加する
- [x] core testを実行する
- [x] 対象packageをbuildする
- [x] 動作非変更をdiffで確認する

## Verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 1020 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功
- param / topic / service / launch契約の変更なし
