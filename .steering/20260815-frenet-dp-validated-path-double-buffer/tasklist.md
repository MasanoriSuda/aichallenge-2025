# Tasklist

- [x] 実行中 DP state に candidate validation clock を追加
- [x] rolling refresh candidate の current-state horizon prevalidation を追加
- [x] prevalidation 成功時だけ active path と runtime lease を原子的に更新
- [x] 棄却時に active path/lease が不変であることをログ化
- [x] continuous DP wait を committed forward execution として保持
- [x] runtime validation lease を 0.30 s に更新
- [x] core unit tests を追加・更新
- [x] package build/test
- [x] 変更をコミット

## Definition of Done

- 未検証 refresh が active path と runtime validation timestamp を消さない。
- hard fault のある candidate は active path へ昇格しない。
- `continuous_dp=1` の健全な wait は短時間 reselect limit だけで失効しない。
- 通常用と cloud config が一致する。
- 対象 package の build/test が成功する。

## Verification

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets 成功
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`:
  1129 tests、0 errors、0 failures、0 skipped
- `git diff --check`: 成功
