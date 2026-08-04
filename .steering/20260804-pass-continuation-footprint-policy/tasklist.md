# Tasklist

- [x] 最新ログと現行継続 preflight の失敗条件を照合する
- [x] 初回 admission と longitudinal continuation の安全境界を設計する
- [x] core に Pass continuation policy resolver を追加する
- [x] controller の preflight policy を分離する
- [x] longitudinal refresh に footprint policy の診断ログを追加する
- [x] core 単体テストを追加する
- [x] `git diff --check` を実行する
- [x] 対象テストを実行する
- [x] `make autoware-build` を実行する
- [x] 検証結果と次回実走項目を記録する

## 検証結果

- `git diff --check`: 成功
- `make autoware-build`: 25 packages 成功
  - `multi_purpose_mpc_ros` の setuptools deprecation warning のみ
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core`
  - 成功
- `colcon test-result --verbose`
  - 832 tests / 0 errors / 0 failures / 0 skipped
  - 過去成果物 `build/joycon_contract_guard/package.xml` の欠損警告は出るが、
    今回対象テストの失敗ではない

## 次回実走で確認する項目

- longitudinal refresh 成功ログで `footprint_policy=1` が出ること
- `outer_role_reversal=1` でも同一横目標の refresh が成功すること
- `rear_clear_refresh` 起因の SafeSeparation が減ること
- `Pass -> Return` が増え、中心間離隔不足だけの Recovery が減ること
- wall / lateral acceleration / footprint overlap による拒否は維持されること
- 6 周で 80 秒超ラップと合計時間が減ること
