# Tasklist

- [x] 最新走行の失敗経路を整理する
- [x] 横経路延長と縦 horizon 更新の責務を分離して設計する
- [x] core に縦方向 refresh action を追加する
- [x] controller に固定横目標の縦方向 refresh を実装する
- [x] core 単体テストを追加・更新する
- [x] `git diff --check` を実行する
- [x] 対象テストを実行する
- [x] `make autoware-build` を実行する
- [x] 検証結果と実走確認項目を記録する

## 検証結果

- `git diff --check`: 成功
- `make autoware-build`: 25 packages 成功
  - `multi_purpose_mpc_ros` の setuptools deprecation warning のみ
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core`
  - 成功
- `colcon test-result --verbose`
  - 829 tests / 0 errors / 0 failures / 0 skipped
  - 過去成果物 `build/joycon_contract_guard/package.xml` の欠損警告は出るが、
    今回対象テストの失敗ではない

## 次回実走で確認する項目

- `Pass longitudinal horizon refreshed` が発生すること
- 1 mission 内で横目標 `goal` が変わらないこと
- `bounded Pass horizon exhausted` が減ること
- `Pass -> Return` が発生すること
- 壁・横加速度 preflight 不成立時は従来どおり refresh を拒否すること
