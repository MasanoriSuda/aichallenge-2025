# Tasklist

- [x] 最新ログから pre-arm -> SafetyBrake -> reverse-only の経路を特定
- [x] 通常確認を維持する限定的な早期開始条件を設計
- [x] 純粋判定と単体テストを追加
- [x] controller 設定、admission、ログへ統合
- [x] config.yaml に既定値を追加
- [x] package build/test
- [x] 差分と既存ユーザー変更を確認

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core --output-on-failure`: 成功
- `colcon test-result --verbose --test-result-base build/multi_purpose_mpc_ros`: 921 tests、0 errors、0 failures
- `git diff --check`: 成功
- `aichallenge/result-summary.json` の既存ユーザー変更には未介入

## Definition of Done

- 今回ログ相当の 3.55 m / 1.69 m/s / relative 1.10 m/s / stable 0.05 s を許可する。
- stable 0、front speed超過、距離範囲外、Mission/hard guard不成立を拒否する。
- `test_v2x_overtake_core` が成功する。
- `multi_purpose_mpc_ros` がビルドできる。
- 実走で新ログと `Follow -> Overtake` を確認できる状態にする。
