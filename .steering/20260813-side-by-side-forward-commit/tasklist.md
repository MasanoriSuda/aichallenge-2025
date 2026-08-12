# Tasklist

- [x] 最新走行の並走・接触・SafetyBrake連鎖を確認する
- [x] ContactContinuationの既存速度・操舵所有経路を確認する
- [x] entry/release横速度ヒステリシスを設計する
- [x] core分類器、controller、ローカル/cloud設定を変更する
- [x] 分類器の境界テストを追加する
- [x] 対象packageをビルドする
- [x] core単体テストを実行する
- [ ] 次回`make dev2`で動的効果を確認する（ユーザー実施）

## 静的確認結果

- `make autoware-build`: 25 packages成功。
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core --output-on-failure`: 成功。
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`: 1009 tests、error/failure/skip 0。
- 境界確認: inactive時0.6 m/sは不採用、active時0.6 m/sは保持、0.81 m/sは解除。
