# Tasklist

- [x] 最新走行のentry開始距離とpre-arm timeoutを確認する
- [x] 30 m早期計画とpre-arm継続条件を設計する
- [x] ローカル・cloud設定とコード既定値を同期する
- [x] 対象packageをビルドする
- [x] core単体テストを実行する
- [ ] 次回`make dev2`で動的効果を確認する（ユーザー実施）

## 静的確認結果

- `make autoware-build`: 25 packages成功。
- `colcon test --packages-select multi_purpose_mpc_ros --ctest-args -R test_v2x_overtake_core --output-on-failure`: 成功。
- `colcon test-result --test-result-base build/multi_purpose_mpc_ros --verbose`: 1009 tests、error/failure/skip 0。
