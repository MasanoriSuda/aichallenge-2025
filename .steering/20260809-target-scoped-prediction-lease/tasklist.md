# Tasklist

- [x] Proレビューと現行HEADを照合
- [x] target-scoped停止証拠を実装
- [x] typed prediction failureを実装
- [x] lease速度所有権を実装
- [x] core単体テストを追加
- [x] `make autoware-build`
- [x] package test（25/25、集計956 tests、0 failure）
- [x] 差分・ユーザー変更の非干渉を確認

## 動的確認

- [ ] stopped targetでentry overrideが同一IDにだけ発火
- [ ] prediction lease中に速度referenceがlease開始時速度を超えない
- [ ] fresh prediction復帰後に通常Pass速度へ復帰
- [ ] target discontinuity/corridor/wall/solverではleaseせず従来どおり離脱

## 検証記録

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功（25/25）
- `colcon test-result --verbose`: 956 tests、0 errors、0 failures
  - 既存build成果物 `build/joycon_contract_guard/package.xml` 欠損の読み取り警告あり。対象packageのtest成否には影響なし。
- `git diff --check`: 成功
- 既存の `aichallenge/result-summary.json` は変更していない
