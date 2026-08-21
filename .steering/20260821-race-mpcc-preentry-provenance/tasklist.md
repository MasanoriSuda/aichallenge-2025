# Task list

- [x] 最新試走ログと上位ログの構造を照合する
- [x] provenance lifecycle と検証規則を実装する
- [x] Entry 前の selected target observation を保存する
- [x] certificate / async lease / shadow log を単一 helper へ統合する
- [x] async lease の破棄理由を構造化する
- [x] provenance / async lease の単体テストを追加する
- [x] package build / test を実行する
- [ ] 実走で `Observed -> Locked`、branch attempt、async adopt を確認する
- [x] 差分レビューを行う（コミットは本tasklistを含めて実施）

## Verification

- `make autoware-build`: 成功（25 packages）
- `test_race_mpcc_foundation`: 6 / 6 成功
- `test_v2x_overtake_core`: 792 / 792 成功
- 次回実走ログでは `Race MPCC shadow` の `target_provenance` stage、左右 `attempted`、`Overtake MPCC-lite async` の `lease` と `adopted` を確認する。
