# Tasklist

- [x] 現行 SafeSeparation / Mission cache / wall preplan を確認する
- [x] 参加者・評価インターフェース契約への非影響を確認する
- [x] safe trajectory prefix の pure policy と unit test を追加する
- [x] SafeSeparation の progress extension と fresh Mission 再利用へ統合する
- [x] predictive wall preplan を追加する
- [x] overtake episode ID を主要ログへ追加する
- [x] config の読み込み・検証・起動時表示を追加する
- [x] core unit test を実行する
- [x] `make autoware-build` を実行する
- [x] 差分をレビューする

## Verification

- `make autoware-build`: 成功（25 packages）
- `ctest -R '^test_v2x_overtake_core$' --output-on-failure`: 成功（1/1）
- `git diff --check`: 成功
- 動的効果確認: `make dev2` による実走を依頼
