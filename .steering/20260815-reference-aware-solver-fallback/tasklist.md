# Tasklist

- [x] 最新ログからsolver失敗後の中立操舵と壁接触の時系列を確認
- [x] fallback操舵のhold／path-track／neutralizeを分離
- [x] 任意目標向けrate limiterへ局所リファクタリング
- [x] configコメントと起動ログを更新
- [x] 単体テスト（25/25 targets、1177 tests、0 failure）
- [x] `make autoware-build`
- [x] 変更をコミット

## Definition of Done

- カーブ中の通常solver fallbackが`path-track`を選ぶ。
- Recovery中は`neutralize`を維持する。
- rate limiterの正負反転を単体テストで確認する。
- 対象packageがビルドできる。
