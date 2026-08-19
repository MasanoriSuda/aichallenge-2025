# Tasklist

- [x] `20260819-201236` の残存solver failureを時系列確認
- [x] global quarantineの適用箇所を確認
- [x] target/side scoped backoff coreを追加
- [x] 同一side失敗時の反対side再評価を追加
- [x] 成功時の候補単位resetを追加
- [x] config / 起動ログ / authorityログを更新
- [x] core単体テストを追加
- [x] `git diff --check`
- [x] `test_v2x_overtake_core`を含むpackage全28テスト
- [x] `make autoware-build`
- [x] 変更をコミット

## 動的確認

- 同じsideの連続失敗ログが `hold=0.50, 1.00, 2.00, 4.00` となること
- `alternate=1/1` では反対sideがtracking authorityを取得すること
- `follow_cap_suppressed=1` は `qualified=1` の場合だけであること
- 旧試走のような約0.5秒周期の10回連続solver failureが消えること
- `Pass -> Return -> Idle` 完遂数、wall接触、callback overrunを併記すること
