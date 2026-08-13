# Tasklist

- [x] `20260813-112615` と前回runを比較
- [x] full-speed実行と2.0 m/s rolloutの不一致を特定
- [x] speed coupling純粋関数とテストを追加
- [x] runtime rear-clear rolloutへ統合
- [x] local/cloud設定を同期
- [x] 完遂予測ログを追加
- [x] `make autoware-build`
- [x] package test / `colcon test-result --verbose`（1073 tests, 0 errors, 0 failures）
- [ ] `make dev2` で効果確認（ユーザー実施）

## Definition of Done

- full-speed Passのrolloutがコース速度cap込みの実行速度を使用する
- hard faultまたは予測非分離時はnominal closingへ戻る
- 40 m / 10 sの絶対上限が維持される
- build/testが成功する
