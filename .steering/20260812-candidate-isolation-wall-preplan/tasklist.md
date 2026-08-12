# Tasklist

- [x] 直近走行と現行candidate selectorの失敗経路を確認
- [x] requirements/design作成
- [x] candidate-local fault isolationを実装
- [x] runtime wall warning samplingを実装
- [x] fresh same-side atomic replacementを実装
- [x] cross-side rejection retry throttleを実装
- [x] config/config_for_cloudへパラメータ追加
- [x] core単体テスト追加
- [x] 対象package単体テスト
- [x] `make autoware-build`
- [x] 差分レビュー

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 1056 tests、0 errors、0 failures
- `git diff --check`: 問題なし

## 動的確認

- [ ] `runtime wall preplan warning`がhard wall違反より前に出る
- [ ] fresh same-side Missionがある場合に置換される
- [ ] actual wall contact/margin violationはRecoveryを維持する
- [ ] 同一alternate棄却ログが40 Hzで連打されない
- [ ] Pass完遂率と壁Recoveryを`20260812-224210`から比較する
