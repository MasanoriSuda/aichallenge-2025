# Tasklist

- [x] 直近走行のSafeSeparation失敗経路を確認
- [x] 既存dynamic Mission waitとlast-feasible rescueを確認
- [x] requirements/design作成
- [x] no-return後のsame-side dynamic wait admissionを追加
- [x] no-return後のalternate replacementを禁止
- [x] paused fresh same-side candidate評価を旧predicted overlapから分離
- [x] committed same-side atomic replacementを有効化
- [x] fresh candidate置換時の縦距離Emergency優先順位を調整
- [x] core単体テスト追加・実行
- [x] `make autoware-build`
- [x] 差分確認

## 動的確認

- [ ] `dynamic mission wait entered`がsoft Pass failureで出る
- [ ] `fresh same-side PassPlan replaced`後にShiftOut/Passへ復帰する
- [ ] no-return後に反対側へ横断しない
- [ ] actual wall/body overlap/solver hard faultはRecoveryを維持する
- [ ] SafetyBrake、Recovery、Pass完遂率を`20260812-214501`と比較する
