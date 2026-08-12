# Tasklist

- [x] 現行minimum-motion横目標と直近実測横離隔を確認
- [x] requirements/design作成
- [x] straight/outer追加clearance設定をconfig/cloud configへ追加
- [x] 追加横目標と通常fallback横目標を生成
- [x] 完全Mission・追加壁reserveを通った候補だけ優先
- [x] innerおよび最終左右戦略比較を変更しない
- [x] 起動ログ・Mission選択ログを追加
- [x] README更新
- [x] 単体テスト追加・実行（511 tests passed）
- [x] `make autoware-build`（25 packages successful）
- [x] 差分・設定一致確認

## 動的確認

- [ ] `clearance_bias=1/0.10/0`が直線・外側成功候補に出る
- [ ] 狭い区間で`clearance_bias=1/0/1`となり従来候補へ戻る
- [ ] イン差しは`clearance_bias=0/0/0`
- [ ] Pass完遂率、横接触、壁Recovery、追い越し時間を前走と比較
