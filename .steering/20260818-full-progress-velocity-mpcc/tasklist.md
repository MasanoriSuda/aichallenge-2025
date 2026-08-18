# Tasklist

- [x] 現行3状態MPCC、persistent OSQP、速度参照の境界を確認
- [x] 5状態3入力Frenet線形化を実装
- [x] stage速度reference/terminal速度costを実装
- [x] 既存MPC問題から拡張QPを構築
- [x] 拡張solver・warm start・legacy変換・fallbackを統合
- [x] configと診断ログを追加
- [x] 単体テストを追加・実行
- [x] package buildを実行
- [x] results.mdを記録
- [x] 変更をコミット

## 動的確認（ユーザー実施）

- [ ] `make dev2` で2台走行を実施
- [ ] ShiftOut/Pass中の中間速度、rear-clear時間、接触・壁逸走を確認
- [ ] 拡張QPから3状態QPへのfallback頻度を確認
