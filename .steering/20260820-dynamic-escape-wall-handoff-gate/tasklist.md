# Tasklist

- [x] `20260820-160913` の因果列を確認する
- [x] 要件・安全契約・ログ設計を記録する
- [x] wall handoff admission gateを純粋ロジックとして追加する
- [x] final control sourceへwall handoff holdを追加する
- [x] 最終予測評価と指令保持をROS nodeへ統合する
- [x] 拒否したMPC control historyを公開指令へ同期する
- [x] entry/block/requalifying/releaseログを追加する
- [x] 単体テストを追加する
- [x] package build/testを実行する
- [x] 差分を確認してコミットする

## 動的確認（ユーザー試走）

- [ ] `predicted-wall-contact` で危険なracing-line採用が拒否される
- [ ] 拒否後に実footprint wall contactへ到達しない
- [ ] 連続した有効予測で通常MPCへ復帰する
- [ ] 保留が過剰失速・長期停止を作らない
