# Tasklist

- [x] 最新ログでcompletion-proof迂回を確認
- [x] 影響するMPCC新規entry経路を特定
- [x] prefix execution request/resolutionへcompletion-proof契約を統合
- [x] asynchronous resultとdual MPCC authorityへ共通契約を接続
- [x] cached prefix leaseへ共通契約を接続
- [x] 採用元と拒否理由のログを追加
- [x] complete Missionとactive Mission継続が回帰しないテストを追加
- [x] 対象単体テスト（29/29、1398 assertions/tests相当でfailure 0）
- [x] `make autoware-build`（25 packages成功）
- [x] 差分レビュー
- [x] コミット（この変更セット）

## 実装結果

- 新規progressive prefixの完遂証明を
  `resolve_mpcc_lite_prefix_execution()`へ統合した。
- `async-result`、`dual-current`、`cached-fresh`の3経路が同じ純粋関数を
  通過する。
- 実行assessmentへの反映を`apply_mpcc_entry_execution_contract`へ集約し、
  sourceとcandidate種別を遷移理由へ残す。
- 拒否ログは`prefix_reason`とcompletion-proofの詳細を1秒throttleで出す。
