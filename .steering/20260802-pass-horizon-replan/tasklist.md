# Tasklist

## 設計

- [x] 最新runのPass遷移と失敗理由を整理する
- [x] 既存progress watchdogとの差を確認する
- [x] rear-clearまでのend-to-end mission案を定義する
- [x] runtime horizon decisionを定義する
- [x] Proレビュー用の論点を作成する

## Core

- [ ] rear-clear時刻・距離をkinematic rolloutへ追加する
- [ ] dynamic Pass距離の上限・validation reserveを純粋関数化する
- [ ] `resolve_committed_pass_horizon()` を追加する
- [ ] mission extensionのatomic update requestを定義する

## Tests

- [ ] 低速対象をrear-clearできるcandidateを確認する
- [ ] 最大時間内にrear-clear不能なcandidateを棄却する
- [ ] validation lead到達前はKeepになることを確認する
- [ ] validation lead到達後はRefreshSameSideになることを確認する
- [ ] rear-clear済みかつReturn corridor成立でReturnになることを確認する
- [ ] opposite-sideだけ成立してもmid-Pass横断しないことを確認する
- [ ] Return不可・短区間安全ならHoldFrozenLineになることを確認する
- [ ] absolute Pass上限超過で無期限保持しないことを確認する

## Controller integration

- [ ] candidate Pass距離をpredicted rear-clear基準へ変更する
- [ ] Returnを含む静的壁・動的corridor preflightを追加する
- [ ] validated Pass距離・時間をmission stateへ保存する
- [ ] same-side runtime extensionを接続する
- [ ] Return / Hold / Abortを既存FSMへ接続する
- [ ] 状態変化ログを追加する

## Verification

- [ ] `git diff --check`
- [ ] `make autoware-build`
- [ ] 追い越しコアテスト
- [ ] `make dev2` で6周以上実施する
- [ ] ShiftOut -> Pass到達率が現行9/9相当を維持する
- [ ] Pass走行距離がvalidated horizonを超えないことを確認する
- [ ] Pass -> Return完遂数、SafetyBrake、wall、solver failureを比較する
- [ ] crash/wallペナルティが増えた場合は採用しない
