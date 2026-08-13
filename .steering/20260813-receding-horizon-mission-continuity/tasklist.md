# Tasklist

- [x] `20260813-105959` の遷移と receding-horizon 稼働状況を確認
- [x] Behavior ownership / SafeSeparation / Return reacquire の原因箇所を特定
- [x] receding-horizon execution lease の純粋関数とテストを追加
- [x] Behavior ShiftOut/Pass ownershipへleaseを統合
- [x] SafeSeparationのtarget dropout/forward escapeへleaseを統合
- [x] 一時optimizer failure時のlast-feasible horizon保持を追加
- [x] runtime wall ReturnをFinishReturn所有へ変更
- [x] fallback理由とlease状態を周期ログへ追加
- [x] local/cloud設定を同期
- [x] `make autoware-build`
- [x] package test / `colcon test-result --verbose`（1071 tests, 0 errors, 0 failures）
- [ ] `make dev2` で効果確認（ユーザー実施）

## Definition of Done

- ShiftOut/Pass中の入口候補棄却によるBehaviorチャタリングが抑制される
- 0.30秒以内の単発V2X欠落で `SafeSeparation aborted: invalid input` へ落ちない
- receding-horizon lease中のSafeSeparationがfull-speed forward escapeを選択できる
- hard wall/emergency/solver faultはleaseを無効化する
- runtime wall Returnが同周期付近でPassへ戻らない
- build/testが成功する
