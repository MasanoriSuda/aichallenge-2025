# Tasklist

- [x] 最新ログと Recovery ownership を照合する
- [x] pure handoff resolver と単体テストを追加する
- [x] pre-arm / committed Mission のハンドオフ条件を統合する
- [x] Reverse stop -> Drive request -> release を段階化する
- [x] Overtake Mission を保持した Recovery 終了処理を追加する
- [x] `test_stuck_recovery_core` を実行する
- [x] `multi_purpose_mpc_ros` をビルドする
- [ ] 実走で効果確認する

## Definition of Done

- 協調停止 Recovery 中に前進 Mission が成立した場合、LowSpeedRejoin を待たずに
  Overtake へ戻れる。
- Reverse 中は停止と Drive report を経由する。
- hard failure では従来 Recovery を維持する。
- 単体テストと package build が成功する。

## 実走確認

- candidate armed から direct handoff までの時間
- Reverse距離と LowSpeedRejoin滞在時間が従来より短くなること
- direct handoff後に `Follow -> Overtake` または committed Overtake 継続となること
- Reverse gear のまま正加速指令を出さないこと
- wall / solver / current overlap悪化時に direct handoffしないこと

## 静的検証結果

- `make autoware-build`: 成功（25 packages）
- `test_stuck_recovery_core`: 125 / 125 成功
- 新規 `StuckRecoveryCoordination.ForwardOvertakeHandoff*`: 2 / 2 成功
- `git diff --check`: 成功
- host の `clang-format` は未導入のため単独チェック未実施
