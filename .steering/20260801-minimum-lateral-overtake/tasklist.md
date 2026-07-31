# Tasklist

- [x] 現行gap plannerとOvertakeLine entryを確認
- [x] 要件・設計を記録
- [x] 最小横目標の純粋関数を実装
- [x] 最小横移動side選択を実装
- [x] 通常Overtake candidate評価へ統合
- [x] 基準ライン非重複時のdirect Pass entryを実装
- [x] 設定と診断ログを追加
- [x] 単体テストを追加・実行
- [x] `make autoware-build`
- [x] 試走判定項目を記録

## 検証結果

- `git diff --check`: 成功
- `make autoware-build`: 成功（25 packages）
- `multi_purpose_mpc_ros`全テスト: 713 tests、0 failures
- 最小横移動テスト: 7 tests、全成功
- 動的効果確認: 未実施（`make dev2`で実施）

## 試走判定項目

- `minimum-motion=base-line` entryで`Idle -> Pass`となり、横目標が約0 mであること。
- 横移動が必要なentryで、goalが候補区間中央ではなく0に近い端になること。
- 候補なしの区間で新規`ShiftOut -> Recovery`が発生せずFollowを維持すること。
- 物理接触や実行中の壁違反ではRecoveryが残っていること。
- 6周のRecovery回数、接触、Overtake完遂数、lap timeを変更前と比較すること。
