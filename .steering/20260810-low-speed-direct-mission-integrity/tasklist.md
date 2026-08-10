# Tasklist

- [x] 現HEADと既存変更を確認する
- [x] LowSpeedDirectのfront→side境界とside更新箇所を確認する
- [x] pass side commit判定を純粋関数化する
- [x] LowSpeedDirect target IDをMission stateへ追加する
- [x] target-aware retained Pass検証を追加する
- [x] reset/handoff時にtarget IDをclearする
- [x] 単体テストを追加する
- [x] 対象packageをビルドする
- [x] 対象単体テストを実行する
- [x] 実走確認項目を整理する

## 検証結果

- `make autoware-build`: 成功（25 packages）
- `test_v2x_overtake_core`: 447 tests passed
- `colcon test-result`: 951 tests, 0 errors, 0 failures
  - 既存の `joycon_contract_guard/package.xml` 欠損に関するskip警告のみ

## 実走で確認するログ

- `Low-speed pass retained for side completion`
- `Low-speed retained Pass rejected: ... reason=...`
- `Low-speed direct Pass kept committed side` が出た場合に全幅切返しが発生しないこと
- front→side→rear-clear→Rejoinが同一target IDで完了すること
