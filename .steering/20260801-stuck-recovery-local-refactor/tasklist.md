# Task List

- [x] 最新ログと現行復帰コードの責務混在を確認する。
- [x] 動作不変の分離境界を決める。
- [x] Rejoin進捗トラッカーを切り出す。
- [x] 復帰候補方向方針を切り出す。
- [x] 既存論理を固定する単体テストを追加する。
- [x] 対象packageのテストとビルドを実行する。
- [x] `git diff --check`とインターフェース非変更を確認する。

## Static Verification

- `make autoware-build`: 25 packages succeeded.
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test targets succeeded.
- `colcon test-result --verbose`: 761 tests, 0 errors, 0 failures, 0 skipped.
- `git diff --check`: succeeded.
- ROS topic/service/message、launch、config、評価systemの変更なし。

`colcon test-result`には既存のstaleな
`build/joycon_contract_guard/package.xml`参照警告が出たが、コマンドは成功し、
対象packageのtest resultに失敗はない。

## Dynamic Verification

今回の変更は動作不変の局所リファクタリングであり、性能効果は判定しない。
次の性能修正後に`make dev2`でP1/P2接触復帰を再現する。
