# Tasklist

- [x] 現行side-replan評価とcommit経路を整理する
- [x] 変更範囲と非対象を記録する
- [x] Pass候補共通評価をcoreへ追加する
- [x] 左右候補比較をcoreへ追加する
- [x] debounce更新をcoreへ追加する
- [x] controllerの重複計算を置換する
- [x] core単体テストを追加する
- [x] 対象packageをbuild/testする
- [x] 動的確認項目を記録する

## 静的確認結果

- `make autoware-build`: 成功（25 package）
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功
- `colcon test-result --verbose`: 934 tests、0 errors、0 failures、0 skipped
- `build/joycon_contract_guard/package.xml` 不在の既知警告は今回の対象外
- `git diff --check`: 問題なし

## 次回の動的確認

本変更は候補評価・比較・debounceの構造整理のみで、速度値、判定閾値、FSM遷移は変更していない。次回の`make dev2`では、同じ走行条件で以下がリファクタリング前と同等であることを確認する。

- opponent-side replanのrequested / pending / committed / released回数
- ShiftOut / Pass / Return / Recoveryの遷移回数
- side選択、physical reserve、debounce時間
- Pass成功率、接触、壁Recovery、ラップ時間
