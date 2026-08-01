# Tasklist

- [x] 現行mission FSM、entry preflight、interface契約を確認
- [x] 要件・設計・非対象範囲を定義
- [x] 全区間mission path profile policyを追加
- [x] entry preflightをShiftOut/Pass/Return一体検査へ変更
- [x] fixed goalをtarget lateral更新から分離
- [x] FollowPrepareの時間・距離上限を追加
- [x] rear-clear latchと完了target再捕捉抑止を追加
- [x] YAML読込・検証・debug logを追加
- [x] 単体テストを追加
- [x] package build/testを実行
- [x] 差分レビューと動的確認項目を記録

## 検証結果

- `make autoware-build`: 成功（25 packages）。
- review後の`multi_purpose_mpc_ros`再build: 成功。
- `colcon test --packages-select multi_purpose_mpc_ros`: 成功。
- `colcon test-result --verbose`: 792 tests、0 errors、0 failures、0 skipped。
  - 既存の`build/joycon_contract_guard/package.xml`欠損警告は継続するが、対象packageの
    test failureはない。
- `git diff --check`: 成功。
- 動的`make dev2`走行は未実施。効果確認は下記ログ指標で行う。

## 動的確認（ユーザー試走）

- `FollowPrepare`連続滞在が4秒/20 mを超えない。
- entry後の`corridor_goal`がtarget lateral揺れで変化しない。
- entry preflight reject後はRecoveryではなくFollowを継続する。
- rear-clear後に同一targetへ`Return -> Pass`しない。
- `Pass -> Return -> Idle`完遂数、wall Recovery数、penaltyを比較する。
