# Tasklist

- [x] 最新走行ログと失敗遷移を確認
- [x] gap plannerと全区間preflightの不整合を特定
- [x] 要件・設計・非対象範囲を定義
- [x] dynamic mission corridor policyを追加
- [x] entry側へdynamic goal intervalを接続
- [x] frozen mission中のside replan/preflightを停止
- [x] 単体テストを追加
- [x] package build/testを実行
- [x] 差分レビューと動的確認項目を記録

## 検証結果

- `make autoware-build`: 成功（25 packages）。
- 最終補強後の`multi_purpose_mpc_ros`単体build: 成功。
- `colcon test --packages-select multi_purpose_mpc_ros`: 25/25 test成功。
- `colcon test-result --verbose`: 796 tests、0 errors、0 failures、0 skipped。
  - 既存の`build/joycon_contract_guard/package.xml`欠損警告は継続するが、対象packageの
    test failureはない。
- 最終補強後の`test_v2x_overtake_core`: 成功。
- `git diff --check`: 成功。
- 動的`make dev2`走行は未実施。

## 動的確認（実装後）

- `full mission execution preflight rejected`がcommit後に発生しない。
- `dynamic mission corridor validated`を経たentryだけが`Idle -> ShiftOut`する。
- `Pass -> FollowPrepare, reason=committed pass paused by safety brake`が減る。
- `body_clear=0`へ入る前に必要横離隔が成立する。
- `Pass -> Return -> Idle`完遂率とpenaltyを比較する。
